#include "msg_utils/pi_msg_adaptor.h"
#include <iostream>
#include <chrono>
#include <functional>

#include <nanomsg/nn.h>
#include <nanomsg/reqrep.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/pair.h>

namespace PIAUTO
{
  namespace msg
  {
    PIMsgAdaptor::PIMsgAdaptor():
        m_p_once_recv_thread(nullptr)
    {
      m_channels_map[NN_SUB] = std::vector<PIMsgChannelInfo>();
      m_channels_map[NN_PUB] = std::vector<PIMsgChannelInfo>();
      m_channels_map[NN_REP] = std::vector<PIMsgChannelInfo>();
      m_channels_map[NN_REQ] = std::vector<PIMsgChannelInfo>();
    }

    PIMsgAdaptor::~PIMsgAdaptor()
    {
      recv_stop = true;
    }

    void PIMsgAdaptor::startRecvLoop()
    {
      m_p_subrecv_thread = new std::thread(&PIMsgAdaptor::subReceiveLoop, this);
      m_p_reprecv_thread = new std::thread(&PIMsgAdaptor::repReceiveLoop, this);

      m_p_subrecv_thread->detach();
      m_p_reprecv_thread->detach();
    }

    void PIMsgAdaptor::stopRecvLoop()
    {
      recv_stop = true;
    }

    int PIMsgAdaptor::registerOneMessageChannel(PIMsgChannelInfo chl)
    {
      // Iterate and print keys and values of unordered_map
      std::cout << "\n url: " << chl.url << ", filter: " << chl.filter << std::endl;
      int fd = -1;
      /*  Create the socket. */
      fd = nn_socket (AF_SP, chl.type);
      if (fd < 0) {
        fprintf (stderr, "nn_socket: %s\n", nn_strerror (nn_errno ()));
        return -1;
      }

      int maxRcvSize = 6 * 1024 * 1024;//6MB

      switch(chl.type)
      {
        case NN_SUB:
          if (nn_setsockopt (fd, NN_SUB, NN_SUB_SUBSCRIBE, chl.filter.c_str(), strlen(chl.filter.c_str())) < 0) {
            fprintf (stderr, "nn_setsockopt NN_SUB_SUBSCRIBE: %s\n", nn_strerror (nn_errno ()));
            nn_close (fd);

            fd = -1;
            break;
          }
          if (nn_setsockopt (fd, NN_SOL_SOCKET, NN_RCVMAXSIZE, &maxRcvSize, sizeof(maxRcvSize)) < 0) {
            fprintf (stderr, "nn_setsockopt NN_RCVMAXSIZE: %s\n", nn_strerror (nn_errno ()));
            nn_close (fd);

            fd = -1;
            break;
          }
        case NN_REQ:
        {
          /*  Bind to the URL.  This will bind to the address and listen
              synchronously; new clients will be accepted asynchronously
              without further action from the calling program. */
          if (nn_connect (fd, chl.url.c_str()) < 0) {
            fprintf (stderr, "nn_connect: %s\n", nn_strerror (nn_errno ()));
            nn_close (fd);

            fd = -1;
            break;
          }
        }
          break;
        case NN_PUB:
        case NN_REP:
        {
          /*  Bind to the URL.  This will bind to the address and listen
              synchronously; new clients will be accepted asynchronously
              without further action from the calling program. */
          printf("%s\n", chl.url.c_str());

          if (nn_bind (fd, chl.url.c_str()) < 0) {
            fprintf (stderr, "nn_bind: %s\n", nn_strerror (nn_errno ()));
            nn_close (fd);

            fd = -1;
            break;
          }
        }
          break;
        default:
          break;
      }

      int timeout = 200;

      if (fd >= 0 && nn_setsockopt (fd, NN_SOL_SOCKET, NN_RCVTIMEO, &timeout, sizeof(int)) < 0) {
        fprintf (stderr, "nn_setsockopt: %s\n", nn_strerror (nn_errno ()));
        nn_close (fd);

        fd = -1;

        return -1;
      }

      if(fd >= 0)
      {
        chl.handler = fd;

        m_channels_map[chl.type].push_back(chl);
      }

      fprintf(stderr, "registerOneMessageChannel result %d\n", fd);

      return fd;
    }

    bool PIMsgAdaptor::isPackageValid(char* msg, int len)
    {
      p_pi_msg_envelope_t p_msg = (p_pi_msg_envelope_t)(msg);

      //fprintf(stderr, "SIZEOF_PI_MSG_ENVELOPE_HEADER %d, p_msg->length %d, len %d\n", SIZEOF_PI_MSG_ENVELOPE_HEADER, p_msg->length, len);

      return (SIZEOF_PI_MSG_ENVELOPE_HEADER + p_msg->length) == len;
    }

    void PIMsgAdaptor::subReceiveLoop()
    {
      struct nn_pollfd pfd [m_channels_map[NN_SUB].size()];
      for(size_t i=0; i< m_channels_map[NN_SUB].size(); i++ )
      {
        pfd [i].fd =m_channels_map[NN_SUB][i].handler;
        pfd [i].events = NN_POLLIN; // NN_POLLIN | NN_POLLOUT;
      }
      
      int rc;
      char *msg;
      while(!recv_stop)
      {

        rc = nn_poll (pfd, m_channels_map[NN_SUB].size(), 50);
        if (rc == 0) {
          //printf ("Timeout!\n");
          continue;
        }
        if (rc == -1) {
          printf ("Error!\n");
          continue;
        }

        for(size_t i=0; i< m_channels_map[NN_SUB].size(); i++ )
        {

          if (pfd [i].revents & NN_POLLIN) {
            //printf ("Message can be received from %d ! \n" , i);

            PIMsgChannelInfo ch = m_channels_map[NN_SUB][i];

            if(ch.handler < 0)
            {
              std::cout << "invalid handler\n" << std::endl;
              continue;
            }

            // need to check if new message arrived mannually.
            rc = nn_recv (ch.handler, &msg, NN_MSG, 0);

            if(rc < 0)
            {
              fprintf (stderr, "subReceiveLoop: nn_recv: %s\n", nn_strerror (nn_errno ()));
            }
            else
            {
              if(isPackageValid(msg, rc))
              {
                m_sig_pub(ch.handler, (p_pi_msg_envelope_t)(msg), msg + offsetof(pi_msg_envelope_t, data), ((p_pi_msg_envelope_t)msg)->length);
              }

              nn_freemsg(msg);
            }
          }

        }


        /*
        for (std::vector<PIMsgChannelInfo>::iterator ch = m_channels_map[NN_SUB].begin(); ch != m_channels_map[NN_SUB].end(); ++ch)
        {
          if(ch->handler < 0)
          {
            std::cout << "invalid handler" << std::endl;
            continue;
          }

          // need to check if new message arrived mannually.
          rc = nn_recv (ch->handler, &msg, NN_MSG, 0);

          if(rc < 0)
          {
            fprintf (stderr, "subReceiveLoop: nn_recv: %s\n", nn_strerror (nn_errno ()));
          }
          else
          {
            if(isPackageValid(msg, rc))
            {
              m_sig_pub(ch->handler, (p_pi_msg_envelope_t)(msg), msg + offsetof(pi_msg_envelope_t, data), ((p_pi_msg_envelope_t)msg)->length);
            }

            nn_freemsg(msg);
          }

          //std::this_thread::sleep_for(std::chrono::milliseconds(m_sleep_gap));
        }

        //std::this_thread::sleep_for(std::chrono::milliseconds(m_sleep_gap));
        */
      }

      std::cout << "systemview pubrecv exit!" << std::endl;
    }

    void PIMsgAdaptor::repReceiveLoop()
    {

      struct nn_pollfd pfd [m_channels_map[NN_REP].size()];
      for(size_t i=0; i< m_channels_map[NN_REP].size(); i++ )
      {
        pfd [i].fd =m_channels_map[NN_REP][i].handler;
        pfd [i].events = NN_POLLIN; // NN_POLLIN | NN_POLLOUT;
      }


      int rc;
      char *msg;
      while(!recv_stop)
      {

        rc = nn_poll (pfd, m_channels_map[NN_REP].size(), 50);
        if (rc == 0) {
          //printf ("Timeout!\n");
          continue;
        }
        if (rc == -1) {
          printf ("Error!\n");
          continue;
        }

        for(size_t i=0; i< m_channels_map[NN_REP].size(); i++ )
        {

          if (pfd [i].revents & NN_POLLIN) {
            //printf ("Message can be received from %d ! \n" , i);

            PIMsgChannelInfo ch = m_channels_map[NN_REP][i];

            if(ch.handler < 0)
            {
              std::cout << "invalid handler\n" << std::endl;
              continue;
            }

            // need to check if new message arrived mannually.
            rc = nn_recv (ch.handler, &msg, NN_MSG, 0);

            if(rc < 0)
            {
              fprintf (stderr, "subReceiveLoop: nn_recv: %s\n", nn_strerror (nn_errno ()));
            }
            else
            {
              if(isPackageValid(msg, rc))
              {
                m_sig_req(ch.handler, (p_pi_msg_envelope_t)(msg), msg + offsetof(pi_msg_envelope_t, data), ((p_pi_msg_envelope_t)msg)->length);
              }

              nn_freemsg(msg);
            }
          }

        }

        /*
        for (std::vector<PIMsgChannelInfo>::iterator ch = m_channels_map[NN_REP].begin(); ch != m_channels_map[NN_REP].end(); ++ch)
        {
          if(ch->handler < 0)
          {
            std::cout << "invalid handler" << std::endl;
            continue;
          }

          // need to check if new message arrived mannually.
          rc = nn_recv (ch->handler, &msg, NN_MSG, 0);

          if(rc > 0)
          {
            if(isPackageValid(msg, rc))
            {
              m_sig_req(ch->handler, (p_pi_msg_envelope_t)(msg), msg + offsetof(pi_msg_envelope_t, data), ((p_pi_msg_envelope_t)msg)->length);
            }

            nn_freemsg(msg);
          }

          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        */
      }

      std::cout << "systemview pubrecv exit!" << std::endl;
    }

    boost::signals2::connection PIMsgAdaptor::addSubscriberToSubMsg(const msg_signal_t::slot_type &subscriber)
    {
      return m_sig_pub.connect(subscriber);
    }

    boost::signals2::connection PIMsgAdaptor::addSubscriberToRepMsg(const msg_signal_t::slot_type &subscriber)
    {
      return m_sig_req.connect(subscriber);
    }

    int PIMsgAdaptor::send(int handler, p_pi_msg_envelope_t p_env, const char* body, unsigned int len, std::function<pi_msg_callback_t> callback)
    {
      //char *msg = new char[SIZEOF_PI_MSG_ENVELOPE_HEADER + len];
      void *msg = nn_allocmsg(SIZEOF_PI_MSG_ENVELOPE_HEADER + len, 0);
      int rc;

      if(msg == NULL)
      {
        fprintf (stderr, "PIMsgAdaptor::send() fail to alloc buffer");
        return PI_MSG_SEND_STATUS_FAILED;
      }

      memcpy (msg, p_env, SIZEOF_PI_MSG_ENVELOPE_HEADER);
      memcpy (msg + offsetof(pi_msg_envelope_t, data), body, len);

      rc = nn_send (handler, msg, SIZEOF_PI_MSG_ENVELOPE_HEADER + len, 0);
      if (rc < 0) {
        //delete[] msg;
        nn_freemsg(msg);
        fprintf (stderr, "nn_send: %s\n", nn_strerror (nn_errno ()));
        return PI_MSG_SEND_STATUS_FAILED;
      }

      //fprintf (stderr, "DEBUG nn_send need %d, send %d\n", static_cast<int>(SIZEOF_PI_MSG_ENVELOPE_HEADER + len), rc);

      if(callback != nullptr)
      {
        m_p_once_recv_thread = new std::thread(&PIMsgAdaptor::recvOnce, this, handler, callback);
        m_p_once_recv_thread->detach();
      }

      //delete[] msg;
      nn_freemsg(msg);
      return PI_MSG_SEND_STATUS_SUCCESS;
    }

    void PIMsgAdaptor::recvOnce(int handler, std::function<pi_msg_callback_t> callback)
    {
      int retry = 3;
      int rc = -1;
      char *msg;

      // need to check if new message arrived mannually.
      while(retry-- != 0 && rc < 0)
      {
        rc = nn_recv (handler, &msg, NN_MSG, 0);
      }

      if(rc > 0)
      {
        if(callback != nullptr)
        {
          callback(handler, (p_pi_msg_envelope_t)(msg), msg + offsetof(pi_msg_envelope_t, data), ((p_pi_msg_envelope_t)msg)->length);
        }

        nn_freemsg(msg);
      }
    }
  }

}