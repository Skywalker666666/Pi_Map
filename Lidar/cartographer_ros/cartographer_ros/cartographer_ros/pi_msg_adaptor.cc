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
        std::cout << "url: " << chl.url << ", filter: " << chl.filter << std::endl;
        int fd = -1; 
        /*  Create the socket. */
        fd = nn_socket (AF_SP, chl.type);
        if (fd < 0) {
            fprintf (stderr, "nn_socket: %s\n", nn_strerror (nn_errno ()));
            return -1;
        }

        switch(chl.type)
        {
            case NN_SUB:
                if (nn_setsockopt (fd, NN_SUB, NN_SUB_SUBSCRIBE, chl.filter.c_str(), strlen(chl.filter.c_str())) < 0) {
                    fprintf (stderr, "nn_setsockopt: %s\n", nn_strerror (nn_errno ()));
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

        return fd;
    }

    bool PIMsgAdaptor::isPackageValid(char* msg, int len)
    {
        p_pi_msg_envelope_t p_msg = (p_pi_msg_envelope_t)(msg);

        return (SIZEOF_PI_MSG_ENVELOPE_HEADER + p_msg->length) == len;
    }

    void PIMsgAdaptor::subReceiveLoop()
    {
        int rc;
        char *msg;
        while(!recv_stop)
        {
            for (std::vector<PIMsgChannelInfo>::iterator ch = m_channels_map[NN_SUB].begin(); ch != m_channels_map[NN_SUB].end(); ++ch)
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
                        m_sig_pub(ch->handler, (p_pi_msg_envelope_t)(msg), msg + offsetof(pi_msg_envelope_t, data), ((p_pi_msg_envelope_t)msg)->length);    
                    }

                    nn_freemsg(msg);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            usleep(10*1000);
        }

        std::cout << "systemview pubrecv exit!" << std::endl;
    }

    void PIMsgAdaptor::repReceiveLoop()
    {
        int rc;
        char *msg;
        while(!recv_stop)
        {
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
        char msg[SIZEOF_PI_MSG_ENVELOPE_HEADER + len];
        int rc;

        memcpy (msg, p_env, SIZEOF_PI_MSG_ENVELOPE_HEADER);
        memcpy (msg + offsetof(pi_msg_envelope_t, data), body, len);

        rc = nn_send (handler, msg, sizeof (msg), 0);
        if (rc < 0) {
            fprintf (stderr, "nn_send: %s\n", nn_strerror (nn_errno ()));
            return PI_MSG_SEND_STATUS_FAILED;
        }  

        if(callback != nullptr)
        {
            m_p_once_recv_thread = new std::thread(&PIMsgAdaptor::recvOnce, this, handler, callback);
            m_p_once_recv_thread->detach();
        }

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