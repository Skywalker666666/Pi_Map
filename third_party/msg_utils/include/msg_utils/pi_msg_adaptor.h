#ifndef PI_MSG_ADAPTOR_H
#define PI_MSG_ADAPTOR_H

#include <boost/signals2/signal.hpp>

#include <functional>
#include <thread>
#include <string>
#include <unordered_map>

#include "msg_utils/pi_msg_com.h"

namespace PIAUTO
{
namespace msg
{

struct PIMsgChannelInfo
{
    // channel type, sub/req, etc.
    int type;

    // url represents tcp url or ipc file path
    std::string url;

    // only message starts with this filter is accepted.
    // mainly used in sub/pub protocol
    std::string filter;

    // channel is not ok if handler is negative.
    int handler;

    // some other paras
    // TBD
};

class PIMsgAdaptor
{
    public:
        typedef std::unordered_map<int, std::vector<PIMsgChannelInfo> > channels_map_t;

        /* handler:     the channel id this message related to.
         * p_env:       message envelope
         * body:        exact message content
         * len:         body length
        */
        using pi_msg_callback_t = int(int handler, p_pi_msg_envelope_t p_env, const char* body, unsigned int len);
        
        PIMsgAdaptor();
        ~PIMsgAdaptor();

        // return non-negative channel handler if successed.
        int registerOneMessageChannel(PIMsgChannelInfo chl);

        void startRecvLoop();
        void stopRecvLoop();

        typedef boost::signals2::signal<pi_msg_callback_t> msg_signal_t;

        // if you registered to a sub protocol, then you can subscribe to it.
        boost::signals2::connection addSubscriberToSubMsg(const msg_signal_t::slot_type &subscriber);

        // if you registered to a rep protocol, then you can subscribe to it either.
        boost::signals2::connection addSubscriberToRepMsg(const msg_signal_t::slot_type &subscriber);

        // manage sending operation for protocols such as publish/reply etc.
        int send(int handler, p_pi_msg_envelope_t p_env, const char* body, unsigned int len, std::function<pi_msg_callback_t> callback);
    private:
        void subReceiveLoop();
        void repReceiveLoop();

        bool isPackageValid(char* msg, int len);

        void recvOnce(int handler, std::function<pi_msg_callback_t> callback);
    private:
        msg_signal_t m_sig_pub;
        msg_signal_t m_sig_req;

        bool recv_stop = false;

        // listen to publish message
        std::thread *m_p_subrecv_thread;

        // listen to request message
        std::thread *m_p_reprecv_thread;

        channels_map_t m_channels_map;

        std::thread *m_p_once_recv_thread;

};

}
}

#endif // PI_MSG_ADAPTOR_H