#include "sonar_mapping.h"

#include <msg_utils/pi_msg_adaptor.h>
#include <glog/logging.h>

#include <thread>
#include <mutex>


//receive cartographer pose
class PiCartoPoseRev
{
  public:
    PIRoBot::RobotPose  LidarPose;

    std::mutex          mutexExitThread;
    bool                isExitThread;

    bool                isSensorDataNew = false; //for pose update

    std::mutex          mutexCartoPose;

    PiCartoPoseRev();
    void AddCallback(); //for nanomsg

    void StartPoseThread();
    void EndPoseThread();
    void Run();
    
    // pose transmit counter from Lidar side 
    int pose_tx_cnt_rcv_;
    // node id from local scan matching
    int node_id_in_local_;
    
    using int64 = int64_t;
    
    int64 time_stamp_tx_;    
    

  private:

    std::thread * pPoseThread;

    // for nanomsg
    //std::shared_ptr<PIAUTO::msg::PIMsgAdaptor> msgPubAdp;
    std::shared_ptr<PIAUTO::msg::PIMsgAdaptor> msgSubAdp;
    //int posPubHandler;
    int posSubHandler;
};

