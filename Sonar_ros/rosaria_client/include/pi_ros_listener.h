//for sonar data
#include <sensor_msgs/PointCloud.h>  
//for pose
#include <nav_msgs/Odometry.h>

#include "sonar_mapping.h"
#include <msg_utils/pi_msg_adaptor.h>
#include <glog/logging.h>

#include <thread>
#include <mutex>

const double sonarX[OurSonarNum]   = {0.069,0.114,0.148,0.166,0.166,0.148,0.114,0.069,-0.157,-0.203,-0.237,-0.255,-0.255,-0.237,-0.203,-0.157};
const double sonarY[OurSonarNum]   = {0.136,0.119,0.078,0.027,-0.027,-0.078,-0.119,-0.136,-0.136,-0.119,-0.078,-0.027,0.027,0.078,0.119,0.136};
const double sonarTh[OurSonarNum]  = {90 ,50 ,30 ,10 ,-10 ,-30 ,-50 ,-90 ,-90 ,-130,-150,-170,170,150,130,90};

//subscribing to a topic using a class method as the callback.
class PiRosListener
{
  public:
    ros::Time           SonarFrameStamp;
    std::string         SonarFrameId;
    PIRoBot::SensorScan Sonarscan;
    bool                FlagStartSonar = false;
    PIRoBot::RobotPose  WheelPose;


    PiRosListener();
    void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void poseCallback(const nav_msgs::Odometry& msg);

  private:

};

