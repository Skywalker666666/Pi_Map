#include "pi_ros_listener.h"
#include "ros/ros.h"

#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/reqrep.h>

#include <ctime>
#include <chrono>

//#define USE_WHEEL_ODOMETRY_POS


//subscribing to a topic using a class method as the callback.
PiRosListener::PiRosListener()
{
}

void PiRosListener::sonarCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
  FlagStartSonar = true;
  SonarFrameStamp = msg->header.stamp;//Getting the Current Frame Time.  SonarFrameId  = msg->header.frame_id;//frame_id_sonar;
  SonarFrameId    = msg->header.frame_id;//frame_id_sonar;


  geometry_msgs::Point32 ps;
  PIRoBot::ScanData sonar_scan_temp;

  // sonar data unpacking method 1:
  Sonarscan.clear();
  for(int s_id = 0; s_id < msg->points.size(); s_id++){
    ps = msg->points[s_id];
    sonar_scan_temp.sx  = sonarX[s_id];
    sonar_scan_temp.sy  = sonarY[s_id];
    sonar_scan_temp.th  = sonarTh[s_id] * M_PI / 180;
    sonar_scan_temp.len = ps.z;
    Sonarscan.push_back(sonar_scan_temp);
    //Sonarscan[s_id] = sonar_temp; //wrong way
  }

  // sonar data unpacking method 2:
  /*
  for(ps : msg->points)
  {
    std::cout << ps.x << std::endl;
  }
  */
}

void PiRosListener::poseCallback(const nav_msgs::Odometry& msg)
{
  double robotX, robotY, robotTh;

  double qw = msg.pose.pose.orientation.w;
  double qx = msg.pose.pose.orientation.x;
  double qy = msg.pose.pose.orientation.y;
  double qz = msg.pose.pose.orientation.z;
  double siny = +2.0 * (qw * qz + qx * qy);
  double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);
  //double yaw = atan2(siny, cosy)*(180.0/M_PI); //for debug
  double yaw = atan2(siny, cosy);

  /* output the pose information using standard output */
  /* std::cout << std::setprecision(2) << std::fixed << 
    "Current position=(" << msg.pose.pose.position.x << ", " << msg.pose.pose.position.y << ") " <<
    "Current direction=" << std::setprecision(2) << std::fixed << msg.pose.pose.orientation.w<<"\r" << std::endl;
  std::flush(std::cout); */

  robotX = msg.pose.pose.position.x;
  robotY = msg.pose.pose.position.y;
  robotTh = yaw;
  WheelPose = PIRoBot::RobotPose(robotX, robotY, robotTh);

  //std::cout << "yaw in degeree: " << yaw << std::endl;
}






