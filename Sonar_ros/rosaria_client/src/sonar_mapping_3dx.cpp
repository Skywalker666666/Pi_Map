#include <ros/ros.h>

#include "my_kbhit.h"
#include "sonar_mapping.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>    // circle

#include "aux_func.h" //like a fix, scale and offset for coordinate conversion

#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <iostream>
#include <fstream>
#include <thread>

#include "pi_ros_listener.h"
#include "pi_carto_pose_rev.h"

using namespace std;
using namespace cv;

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_A 0x61
#define KEYCODE_Z 0x7A
#define KEYCODE_S 0x73
#define KEYCODE_X 0x78
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20

ofstream ogm_txt_file;
ofstream botSonarFile;
ofstream poseFile;

char* scan_raw_pose_path = (char *)"/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/scan_raw_data_sonar.txt";
char* pose_raw_path      = (char *)"/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/pose_raw_data.txt";
char* sonar_2d_ogm_path  = (char *)"/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/sonar_2d_ogm.txt";

bool FLAG_DISPLAY = false;
//bool FLAG_DISPLAY = true;

class TeleopRosAria
{
  public:
    TeleopRosAria();
    //void keyInit();
    bool keyProcess();
  private:
    ros::NodeHandle nh_;
    double linear_, angular_, l_scale_, a_scale_;
    double current_linear_, current_angular_, step_linear_, step_angular_;
    ros::Publisher twist_pub_;
};


TeleopRosAria::TeleopRosAria():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0),
  current_angular_(0.1),
  current_linear_(0.1),
  step_linear_(0.2),
  step_angular_(0.1)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
}

int kfd = 0;
//struct termios cooked, raw;

void quit(int sig)
{
  //tcsetattr(kfd, TCSANOW, &cooked);
  //ros::shutdown();
  exit(0);
}


/*
void TeleopRosAria::keyInit()
{
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");
  puts("Press the space bar to stop the robot.");
  puts("Press q to stop the program");
  puts("a/z - Increase/decrease linear velocity");
  puts("s/x - Increase/decrease angular velocity");
}
*/

bool TeleopRosAria::keyProcess()
{
  char c;
  bool dirty=false;
/*
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");
  puts("Press the space bar to stop the robot.");
  puts("Press q to stop the program");
  puts("a/z - Increase/decrease linear velocity");
  puts("s/x - Increase/decrease angular velocity");
*/
  //for(;;)
  //{
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
	  {
  	  perror("read():");
  	  exit(-1);
	  }

    //read(kfd, &c, 1);

    linear_=angular_=0;
    char printable[100];
    ROS_DEBUG("value: 0x%02X\n", c);
    switch(c)
	  {
    	case KEYCODE_L:
    	  ROS_DEBUG("LEFT");
    	  angular_ = current_angular_;
    	  linear_ = 0;
    	  dirty = true;
    	  break;
    	case KEYCODE_R:
    	  ROS_DEBUG("RIGHT");
    	  angular_ = -current_angular_;
    	  linear_ = 0;
    	  dirty = true;
    	  break;
    	case KEYCODE_U:
    	  ROS_DEBUG("UP");
    	  linear_ = current_linear_;
    	  angular_ = 0;
    	  dirty = true;
    	  break;
    	case KEYCODE_D:
    	  ROS_DEBUG("DOWN");
    	  linear_ = -current_linear_;
    	  angular_ = 0;
    	  dirty = true;
    	  break;
	case KEYCODE_A:
	  ROS_DEBUG("INCREASE LINEAR SPEED");
	  current_linear_ += step_linear_;
	  sprintf(printable, "Linear speed: %02f", current_linear_);
	  puts(printable);
	  dirty=true;
	  break;
	case KEYCODE_Z:
	  ROS_DEBUG("DECREASE LINEAR SPEED");
          current_linear_ -= step_linear_;
	  if(current_linear_ < 0)
	      current_linear_ = 0;
          sprintf(printable, "Linear speed: %02f", current_linear_);
          puts(printable);
          dirty=true;
          break;
	case KEYCODE_S:
	  ROS_DEBUG("INCREASE ANGULAR SPEED");
          current_angular_ += step_angular_;
          sprintf(printable, "Angular speed: %02f", current_angular_);
          puts(printable);
          dirty=true;
          break;
	case KEYCODE_X:
	  ROS_DEBUG("DECREASE ANGULAR SPEED");
          current_angular_ -= step_angular_;
	  if(current_angular_ < 0)
	      current_angular_ = 0;
          sprintf(printable, "Angular speed: %02f", current_angular_);
          puts(printable);
          dirty=true;
          break;
    	case KEYCODE_SPACE:
    	  ROS_DEBUG("STOP");
    	  linear_ = 0;
    	  angular_ = 0;
    	  dirty = true;
    	  break;
      case KEYCODE_Q:
        ROS_DEBUG("QUIT");
        ROS_INFO_STREAM("You quit the teleop successfully");
        return true;
        break;
    }
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(dirty ==true)
  	{
  	  twist_pub_.publish(twist);
  	  dirty=false;
  	}
  //}
  return false;
}

int main(int argc, char **argv)
{
  using int64 = int64_t;
  std::string xxx_sonar_map_algo_option = "CLD";
 
  
  PIRoBot::SonarMapping mapping(xxx_sonar_map_algo_option);
  mapping.Init(0.05, 80);
  mapping.StartMappingThread();
  
  // Initialization of pOgm
  double Ogm_init_val;
//#ifdef DS_ALGORITHM
if (xxx_sonar_map_algo_option == "DS") {
  //"DS_ALGORITHM"
  Ogm_init_val = 0.0;
}
else {
//#else
  Ogm_init_val = 0.5;
}
//#endif

  mapping.pOgm -> InitOgm(Ogm_init_val);//defalt vlue is 0.5
  mapping.pOgm2-> InitOgm(Ogm_init_val);//defalt vlue is 0.5
  // for DS algorithm, initial value is 0.5

  ros::init(argc, argv, "RosAria_sonars");
  ros::NodeHandle nh;
 
  ros::Rate loop_rate(10); //10Hz to trigger call back, this frequency should bigger than pub speed.

  ROS_INFO_STREAM("Sonar Node: The robot is ready for sonar scan and pose!");
  
  ////scan of sonar
  PIRoBot::SensorScan sscan;
  ////pose of robot
  PIRoBot::RobotPose  p3dx_pose;

  // used to collect sonar scan
  vector<PIRoBot::SensorScan> sonarVec;
  vector<PIRoBot::RobotPose> poseVec;
  vector<int> txcntVec;  
  vector<int> nodeidVec;
  vector<int64> timestpVec;
  
  p3dx_pose.x  = 0;
  p3dx_pose.y  = 0;
  p3dx_pose.th = 0;

  TeleopRosAria teleop_RosAria;
  //subscribing to a topic using a class method as the callback.
  PiRosListener slistener;
  PiCartoPoseRev cartoporeceiver;
 
  cartoporeceiver.AddCallback();
  cartoporeceiver.StartPoseThread();

  ros::Subscriber sonar_sub = nh.subscribe("RosAria/sonar", 50, &PiRosListener::sonarCallback, &slistener); //supply sonar reading

  //ros::Subscriber pose_sub  = nh.subscribe("RosAria/pose", 1000, &PiRosListener::poseCallback, &slistener); //supply pose

  init_kbhit();

  while (ros::ok())
  {
    //usleep(1000);

    if(kbhit()) {
      bool ret = teleop_RosAria.keyProcess();
      std::cout << "ret of key process: " << ret << std::endl;
      if(ret) {
        ros::shutdown();
        break;
      }
    }

    sscan = slistener.Sonarscan;

    // for test and turn off pose.
    /* 
    p3dx_pose.x  = 0;
    p3dx_pose.y  = 0;
    p3dx_pose.th = 0; 
    */
    
    cartoporeceiver.isSensorDataNew = true;
    
    //p3dx_pose  = slistener.WheelPose;
    p3dx_pose  = cartoporeceiver.LidarPose;
    //p3dx_pose.x = p3dx_pose.x - 0.1; 

    int pose_tx_counter = cartoporeceiver.pose_tx_cnt_rcv_;
    int node_id = cartoporeceiver.node_id_in_local_;
    int64 time_stamp_carto_tx = cartoporeceiver.time_stamp_tx_;
    
    //std::cout << "subscriber return: " << sonar_sub << std::endl;
    //std::cout << "Stamp of Frame: " << slistener.SonarFrameStamp << std::endl;
    //std::cout << "ID of Frame: "    << slistener.SonarFrameId    << std::endl;
    //std::cout << "yaw in degeree: " << yaw << std::endl;
    std::cout << std::setprecision(2) << std::fixed << 
    "tx_cnt: " << pose_tx_counter << " node id: " << node_id <<
    " Current position=(" << p3dx_pose.x << ", " << p3dx_pose.y << ") " <<
    "Current direction=" << std::setprecision(2) << std::fixed << p3dx_pose.th << std::endl;
    
    //std::cout << "Sonar No.: " << s_id << std::endl;
    //std::cout << "LocalX: "    << ps.x << std::endl;
    //std::cout << "LocalY: "    << ps.y << std::endl;
    //std::cout << "Range: "     << ps.z << std::endl;
    
    //std::cout << "FlagStartSonar: " << slistener.FlagStartSonar << std::endl;

    if (slistener.FlagStartSonar) {
      mapping.UpdateSensor(sscan, p3dx_pose);

      sonarVec.push_back(sscan);
      poseVec.push_back(p3dx_pose);
      txcntVec.push_back(pose_tx_counter);
      nodeidVec.push_back(node_id);
      timestpVec.push_back(time_stamp_carto_tx);
      
      // plot map
      if (FLAG_DISPLAY) {
        const cv::Point2d imgOriginOffsetInMap = cv::Point2d(-40.0, 40.0);
        const double img2mapScale = 0.05;
        #define MapCoord2ImgCoord(x, y) mapCoord2ImgCoord(x, y, img2mapScale, imgOriginOffsetInMap)
        //Visualize
        cv::Mat ogmMat = mapping.pOgm->Visualize();
        cv::namedWindow("ogm", cv::WINDOW_NORMAL);
        cv::Mat bgrOgmMat;
        cvtColor(ogmMat, bgrOgmMat, CV_GRAY2BGR);
        cv::Point2i ptImg;
        cv::Point3d pt(p3dx_pose.x,p3dx_pose.y, 0.0);
        MapCoord2ImgCoord(pt, ptImg);
        cv::circle(bgrOgmMat, ptImg, 3, Scalar(255,0,0), -1);
        cv::Point2i ptImg2;
        ptImg2.x = ptImg.x + 10 * cos(p3dx_pose.th);
        ptImg2.y = ptImg.y - 10 * sin(p3dx_pose.th);
        cv::arrowedLine(bgrOgmMat, ptImg, ptImg2, Scalar(0,255,0)); cv::imshow("ogm", bgrOgmMat);
        cv::waitKey(2);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  finish_kbhit();


  botSonarFile.open(scan_raw_pose_path, ios::out);
  for(int i=0; i<sonarVec.size(); i++) {
    const PIRoBot::SensorScan & _sensorscan = sonarVec[i];
    botSonarFile << i;
    botSonarFile << " " << txcntVec[i];
    botSonarFile << " " << nodeidVec[i];
    botSonarFile << " " << timestpVec[i];

    for(int k = 0; k < OurSonarNum; k++ ) {
      const PIRoBot::ScanData _sonar = _sensorscan[k];
      botSonarFile<<" "<<_sonar.len;
    }
    botSonarFile<<std::endl;
  }
  botSonarFile.close();


  poseFile.open(pose_raw_path, ios::out);
  for(int i=0; i<poseVec.size(); i++) {
    const PIRoBot::RobotPose & _pose = poseVec[i];
    poseFile << i << " " << txcntVec[i] << " " << nodeidVec[i] << " " << 
    timestpVec[i] << " "<< std::fixed << std::setprecision(10) << 
    _pose.x << " " << _pose.y << " " << _pose.th << std::endl;
    //poseFile << ms << " " << std::fixed << std::setprecision(10) << _pose.x << " " << _pose.y << " " << _pose.th << std::endl;
  }
  poseFile.close();

//#ifdef DS_ALGORITHM
if (xxx_sonar_map_algo_option == "DS" ) {
  // "DS_ALGORITHM"
  ogm_txt_file.open(sonar_2d_ogm_path, ios::out);
  for (int x = 0; x < (mapping.pOgm->sq_size() * 20); x++) {
    for (int y = 0; y < (mapping.pOgm->sq_size() * 20); y++) {
      double cellValue;
      double cellValueF = mapping.pOgm->GetVal(x, y);
      double cellValueE = mapping.pOgm2->GetVal(x, y);
      if (cellValueF == 0.0 && cellValueE == 0.0) {
        cellValue = 0.5;
      }
      else if (cellValueF >= cellValueE) {
        cellValue = 1.0;
      }
      else if (cellValueF < cellValueE) {
        cellValue = 0.0;
      }
      ogm_txt_file << x << "," << y << "," << cellValue << std::endl;
    }
  }
  ogm_txt_file.close();
  std::cout << "file finished."<< std::endl;
}
else{
//#else
  //ros::spin();
  ogm_txt_file.open(sonar_2d_ogm_path, ios::out);
  for (int x = 0; x < (mapping.pOgm->sq_size() * 20); x++) {
    for (int y = 0; y < (mapping.pOgm->sq_size() * 20); y++) {
      double cellValue = mapping.pOgm->GetVal(x, y);
      ogm_txt_file << x << "," << y << "," << cellValue << std::endl;
    }
  }
  ogm_txt_file.close();
  std::cout << "file finished."<< std::endl;
}
//#endif

  cartoporeceiver.EndPoseThread();
  mapping.EndMappingThread();
  signal(SIGINT,quit);
  return 0;
}

