#include <ros/ros.h>

//#include "my_kbhit.h"
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
#include <sstream>

#include "pi_ros_listener.h"
#include "pi_carto_pose_rev.h"

#include "cartographer/common/port.h"


using namespace std;
using namespace cv;


DEFINE_string(scan_raw_path, "",
              "Filename of a scan raw data.");
DEFINE_string(pose_raw_path, "", 
	      "Filename of pose raw data.");
DEFINE_string(sonar_2d_ogm_path, "", 
	      "Filename of output map.");

DEFINE_string(orig_node_pose_path, "", 
	      "Filename of original nodes poses.");

DEFINE_string(opti_node_pose_path, "", 
	      "Filename of partial global optimized nodes poses.");

DEFINE_string(final_node_pose_path, "", 
	      "Filename of final nodes poses.");

DEFINE_string(pose_type, "", 
	      "type of poses.");

DEFINE_string(sonar_map_algo_option, "", 
	      "type of sonar mapping algorithm.");


DEFINE_string(final_node_pose_interpo_path, "", 
	      "Filename of final nodes poses interpolation.");


ofstream ogm_txt_file;
ofstream final_node_pose_interpo_file;
//ofstream botSonarFile;
//ofstream poseFile;

//ifstream inbotSonarFile; 
//ifstream inposeFile; 


//bool FLAG_DISPLAY = false;
bool FLAG_DISPLAY = true;


void quit(int sig)
{
  //tcsetattr(kfd, TCSANOW, &cooked);
  //ros::shutdown();
  exit(0);
}


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_scan_raw_path.empty()) << "-scan_raw_path is missing.";
  CHECK(!FLAGS_pose_raw_path.empty()) << "-pose_raw_path is missing.";
  CHECK(!FLAGS_orig_node_pose_path.empty()) << "-orig_node_pose_path is missing.";
  CHECK(!FLAGS_opti_node_pose_path.empty()) << "-opti_node_pose_path is missing.";
  CHECK(!FLAGS_final_node_pose_path.empty()) << "-final_node_pose_path is missing.";
  
  std::string path_temp = FLAGS_scan_raw_path;
  char* scan_raw_path = &path_temp[0];

  std::string path_temp2 = FLAGS_pose_raw_path;  
  char* pose_raw_path    = &path_temp2[0];
  //char* sonar_2d_ogm_path  = (char *)"/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/sonar_2d_ogm.txt";
  
  std::string path_temp3 = FLAGS_sonar_2d_ogm_path;  
  char* sonar_2d_ogm_path= &path_temp3[0];
  
  std::string path_temp4 = FLAGS_orig_node_pose_path;  
  char* orig_node_pose_path= &path_temp4[0];
  
  std::string path_temp5 = FLAGS_opti_node_pose_path;  
  char* opti_node_pose_path= &path_temp5[0]; 

  std::string path_temp6 = FLAGS_final_node_pose_path;  
  char* final_node_pose_path= &path_temp6[0]; 
  
  std::string path_temp7 = FLAGS_final_node_pose_interpo_path;
  char* final_node_pose_interpo_path= &path_temp7[0];
  
  PIRoBot::SonarMapping mapping(FLAGS_sonar_map_algo_option);
  mapping.Init(0.05, 80);
  mapping.StartMappingThread();
  
  // Initialization of pOgm
  double Ogm_init_val;
//#ifdef DS_ALGORITHM
  if (FLAGS_sonar_map_algo_option == "DS" ) {
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

  //ros::init(argc, argv, "RosAria_sonars");
  //ros::NodeHandle nh;
 
  //ros::Rate loop_rate(10); //10Hz to trigger call back, this frequency should bigger than pub speed.

  //ROS_INFO_STREAM("Sonar Node: The robot is ready for sonar scan and pose!");
  
  ////scan of sonar
  PIRoBot::SensorScan sscan;
  ////pose of robot
  PIRoBot::RobotPose  p3dx_pose;

  // used to collect sonar scan
  vector<PIRoBot::SensorScan> sonarVec;
  vector<PIRoBot::RobotPose> poseVec;

  //collect all final pose to collection
  vector<PIRoBot::RobotPose> poseCollectionVec;  
  vector<PIRoBot::RobotPose> poseVecOrig;
  vector<PIRoBot::RobotPose> poseVecOpti;
  // absolute pose for nodes
  vector<PIRoBot::RobotPose> poseVecFinal;
  // pose increment between nodes, for Final pose interpolation.
  vector<PIRoBot::RobotPose> poseVecFinalInterp;
  vector<PIRoBot::RobotPose> poseVecFinalDelta;
  vector<int> nDeltaVec;
  vector<int> nDeltaVec2;

  //collect all original pose to collection
  std::unordered_map<int64, PIRoBot::RobotPose> timePoseCollectionUMap; 
  // only for Final pose
  std::map<int64, PIRoBot::RobotPose> timePoseCollecMap; 

  // for Final pose interpolation, collect pose increment between nodes
  std::map<int64, PIRoBot::RobotPose> timePoseDeltaCollecMap;  
  
  
  vector<int> txcntVec;  
  vector<int> nodeidVec;
  vector<int64> timestpVec;
  
  
  p3dx_pose.x  = 0;
  p3dx_pose.y  = 0;
  p3dx_pose.th = 0;

  PIRoBot::ScanData sonar_scan_temp;

  std::ifstream inbotSonarFile(scan_raw_path); //default is ios::in
  std::ifstream inposeFile(pose_raw_path); //default is ios::in
  std::ifstream inorigposeFile(orig_node_pose_path); //default is ios::in
  std::ifstream inoptiposeFile(opti_node_pose_path); //default is ios::in
  std::ifstream infinalposeFile(final_node_pose_path); //default is ios::in
  
  std::string line;
  std::vector<std::string> tokens;

  while(std::getline(inbotSonarFile, line)) {     // '\n' is the default delimiter
    std::istringstream iss(line);
    std::string token;
    tokens.clear();
    while(std::getline(iss, token, ' ')){   // but we can specify a different one, like ' ' space
        tokens.push_back(token);
    }
    
    sscan.clear();
    for(int s_id = 0; s_id < OurSonarNum; s_id++){
      sonar_scan_temp.sx  = sonarX[s_id];
      sonar_scan_temp.sy  = sonarY[s_id];
      sonar_scan_temp.th  = sonarTh[s_id] * M_PI / 180;
      sonar_scan_temp.len = std::stod(tokens[s_id + 4]);
      sscan.push_back(sonar_scan_temp);
    }
    sonarVec.push_back(sscan);   
  }  
  
  // pose from online
  while(std::getline(inposeFile, line)) {     // '\n' is the default delimiter
    std::istringstream iss(line);
    std::string token;
    tokens.clear();
    while(std::getline(iss, token, ' ')){   // but we can specify a different one, like ' ' space
        tokens.push_back(token);
    }
    // cos(), sin(): Returns the cosine of an angle of x radians.
    p3dx_pose = PIRoBot::RobotPose(std::stod(tokens[4]) - 0.1 * cos(std::stod(tokens[6])), 
				   std::stod(tokens[5]) - 0.1 * sin(std::stod(tokens[6])), 
				   std::stod(tokens[6]));
    // when running in the online, no chassis center offset.
    
    poseVec.push_back(p3dx_pose);
    // nodeid is the id of node in pose graph
    nodeidVec.push_back(std::stod(tokens[2]));
    timestpVec.push_back(std::strtoll(tokens[3].c_str(), NULL, 10));
  }  

  
  
  if(FLAGS_pose_type == "LPF"){  //Local Pose 
    // pose from original local info
    while(std::getline(inorigposeFile, line)) {  // '\n' is the default delimiter
      std::istringstream iss(line);
      std::string token;
      tokens.clear();
      while(std::getline(iss, token, ' ')){  // but we can specify a different one, like ' ' space
        tokens.push_back(token);
      }
      p3dx_pose = PIRoBot::RobotPose(std::stod(tokens[3]) - 0.1 * cos(std::stod(tokens[5])), 
				     std::stod(tokens[4]) - 0.1 * sin(std::stod(tokens[5])), 
				     std::stod(tokens[5]));
    
      int64 key_time = std::strtoll(tokens[1].c_str(), NULL, 10);
      if (timePoseCollectionUMap.find(key_time) == timePoseCollectionUMap.end()) {
        // method 1:
        timePoseCollectionUMap[key_time] = p3dx_pose;
	// method 2:
        //timePoseCollectionUMap.insert({key_time, p3dx_pose});
      }
    }
  
    // assign pose vector Original
    for(int i=0; i<timestpVec.size(); i++){
      // TO DO: Here, we may need to check timestamp membership in two files.
      // But it should be ok, because our system will terminate teh sonar side receiver first
      // before teminating lidar side transmit.
      poseVecOrig.push_back(timePoseCollectionUMap.at(timestpVec[i]));
    }
  }
  else if(FLAGS_pose_type == "PGPF"){  //Partial Global Pose
    // pose from optimal global info
    while(std::getline(inoptiposeFile, line)) {  // '\n' is the default delimiter
      std::istringstream iss(line);
      std::string token;
      tokens.clear();
      while(std::getline(iss, token, ' ')){  // but we can specify a different one, like ' ' space
        tokens.push_back(token);
      }
      p3dx_pose = PIRoBot::RobotPose(std::stod(tokens[3]) - 0.1 * cos(std::stod(tokens[5])), 
				     std::stod(tokens[4]) - 0.1 * sin(std::stod(tokens[5])), 
				     std::stod(tokens[5]));
    
      int64 key_time = std::strtoll(tokens[1].c_str(), NULL, 10);
      // TO DO, the repetation info can be utilized for interpolation, but we will pay more attention
      // to the interpolation of Final pose first.
      if (timePoseCollectionUMap.find(key_time) == timePoseCollectionUMap.end()) {
        // method 1:
        timePoseCollectionUMap[key_time] = p3dx_pose;
	// method 2:
        //timePoseCollectionUMap.insert({key_time, p3dx_pose});
      }
    }
  
    // assign pose vector Optimal
    for(int i=0; i<timestpVec.size(); i++){
      // TO DO: Here, we may need to check timestamp membership in two files.
      // But it should be ok, because our system will terminate teh sonar side receiver first
      // before teminating lidar side transmit.
      poseVecOpti.push_back(timePoseCollectionUMap.at(timestpVec[i]));
    }     
  }
  else if(FLAGS_pose_type == "FGPF"){  //Final Global Pose
    //method 1:
    /*
    // pose from final optimization
    while(std::getline(infinalposeFile, line)) {     // '\n' is the default delimiter
      std::istringstream iss(line);
      std::string token;
      tokens.clear();
      while(std::getline(iss, token, ' ')){   // but we can specify a different one, like ' ' space
        tokens.push_back(token);
      }
      p3dx_pose = PIRoBot::RobotPose(std::stod(tokens[3]) - 0.1 * cos(std::stod(tokens[5])), 
				     std::stod(tokens[4]) - 0.1 * sin(std::stod(tokens[5])), 
				     std::stod(tokens[5]));

      poseCollectionVec.push_back(p3dx_pose);
    }  
  
    // assign pose vector Final
    for(int i=0; i<nodeidVec.size(); i++) {
      poseVecFinal.push_back(poseCollectionVec[nodeidVec[i]]);
    } 
    */

    //method 2:
    // pose from final optimization
    while(std::getline(infinalposeFile, line)) {     // '\n' is the default delimiter
      std::istringstream iss(line);
      std::string token;
      tokens.clear();
      while(std::getline(iss, token, ' ')){   // but we can specify a different one, like ' ' space
        tokens.push_back(token);
      }
      p3dx_pose = PIRoBot::RobotPose(std::stod(tokens[3]) - 0.1 * cos(std::stod(tokens[5])), 
				     std::stod(tokens[4]) - 0.1 * sin(std::stod(tokens[5])), 
				     std::stod(tokens[5]));
      int64 key_time = std::strtoll(tokens[1].c_str(), NULL, 10);

      //poseCollectionVec.push_back(p3dx_pose);
      if (timePoseCollecMap.find(key_time) == timePoseCollecMap.end()) {
        // method 1:
        timePoseCollecMap[key_time] = p3dx_pose;
	// method 2:
        //timePoseCollecMap.insert({key_time, p3dx_pose});
      }      
    }  
  
    // assign pose vector Final
    for(int i=0; i<timestpVec.size(); i++) {
      for (auto it = timePoseCollecMap.begin(); it != timePoseCollecMap.end(); ++it) {
        //std::cout << it->first << ", " << it->second << '\n';
	if (it->first >= timestpVec[i]) {
	  poseVecFinal.push_back(it->second);
	  break;
	}
      }
    }    
  }
  else if(FLAGS_pose_type == "FGPFINTP"){  //Final Global Pose interpolation
    int start_flag = 0;
    int n_delta = 0;
    // before interpolation
    PIRoBot::RobotPose final_p3dx_pose;
    // interpolation increment
    PIRoBot::RobotPose delta_p3dx_pose;
    // after interpolation    
    PIRoBot::RobotPose interpolated_p3dx_pose;
    
    PIRoBot::RobotPose last_p3dx_pose = PIRoBot::RobotPose(0.0, 0.0, 0.0);
    // pose from final optimization
    while(std::getline(infinalposeFile, line)) {     // '\n' is the default delimiter
      std::istringstream iss(line);
      std::string token;
      tokens.clear();
      while(std::getline(iss, token, ' ')){   // but we can specify a different one, like ' ' space
        tokens.push_back(token);
      }
      p3dx_pose = PIRoBot::RobotPose(std::stod(tokens[3]) - 0.1 * cos(std::stod(tokens[5])), 
				     std::stod(tokens[4]) - 0.1 * sin(std::stod(tokens[5])), 
				     std::stod(tokens[5]));
      int64 key_time = std::strtoll(tokens[1].c_str(), NULL, 10);
      
      //poseCollectionVec.push_back(p3dx_pose);
      if (timePoseCollecMap.find(key_time) == timePoseCollecMap.end()) {
        timePoseCollecMap[key_time] = p3dx_pose;
	timePoseDeltaCollecMap[key_time] = p3dx_pose - last_p3dx_pose;
	last_p3dx_pose = p3dx_pose;
      }      
    }  
  
    // assign pose vector Final
    for (int i=0; i<timestpVec.size(); i++) {
      // compare with last element of time stamp in final pose file.
      if ( timestpVec[i] <= timePoseCollecMap.rbegin()->first ){ // we execute the interpolation.
        //TO DO, not needed to start from beginning every time, cuz timstpVec is monotonic
        for (auto it = timePoseCollecMap.begin(); it != timePoseCollecMap.end(); ++it) {
          // std::cout << it->first << ", " << it->second << '\n';
          if (it->first >= timestpVec[i]) {
            // check pose "it->second", if it is new one, counter back to 0.
            final_p3dx_pose = it->second;
            
            // new pose, not found in the history.
            if ( std::find(poseVecFinal.begin(), poseVecFinal.end(), final_p3dx_pose) == poseVecFinal.end() ) {
              if (start_flag == 0){
                // first pose. set the flag.
                start_flag = 1;
              }
              else{
                // not first pose
                for (int k = 0; k <= n_delta; k++) {
          	  nDeltaVec2.push_back(n_delta + 1);
                }	  
              }	    
              n_delta = 0;
            }
            else {// found in the history.
              n_delta += 1;
            }
            
            nDeltaVec.push_back(n_delta);
            poseVecFinal.push_back(final_p3dx_pose);
            poseVecFinalDelta.push_back(timePoseDeltaCollecMap[it->first]);
            break;
          }
        }// timePoseCollecMap for loop
        
        /*
        // tail completeness, version 1.
        if(i == timestpVec.size() - 1) {
          for (int k = 0; k <= n_delta; k++) {
            nDeltaVec2.push_back(1 + n_delta);
          }// k for loop
        }// tail if
        */
      }
      else{ // we copy the original value, no interpolation. better tail section handling. version 2
        if (start_flag == 1){
          // last pose has been interpolated. reset the flag.
          start_flag = 0;
          for (int k = 0; k <= n_delta; k++) {
            nDeltaVec2.push_back(n_delta + 1);
          }//k for loop
        }// if start_flag
        nDeltaVec.push_back(1);// or we can put 0 here.
        nDeltaVec2.push_back(1);
        poseVecFinal.push_back(final_p3dx_pose);
        poseVecFinalDelta.push_back(timePoseDeltaCollecMap.rbegin()->second);
      } 

    }//timestpVec for loop   
    
    for (int i=0; i<timestpVec.size(); i++) {
      final_p3dx_pose = poseVecFinal[i];
      delta_p3dx_pose = poseVecFinalDelta[i];
      n_delta = nDeltaVec[i];
      int total_sub_n_delta = nDeltaVec2[i];
      //std::cout << "n_delta: " << n_delta << " total_sub_n_delta: " << total_sub_n_delta << std::endl;
      double interp_ratio = static_cast<double>(n_delta)/static_cast<double>(total_sub_n_delta);
      interpolated_p3dx_pose.x = final_p3dx_pose.x +  interp_ratio * delta_p3dx_pose.x;  
      interpolated_p3dx_pose.y = final_p3dx_pose.y +  interp_ratio * delta_p3dx_pose.y;
      // should be very careful about the theta interpolation, cuze it is only -pi to pi.
      //interpolated_p3dx_pose.th = final_p3dx_pose.th +  interp_ratio * delta_p3dx_pose.th;
      interpolated_p3dx_pose.th = final_p3dx_pose.th;
      poseVecFinalInterp.push_back(interpolated_p3dx_pose);	    
      //std::cout << "interpolated_p3dx_pose.x: " << interpolated_p3dx_pose.x << std::endl;
    }
    
    // save the result of interpolation.
    int interpo_id = 0;
    final_node_pose_interpo_file.open(final_node_pose_interpo_path, ios::out);
    for (int j=0; j<poseVecFinalInterp.size(); j++) {
      const PIRoBot::RobotPose & inter_pose = poseVecFinalInterp[j];
      final_node_pose_interpo_file << interpo_id << " " 
      << timestpVec[j] << " " 
      << inter_pose.x  << " " 
      << inter_pose.y  << " " 
      << inter_pose.th << std::endl;
      interpo_id += 1;
    }
    final_node_pose_interpo_file.close();
    
    
  }//else if(FLAGS_pose_type == "FGPF_INTP")

  
  int last_updated_node_id = 0;
  for(int i=0; i<sonarVec.size(); i++) {
    //To Do: check the size of sonarVec and poseVec, should be same
    //read the file
 
    sscan = sonarVec[i];
    
    if(FLAGS_pose_type == "LPF"){
      //std::cout << "mode is: " << FLAGS_pose_type << std::endl; 
      p3dx_pose  = poseVecOrig[i];
    }
    else if(FLAGS_pose_type == "PGPF"){
      p3dx_pose  = poseVecOpti[i];
    }
    else if(FLAGS_pose_type == "FGPF"){
      p3dx_pose  = poseVecFinal[i]; 
    }
    else if(FLAGS_pose_type == "FGPFINTP"){
      p3dx_pose  = poseVecFinalInterp[i]; 
    }    
    
 
    //std::cout << "subscriber return: " << sonar_sub << std::endl;
    //std::cout << "Stamp of Frame: " << slistener.SonarFrameStamp << std::endl;
    //std::cout << "ID of Frame: "    << slistener.SonarFrameId    << std::endl;
    //std::cout << "yaw in degeree: " << yaw << std::endl;

    //if (slistener.FlagStartSonar) {
    //if ( nodeidVec[i] != last_updated_node_id) {
      std::cout << std::setprecision(2) << std::fixed << 
      " node id: " << nodeidVec[i] <<
      " Current position=(" << p3dx_pose.x << ", " << p3dx_pose.y << ") " <<
      "Current direction=" << std::setprecision(2) << std::fixed << p3dx_pose.th << std::endl;
         
        
      mapping.UpdateSensor(sscan, p3dx_pose);

      //sonarVec.push_back(sscan);
      //poseVec.push_back(p3dx_pose);

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
      last_updated_node_id = nodeidVec[i];
    //}// nodeidVec
    //ros::spinOnce();
    //loop_rate.sleep();
  }
  
//  #ifdef DS_ALGORITHM
if (FLAGS_sonar_map_algo_option == "DS" ) {
  //"DS_ALGORITHM"
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
else {
//#else
  ogm_txt_file.open(sonar_2d_ogm_path, ios::out);
  for (int x = 0; x < (mapping.pOgm->sq_size() * 20); x++) {
    for (int y = 0; y < (mapping.pOgm->sq_size() * 20); y++) {
      double cellValue = mapping.pOgm->GetVal(x, y);
      ogm_txt_file << x << "," << y << "," << cellValue << std::endl;
    }
  }
  ogm_txt_file.close();
  std::cout << "ogm map file finished saving."<< std::endl;
}
  //#endif
  
  //User prompt
  std::string AnyKeys;
  std::cout << "Press AnyKeys to go on!!!";
  std::getline(std::cin, AnyKeys);
  std::cout << AnyKeys << " is received, Bye Bye!" << std::endl;  
    
  
  
  //cartoporeceiver.EndPoseThread();
  mapping.EndMappingThread();
  signal(SIGINT,quit);
  return 0;
}

