//============================================================================
// Copyright   : PerceptIn
//============================================================================

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sys/time.h>
#include <glog/logging.h>
#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/reqrep.h>
#include "sonar_mapping.h"

#include <ctime>
#include <chrono>

//#define USE_WHEEL_ODOMETRY_POS
#define M_PI		3.14159265358979323846	/* pi */

// down sampling the pose from Lidar
long int pose_cnt = 0;
// calculate Update frequency
std::chrono::high_resolution_clock::time_point pose_time = {};
std::chrono::high_resolution_clock::time_point pose_time2 = {};
std::chrono::high_resolution_clock::time_point pose_time3 = {};

std::ofstream lidarPosFile;

namespace PIRoBot {
SonarMapping::SonarMapping():pMappingThread(nullptr), isExitThread(false)
{
  std::string addr = "ipc:///tmp/pos_data.ipc";
  msgPubAdp = std::make_shared<PIAUTO::msg::PIMsgAdaptor>();
  msgSubAdp = std::make_shared<PIAUTO::msg::PIMsgAdaptor>();
  posPubHandler = msgPubAdp->registerOneMessageChannel(
      {NN_PUB, addr.c_str(), "robot_pos", -1});
  posSubHandler = msgSubAdp->registerOneMessageChannel(
      {NN_SUB, addr.c_str(), "robot_pos", -1});
  // wait nanomsg ready
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void SonarMapping::AddCallback()
{
 
  // callback register
  msgSubAdp->addSubscriberToSubMsg([&](int handler,
                                       p_pi_msg_envelope_t p_env,
                                       const char* body, unsigned int len) {
    if (p_env->type == PIMSG_ROBOT_POS_2D) {

#ifdef USE_WHEEL_ODOMETRY_POS
      // recv pos info from nanomsg
      double robotX, robotY, robotTh;
      memcpy(&robotX, body, sizeof(double));
      memcpy(&robotY, body + sizeof(double), sizeof(double));
      memcpy(&robotTh, body + sizeof(double) * 2, sizeof(double));
#else // POS from Lidar
      // recv pos info from nanomsg
      double robotX, robotY, robotTh;
      double rx, ry, rz, qw, qx, qy, qz;
      memcpy(&rx, body, sizeof(double));
      memcpy(&ry, body + sizeof(double), sizeof(double));
      memcpy(&rz, body + sizeof(double) * 2, sizeof(double));
      memcpy(&qw, body + sizeof(double) * 3, sizeof(double));
      memcpy(&qx, body + sizeof(double) * 4, sizeof(double));
      memcpy(&qy, body + sizeof(double) * 5, sizeof(double));
      memcpy(&qz, body + sizeof(double) * 6, sizeof(double));

      double siny = +2.0 * (qw * qz+ qx * qy);
      double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);
      //double yaw = atan2(siny, cosy)*(180.0/M_PI);
      double yaw = atan2(siny, cosy);

      robotX = rx; //meter
      robotY = ry; //meter
      robotTh = yaw; //radian (Rad)
      //robotTh = yaw / 10; //huwei remark: the th of lidar is not accurate???
#endif

      std::chrono::high_resolution_clock::time_point pose_time_last3;
      pose_time_last3 = pose_time3;
      pose_time3 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed3 = pose_time3 - pose_time_last3;
      std::cout << "Receive duration: " << elapsed3.count() << " seconds." << std::endl;
      std::cout << "Receive Frequency: " << 1/elapsed3.count() << " Hz." << std::endl;


      std::cout << "robotX " << robotX << " robotY " << robotY << " robotTh " << robotTh << std::endl;

      RobotPose _pose = RobotPose(robotX, robotY, robotTh);

      // recv sonar info from callback
      SensorScan _scan;
      bool _isNewSensorData;
      {
        std::lock_guard<std::mutex> lock(mutexSonarScan);
        _isNewSensorData = bNewSensorData;
        if(bNewSensorData) {
          bNewSensorData = false;
          _scan = sonar_scan;
        }
      }
      std::cout << "_isNewSensorData: " << _isNewSensorData << std::endl;
      if(!_isNewSensorData) {
        std::cout << "Entered if !_isNew,  and bNewSensorData: " << bNewSensorData << std::endl;
        return 0;
      }
      std::cout << "Not entered if !_isNew,  and bNewSensorData: " << bNewSensorData << std::endl;

      UpdateMap(_scan, _pose);
      return 1;
    }
  });
}

void SonarMapping::UpdateSensor(const SensorScan & _scan, const RobotPose & _pose) {
  std::lock_guard<std::mutex> lock(mutexSonarScan);
  sonar_scan = _scan;
  robot_pose = _pose;
  bNewSensorData = true;
  

  std::chrono::high_resolution_clock::time_point pose_time_last2;
  pose_time_last2 = pose_time2;
  pose_time2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed2 = pose_time2 - pose_time_last2;
  std::cout << "UpdateSensor duration: " << elapsed2.count() << " seconds." << std::endl;
  std::cout << "UpdateSensor Frequency: " << 1/elapsed2.count() << " Hz." << std::endl;

#ifdef USE_WHEEL_ODOMETRY_POS
  // send the robot pose by nanomsg
  int msgSize = sizeof(double) * 3;
  char msgBuf[msgSize];
  memcpy(msgBuf, &(robot_pose.x), sizeof(double));
  memcpy(msgBuf + sizeof(double), &(robot_pose.y), sizeof(double));
  memcpy(msgBuf + sizeof(double) * 2, &(robot_pose.th), sizeof(double));

  pi_msg_envelope_t env;
  memcpy(env.filter, "robot_pos", sizeof("robot_pos"));
  env.type = PIMSG_ROBOT_POS_2D;
  env.length = msgSize;

  int rc = msgPubAdp->send(posPubHandler, &env, msgBuf, msgSize, nullptr);

  std::cout << "pos is send, result: " << rc << " [0 is ok]" << std::endl;

  //TODO: sleep time as params?
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
#endif //USE_WHEEL_ODOMETRY_POS
}

void SonarMapping::StartMappingThread() {
  LOG(INFO)<<"start SonarMapping thread";
  pMappingThread = new std::thread(&SonarMapping::Run, this);
}

void SonarMapping::EndMappingThread() {
  LOG(INFO)<<"end mapping thread";
  pMappingThread->join();
  delete pMappingThread;
  pMappingThread = nullptr;
}

void SonarMapping::Run()  {

  msgSubAdp->startRecvLoop();

  while(1) {
    mutexExitThread.lock();
    if(isExitThread) {
      mutexExitThread.unlock();
      break;
    }
    mutexExitThread.unlock();

    //this->RunOnce();
    usleep(5000);
  }

  LOG(INFO)<<"exit mapping thread";
}

//void SonarMapping::RunOnce() {
//  SensorScan _scan;
//  RobotPose _pose;
//  bool _isNewSensorData;
//  {
//    std::lock_guard<std::mutex> lock(mutexSonarScan);
//    _isNewSensorData = bNewSensorData;
//    if(bNewSensorData) {
//      bNewSensorData = false;
//      _scan = sonar_scan;
//      _pose = robot_pose;
//
//    }
//  }
//  if(!_isNewSensorData) {
//    return;
//  }
//  UpdateMap(_scan, _pose);
//}

// given two points formating a line, calculate the point on the line segment
void SonarMapping::CalcLinePassTwoPt(const PntIdx & start, const PntIdx & end, std::vector<PntIdx> & line) {
  line.clear();
  size_t y_delta = (end.y_idx > start.y_idx)
    ? (end.y_idx - start.y_idx) : (start.y_idx - end.y_idx);
  size_t x_delta = (end.x_idx > start.x_idx)
    ? (end.x_idx - start.x_idx) : (start.x_idx - end.x_idx);
  if (y_delta < x_delta) {
    double k
      = (static_cast<double>(end.y_idx) - static_cast<double>(start.y_idx))
      / (static_cast<double>(end.x_idx) - static_cast<double>(start.x_idx));
    int dx = (end.x_idx > start.x_idx) ? 1 : -1; // +1 or -1;
    PntIdx cur = start;
    line.push_back(cur);
    while(cur.x_idx != end.x_idx) {
      cur.x_idx += dx;
      cur.y_idx = start.y_idx + static_cast<int>((static_cast<double>(cur.x_idx) - static_cast<double>(start.x_idx))*k); // may be negative;
      line.push_back(cur);
    }
  }
  else {
    double k
      = (static_cast<double>(end.x_idx) - static_cast<double>(start.x_idx))
      / (static_cast<double>(end.y_idx) - static_cast<double>(start.y_idx));
    int dy = (end.y_idx > start.y_idx) ? 1 : -1; // +1 or -1;
    PntIdx cur = start;
    line.push_back(cur);
    while(cur.y_idx != end.y_idx) {
      cur.y_idx += dy;
      cur.x_idx = start.x_idx + static_cast<int>((static_cast<double>(cur.y_idx) - static_cast<double>(start.y_idx))*k); // may be negative;
      line.push_back(cur);
    }
  }
}

void SonarMapping::UpdateMap(const SensorScan & _scan, const RobotPose & _pose) {
  /*pose_cnt++;
  if(pose_cnt % 2 != 0) {
    //std::cout << "directly return, no update: " << pose_cnt << std::endl;
    return;
  }*/
  //std::cout << "do update: " << pose_cnt << std::endl;
 
  std::chrono::high_resolution_clock::time_point pose_time_last;
  pose_time_last = pose_time;
  pose_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = pose_time - pose_time_last;
  std::cout << "duration: " << elapsed.count() << " seconds." << std::endl;
  std::cout << "Frequency: " << 1/elapsed.count() << " Hz." << std::endl;

  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  // millisecond

  lidarPosFile.open("our_lidar_data.txt", std::fstream::out | std::fstream::app);
  lidarPosFile << ms << " " << std::fixed << std::setprecision(10) << _pose.x << " " << _pose.y << " " << _pose.th << std::endl;
  lidarPosFile.close();
 
  const double max_sonar_dist_threshold = 4.5;
  VLOG(1)<<"sonar update map";
  std::cout << "sonar update map" << std::endl;
  for(int i=0; i<_scan.size(); i++) {
    ScanData _sonar = _scan[i];
    if(_sonar.len > max_sonar_dist_threshold) {
      continue;
    }
    double startPtMapX, startPtMapY;
    double endPtMapX, endPtMapY;
    double curHeading;
    startPtMapX = (_sonar.sx * cos(_pose.th) - _sonar.sy * sin(_pose.th)) + _pose.x;
    startPtMapY = (_sonar.sx * sin(_pose.th) + _sonar.sy * cos(_pose.th)) + _pose.y;
    curHeading = _sonar.th + _pose.th;
    const double expandLen = 0.05 * 5;
    endPtMapX = startPtMapX + (_sonar.len + expandLen) * cos(curHeading);
    endPtMapY = startPtMapY + (_sonar.len + expandLen) * sin(curHeading);
    PntIdx startPtOgmIdx, endPtOgmIdx;
    pOgm->CalcIdx(startPtMapX, startPtMapY, &startPtOgmIdx);
    pOgm->CalcIdx(endPtMapX, endPtMapY, &endPtOgmIdx);
    std::vector<PntIdx> linePtIdx;
    CalcLinePassTwoPt(startPtOgmIdx, endPtOgmIdx, linePtIdx);
    double obsPtMapX, obsPtMapY;
    obsPtMapX = startPtMapX + _sonar.len * cos(curHeading);
    obsPtMapY = startPtMapY + _sonar.len * sin(curHeading);
    for(auto jt=linePtIdx.begin(); jt < linePtIdx.end(); jt++) {
      double ogmVal = pOgm->GetVal(*jt);
      if(ogmVal < 0) {
        LOG(ERROR)<<"point on line out bound of ogm"<<jt->x_idx<<" "<<jt->y_idx;
        continue;
      }
      double curPtMapX, curPtMapY;
      pOgm->CalcCoord(*jt, &curPtMapX, &curPtMapY); // may return False??
      double squareDistFromObs = pow(curPtMapX - obsPtMapX, 2) + pow(curPtMapY - obsPtMapY, 2);
      const double sigmaObs = 0.1; //0.25;
      const double squareSigmaObs = pow(sigmaObs, 2);
      double expectVal = exp(- squareDistFromObs / squareSigmaObs);
      if( linePtIdx.end() - jt < 5 ) {
        //LOG(INFO)<<"behind obstacle expectVal "<<expectVal;
        expectVal = std::max(expectVal, 0.5);
      }
      if(std::abs(ogmVal - 0.5) <  1e-6) {
        // old ogm val is unknown, so directly set a new val
        pOgm->SetVal(*jt, expectVal);
      }
      else {
        const double measureWeight = 0.1;
        double newVal = ogmVal * (1 - measureWeight) + expectVal * measureWeight;
        pOgm->SetVal(*jt, newVal);
      }
    }
  }
  VLOG(1)<<"sonar update map finish";
}

}
