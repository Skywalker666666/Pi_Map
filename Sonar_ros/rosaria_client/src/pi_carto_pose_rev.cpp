#include "pi_carto_pose_rev.h"
#include "ros/ros.h"

#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/reqrep.h>

#include <ctime>
#include <chrono>


#include "cartographer/common/time.h"


// calculate Update frequency
//std::chrono::high_resolution_clock::time_point pose_time3 = {};
std::chrono::high_resolution_clock::time_point pose_time3 = std::chrono::high_resolution_clock::time_point::min();

using int64 = int64_t;

//subscribing to a topic using a class method as the callback.
PiCartoPoseRev::PiCartoPoseRev(): pPoseThread(nullptr), isExitThread(false)
{
  std::string addr = "ipc:///tmp/pos_data.ipc";
  //msgPubAdp = std::make_shared<PIAUTO::msg::PIMsgAdaptor>();
  msgSubAdp = std::make_shared<PIAUTO::msg::PIMsgAdaptor>();
  //posPubHandler = msgPubAdp->registerOneMessageChannel(
  //    {NN_PUB, addr.c_str(), "robot_pos", -1});
  posSubHandler = msgSubAdp->registerOneMessageChannel(
      {NN_SUB, addr.c_str(), "robot_pos", -1});
  // wait nanomsg ready
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}



void PiCartoPoseRev::AddCallback()
{
  // callback register
  msgSubAdp->addSubscriberToSubMsg([&](int handler,
                                       p_pi_msg_envelope_t p_env,
                                       const char* body, unsigned int len) {
    if (p_env->type == PIMSG_ROBOT_POS_2D)
    {
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
      
      int pose_tx_cnt_rcv, node_id_in_local;
      int64 time_stamp_tx;
      
      memcpy(&pose_tx_cnt_rcv, body, sizeof(int));
      memcpy(&node_id_in_local, body + sizeof(int), sizeof(int));
      memcpy(&time_stamp_tx, body + sizeof(int) * 2, sizeof(int64));
        
      memcpy(&rx, body + sizeof(int) * 2 + sizeof(int64), sizeof(double));
      memcpy(&ry, body + sizeof(int) * 2 + sizeof(int64) + sizeof(double), sizeof(double));
      memcpy(&rz, body + sizeof(int) * 2 + sizeof(int64) + sizeof(double) * 2, sizeof(double));
      memcpy(&qw, body + sizeof(int) * 2 + sizeof(int64) + sizeof(double) * 3, sizeof(double));
      memcpy(&qx, body + sizeof(int) * 2 + sizeof(int64) + sizeof(double) * 4, sizeof(double));
      memcpy(&qy, body + sizeof(int) * 2 + sizeof(int64) + sizeof(double) * 5, sizeof(double));
      memcpy(&qz, body + sizeof(int) * 2 + sizeof(int64) + sizeof(double) * 6, sizeof(double));

      double siny = +2.0 * (qw * qz+ qx * qy);
      double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);
      //double yaw = atan2(siny, cosy)*(180.0/M_PI);
      double yaw = atan2(siny, cosy);

      robotX = rx; //meter
      robotY = ry; //meter
      robotTh = yaw; //radian (Rad)
      //robotTh = yaw / 10; //huwei remark: the th of lidar is not accurate???
#endif

      /*std::chrono::high_resolution_clock::time_point pose_time_last3;
      pose_time_last3 = pose_time3;
      pose_time3 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed3 = pose_time3 - pose_time_last3;
      std::cout << "Receive duration: " << elapsed3.count() << " seconds." << std::endl;
      std::cout << "Receive Frequency: " << 1/elapsed3.count() << " Hz." << std::endl;
      */
      if (isSensorDataNew)
      {
        std::lock_guard<std::mutex> lock(mutexCartoPose);
        isSensorDataNew = false;
        std::cout << "robotX " << robotX << " robotY " << robotY << " robotTh " << robotTh << std::endl;

        LidarPose = PIRoBot::RobotPose(robotX, robotY, robotTh);
	
	pose_tx_cnt_rcv_ = pose_tx_cnt_rcv;
	node_id_in_local_ = node_id_in_local;
	time_stamp_tx_ = time_stamp_tx;
	
      }
      /*
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
      */
      return 1;
    }
  });
}


void PiCartoPoseRev::StartPoseThread() {
  LOG(INFO)<<"start pose thread";
  pPoseThread = new std::thread(&PiCartoPoseRev::Run, this);
}

void PiCartoPoseRev::EndPoseThread() {
  LOG(INFO)<<"end pose thread";
  pPoseThread->join();
  delete pPoseThread;
  pPoseThread = nullptr;
}



void PiCartoPoseRev::Run()  {

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

  LOG(INFO)<<"exit pose receiving thread.";
}















