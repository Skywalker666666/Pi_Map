//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <iostream>
#include <fstream>
#include<thread>
#include <sys/time.h> // for gettimeofday

#include "my_kbhit.h"
#include "our_bot.h"
#include "sonar_mapping.h"

#include "aux_func.h"

DEFINE_string(serialName, "/dev/ttyUSB1", "serial port name");
DEFINE_bool(display, false, "whether the host pc connects a display");

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>    // circle

using namespace std;
using namespace cv;

static bool notFirst = false;
ofstream ogm_txt_file;

ofstream botFile;
ofstream botSonarFile;

// return true to exit main loop.
bool ProcessKey(PIRoBot::Bot * pBot, int key) {
  static double curLinearV = 0.0;
  static double curAngularV = 0.0;
  double stepLinearV = 0.05;
  double stepAngularV = 0.1; // 2017.03.28 Note: increase from 0.05, because our bot's low speed control is not well yet.
  double maxLinearV = 0.25;
  double maxAngularV = 1.5;

  double mySnakeRotRad[] = {0, 1.57, 3.14, 1.57};
  static int mySnakeRotNum = 0;
  static double mySnakeTranLongLen = 1.0;
  static double mySnakeTranShortLen = 0.1;
  static double squareDist = 1.0;

  PIRoBot::OurBot * pOurbot = nullptr;

  if(key == 'z') {
    cout<<"stop"<<endl;
    curLinearV = 0;
    curAngularV = 0;
    pBot->SetLinearAndAugularVelocity(curLinearV, curAngularV);
  }
  else if(key == 'w') {
    curLinearV += stepLinearV;
    curLinearV = (curLinearV < maxLinearV) ? curLinearV : maxLinearV;
    curAngularV = 0;
    pBot->SetLinearAndAugularVelocity(curLinearV, curAngularV);
  }
  else if(key == 's') {
    curLinearV -= stepLinearV;
    curLinearV = (curLinearV > -maxLinearV) ? curLinearV : -maxLinearV;
    curAngularV = 0;
    pBot->SetLinearAndAugularVelocity(curLinearV, curAngularV);
  }
  else if(key == 'a') {
    cout<<"rotate left"<<endl;
    curAngularV += stepAngularV;
    curAngularV = (curAngularV < maxAngularV) ? curAngularV : maxAngularV;
    curLinearV = 0;
    pBot->SetLinearAndAugularVelocity(curLinearV, curAngularV);
  }
  else if(key == 'd') {
    cout<<"rotate right"<<endl;
    curAngularV -= stepAngularV;
    curAngularV = (curAngularV > -maxAngularV) ? curAngularV : -maxAngularV;
    curLinearV = 0;
    pBot->SetLinearAndAugularVelocity(curLinearV, curAngularV);
  }
  else if(key == 'c') {
    cout<<"stop bot"<<endl;
    pOurbot = dynamic_cast<PIRoBot::OurBot *>(pBot);
    CHECK(pOurbot);
    pOurbot->StopBot();
  }
  else if(key == 0x1b || key == 'q') {
    pBot->mutexExitThread.lock();
    pBot->isExitThread = true;
    pBot->mutexExitThread.unlock();

    return true;
  }
  return false;
}

int main(int argc, char *argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  //PIRoBot::HBot * pHbot = new PIRoBot::HBot();
  PIRoBot::Bot * pBot = new PIRoBot::OurBot();
  PIRoBot::OurBot * pOurbot = dynamic_cast<PIRoBot::OurBot*>(pBot);
  bool ret = pBot->InitBot(FLAGS_serialName);
  if(!ret)
    return -1;
  pBot->StartBotThread();
  //pBot->SetLinearAndAugularVelocity Unit is m/s and rad/s

  PIRoBot::SonarMapping mapping;
  mapping.Init(0.05, 80);
  mapping.AddCallback();
  mapping.StartMappingThread();

  vector<OurBotDataSensor> botVec;
  vector<Sonar> sonarVec;

  vector<size_t> recvTimeVec;
  vector<size_t> sonarRecvTimeVec;
  size_t recvTime;
  size_t lastTimestamp = 0;
  init_kbhit();
  while(1) {
    usleep(1000);
    if(kbhit()) {
      const int key = getchar();
      bool ret = ProcessKey(pBot, key);
      if(ret) {
        break;
      }
    }

    std::string timestamp;
    double posX;
    double posY;
    double posTh;
    double leftEncoderPos;
    double rightEncoderPos;
    double gyrZ;
    OurBotDataSensor botDataSensor;

    // bot data sensor (wheel odometry)
    bool bNewBotDataSensor = false;
    pOurbot->mutexNewBotDataSensor.lock();
    if(pOurbot->bNewBotDataSensor) {
      pOurbot->bNewBotDataSensor = false;
      bNewBotDataSensor = true;
      botDataSensor = pOurbot->botDataSensor;
      recvTime = pOurbot->recvTimestamp;
    }
    pOurbot->mutexNewBotDataSensor.unlock();

    if(bNewBotDataSensor) {
      botVec.push_back(botDataSensor);
      recvTimeVec.push_back(recvTime);
      if(lastTimestamp != 0) {
        double deltaTime = (double)botDataSensor.timestamp - (double)lastTimestamp;
        if(deltaTime > 30 || deltaTime < 10) {
          cout<<"wrong time: "<<deltaTime<<" "<<botDataSensor.timestamp<<" "<<lastTimestamp<<endl;
        }
      }
      lastTimestamp = botDataSensor.timestamp;
      //cout<<"bot data sensor: "<<botDataSensor.timestamp<<" "<<botDataSensor.posX<<" "<<botDataSensor.posY<<" "<<botDataSensor.posTh<<" "<<botDataSensor.leftWheelVel<<" "<<botDataSensor.leftEncoder<<" "<<endl;
      bNewBotDataSensor = false;
    }

    // sonar data
    size_t sonarLastTimestamp = 0;
    Sonar sonar;
    bool bNewSonar = false;
    pOurbot->mutexNewSonarData.lock();
    if(pOurbot->bNewSonar) {
      pOurbot->bNewSonar = false;
      bNewSonar = true;
      sonar = pOurbot->sonarData;
      recvTime = pOurbot->recvTimestamp;
    }
    pOurbot->mutexNewSonarData.unlock();

    if(bNewSonar) {
      PIRoBot::SensorScan _scan;
      pOurbot->ConvertSonarToSensorScan(sonar, _scan);
      PIRoBot::RobotPose _pose;
      // for simplicify, directly use botVec's newest data. should make sure _scan and _pose are time synchronize
      OurBotDataSensor _bot_pose;
      _bot_pose = botVec.back();
      _pose.x = _bot_pose.posX;
      _pose.y = _bot_pose.posY;
      _pose.th = _bot_pose.posTh;
      mapping.UpdateSensor(_scan, _pose);

      sonarVec.push_back(sonar);
      sonarRecvTimeVec.push_back(recvTime);
      if(sonarLastTimestamp != 0) {
        double sonarDeltaTime = (double)sonar.timestamp - (double)sonarLastTimestamp;
        if(sonarDeltaTime > 30 || sonarDeltaTime < 10) {
          cout<<"sonar wrong time: "<<sonarDeltaTime<<" "<<sonar.timestamp<<" "<<sonarLastTimestamp<<endl;
        }
      }
      sonarLastTimestamp = sonar.timestamp;
      bNewSonar = false;

      if (FLAGS_display) {
        const cv::Point2d imgOriginOffsetInMap = cv::Point2d(-40.0, 40.0);
        const double img2mapScale = 0.05;
        #define MapCoord2ImgCoord(x, y) mapCoord2ImgCoord(x, y, img2mapScale, imgOriginOffsetInMap)
        //Visualize
        cv::Mat ogmMat = mapping.pOgm->Visualize();
        cv::namedWindow("ogm", cv::WINDOW_NORMAL);
        cv::Mat bgrOgmMat;
        cvtColor(ogmMat, bgrOgmMat, CV_GRAY2BGR);
        cv::Point2i ptImg;
        cv::Point3d pt(_pose.x,_pose.y, 0.0);
        MapCoord2ImgCoord(pt, ptImg);
        cv::circle(bgrOgmMat, ptImg, 3, Scalar(255,0,0), -1);
        cv::Point2i ptImg2;
        ptImg2.x = ptImg.x + 10 * cos(_pose.th);
        ptImg2.y = ptImg.y - 10 * sin(_pose.th);
        cv::arrowedLine(bgrOgmMat, ptImg, ptImg2, Scalar(0,255,0));
        cv::imshow("ogm", bgrOgmMat);
        cv::waitKey(2);
      }
    }

  } // end of "while (1)"
  finish_kbhit();
  CHECK_EQ(botVec.size(), recvTimeVec.size());

  ogm_txt_file.open("sonar_2d_ogm.txt", ios::out);
  for (int x = 0; x < (mapping.pOgm->sq_size() * 20); x++) {
    for (int y = 0; y < (mapping.pOgm->sq_size() * 20); y++) {
        double cellValue = mapping.pOgm->GetVal(x, y);
        ogm_txt_file << x << "," << y << "," << cellValue << std::endl;
    }
  }
  ogm_txt_file.close();

  botFile.open("our_bot_data.txt", ios::out);
  for(int i=0; i<botVec.size(); i++) {
    const OurBotDataSensor & _d = botVec[i];
    botFile<<_d.timestamp<<" "<<_d.posX<<" "<<_d.posY<<" "<<_d.posTh<<" "<<_d.leftEncoder<<" "<<_d.rightEncoder<<" "<<_d.leftWheelVel<<" "<<_d.rightWheelVel<<" "<<_d.gyrYaw<<" "<<recvTimeVec[i]<<endl;
  }
  botFile.close();

  botSonarFile.open("sonar_data.txt", ios::out);
  for(int i=0; i<sonarVec.size(); i++) {
    const Sonar & _sonar = sonarVec[i];
    botSonarFile<<_sonar.timestamp;
    for(int k = 0; k < OurSonarNum; k++ ) {
        botSonarFile<<" "<<_sonar.sonar[k];
    }
    botSonarFile<<" "<<sonarRecvTimeVec[i]<<std::endl;
  }
  botSonarFile.close();
  pBot->EndBotThread();
  pBot->Finish();
}

