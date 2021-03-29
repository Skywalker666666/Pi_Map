//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include <glog/logging.h>
#include "debug.h"
#include "our_bot.h"
#include "common_helper.h" // for get13bitTimestamp
#include <cstring>
#include <sstream>
#include <iostream>
#include <iomanip>        // setfill && setw
#include <algorithm>        // min && max
#include <sys/time.h>        // gettimeofday
#include <cmath> // for M_PI

namespace PIRoBot{

#define MONITOR_INTERMEDIATE

OurBot::OurBot() : circleBuf(1024) {
  bNewBotDataSensor = false;
  movDir = MoveStop;
  isRotating = false;

  InitSonarPos();
}
bool OurBot::InitBot(std::string serialName) {
  bool ret = serial.Open(serialName.c_str());
  if(ret) {
    LOG(INFO)<<"open serial OK";
  }
  return ret;
}
void OurBot::Finish() {
  serial.Close();
}

void OurBot::InitSonarPos() {
  for(int i=0; i<OurSonarNum; i++) {
    sonarPos[i].x = sonarX[i];
    sonarPos[i].y = sonarY[i];
    sonarPos[i].th = sonarTh[i];
  }
}

void OurBot::WriteCmd(OurFuncCode fc, unsigned char * cmdBuf, int32_t cmdLen) {
  unsigned char sendBuf[OurMaxPackageSize];
  int32_t sendLen;
  PackageData(fc, cmdBuf, cmdLen, sendBuf, &sendLen);
  serial.Write(sendBuf, sendLen);
} 
// here we directly write cmd in this function instead of in RunOnce, so no mutex is needed. which is better?
void OurBot::SetLinearAndAugularVelocity(double _linearV, double _angularV) {
  if(_linearV > 0) {
    movDir = PIRoBot::MoveForeward;
  } else if (_linearV == 0) {
    movDir = PIRoBot::MoveStop;
  } else if(_linearV < 0) {
    movDir = PIRoBot::MoveBackward;
  }

  if(_angularV != 0) {
    isRotating = true;
  } else {
    isRotating = false;
  }

  OurBotSetVel vel;
  vel.linearV = _linearV;
  vel.angularV = _angularV;
  WriteCmd(FC_OurBotSetVel, reinterpret_cast<unsigned char*>(&vel), sizeof(OurBotSetVel));
}

void OurBot::SetDeltaDistAndAngle(float _delDist, float _delAngle) {
    OurBotDeltaPos dPos;
    dPos.dDist = _delDist;
    dPos.dAngle = _delAngle;
    WriteCmd(FC_OurBotSetDeltaPos, reinterpret_cast<unsigned char*>(&dPos), sizeof(OurBotDeltaPos));
}

void OurBot::StopBot() {
  WriteCmd(FC_OurBotStop, NULL, 0); // TODO: modify PackageData !!
}

long getTimeMs() {
    long time = 0;
    struct timeval now;
    gettimeofday(&now, NULL);
    time = now.tv_sec*1000 + now.tv_usec/1000.0;
    return time;
}

void restrict_pi_rad(float & angle) {
    if(angle > M_PI) {
        angle -= 2*M_PI;
    } else if(angle < -M_PI) {
        angle += 2*M_PI;
    }
}

RangeType OurBot::getRangeType(float distance) {
    RangeType rangeType;
    if(distance > rangeFarThresh) {
        rangeType = RangeFar;
    } else if(distance > rangeMidThresh) {
        rangeType = RangeMid;
    } else {
        rangeType = RangeNear;
    }
    return rangeType;
}

void OurBot::ProcessSonarData(const Sonar *curSonar) {
    ControlType control = StopMoveCtrl;

    static int ctrl_index = 0;

    float leftFrontMin = 1000, rightFrontMin = 1000;
    float leftSideMin = 1000, rightSideMin = 1000;
    if(MoveForeward == movDir) {
        leftFrontMin = std::min(curSonar->sonar[2], curSonar->sonar[3]);
        rightFrontMin = std::min(curSonar->sonar[4], curSonar->sonar[5]);

        leftSideMin = std::min(curSonar->sonar[0], curSonar->sonar[1]);
        rightSideMin = std::min(curSonar->sonar[6], curSonar->sonar[7]);
    }
    else if(MoveBackward == movDir) {
        leftFrontMin = std::min(curSonar->sonar[10], curSonar->sonar[11]);
        rightFrontMin = std::min(curSonar->sonar[12], curSonar->sonar[13]);

        leftSideMin = std::min(curSonar->sonar[8], curSonar->sonar[9]);
        rightSideMin = std::min(curSonar->sonar[14], curSonar->sonar[15]);
    }
    RangeType leftRange = getRangeType(leftFrontMin);
    RangeType rightRange = getRangeType(rightFrontMin);
    switch(leftRange) {
        case RangeNear: {
            switch(rightRange) {
                case RangeNear: {
                    if(leftFrontMin < rangeStopThresh && rightFrontMin < rangeStopThresh) {
                        control = StopMoveCtrl;
                    }
                    else if(leftFrontMin < rightFrontMin) {
                        control = TurnRightMuchCtrl;
                    } else {
                        control = TurnLeftMuchCtrl;
                    }
                    break;
                }
                case RangeMid: {
                    control = TurnRightLittleCtrl;
                    break;
                }
                case RangeFar: {
                    control = TurnRightMuchCtrl;
                    break;
                }
            }
            break;
        }
        case RangeMid: {
            switch(rightRange) {
                case RangeNear: {
                    control = TurnLeftLittleCtrl;
                    break;
                }
                case RangeMid: {
                    VLOG(1)<<ctrl_index<<" - prefix result: left-Medium-right-Medium!";
                    DebugStdOut(std::cout<<ctrl_index<<" - prefix result: left-Medium-right-Medium!"<<std::endl);
                    control = GoAheadCtrl;
                    break;
                }
                case RangeFar: {
                    control = TurnRightLittleCtrl;
                    break;
                }
            }
            break;
        }
        case RangeFar: {
            switch(rightRange) {
                case RangeNear: {
                    control = TurnLeftMuchCtrl;
                    break;
                }
                case RangeMid: {
                    control = TurnLeftLittleCtrl;
                    break;
                }
                case RangeFar: {
                    VLOG(1)<<ctrl_index<<" - prefix result: left-far-right-far!";
                    DebugStdOut(std::cout<<ctrl_index<<" - prefix result: left-far-right-far!"<<std::endl);
                    control = GoAheadCtrl;
                    break;
                }
            }
            break;
        }
    } // end 'switch(leftRange)'

    if(control == GoAheadCtrl || control == TurnLeftLittleCtrl || control == TurnRightLittleCtrl) {
        if(leftSideMin < rangeStopThresh) {
            if(rightSideMin < rangeStopThresh) {
                VLOG(1)<<ctrl_index<<" - prefix result: leftside-stopThresh-rightside-stopThresh!";
                DebugStdOut(std::cout<<ctrl_index<<" - prefix result: leftside-stopThresh-rightside-stopThresh!"<<std::endl);
                control = StopMoveCtrl;
            } else {
                control = TurnRightSideLittleCtrl;
            }
        } else if(rightSideMin < rangeStopThresh) {
            control = TurnLeftSideLittleCtrl;
        }
    }

    static bool adjust = false;

    if(StopMoveCtrl == control) {
        VLOG(1)<<ctrl_index<<" - result: "<<controls[StopMoveCtrl];
        DebugStdOut(std::cout<<ctrl_index<<" - result: "<<controls[StopMoveCtrl]<<std::endl);
        adjust = false;
        SetLinearAndAugularVelocity(0.0, 0.0);
        StopBot();
        return;
    } else if(GoAheadCtrl == control) {
        VLOG(1)<<ctrl_index<<" - result: "<<controls[GoAheadCtrl];
        DebugStdOut(std::cout<<ctrl_index<<" - result: "<<controls[GoAheadCtrl]<<std::endl);
        if(adjust == true) {
            adjust = false;
            SetLinearAndAugularVelocity(0.10, 0);
        }
        return;
    }

    /**
     * 90 -> 1.5708
     * 45 -> 0.7854
     * 40 -> 0.6981
     * 10 -> 0.1745
     * 5 -> 0.0873
     * 4 -> 0.0698
     * 2 -> 0.0349
     * 1 -> 0.0175
     * 0.5 -> 0.0087
     * */
    const float turnMuchAngle = 1.5708;
    const float turnMediumAngle = 0.6981;
    const float turnSideLittleAngle = 0.1745;
    const float turnLittleAngle = 0.0873;
    const float rotateFiniAngle = 0.0175;

    float desiredTh = curTh;
    static float lastDesiredTh = 0.0;

    static bool haveAvoidLeft = false;
    static bool haveAvoidRight = false;

    float diffAngle = lastDesiredTh - curTh;
    restrict_pi_rad(diffAngle);

#define ADJUST_TIMEOUT  (3 * 1000)


    if(control != TurnRightMuchCtrl && haveAvoidLeft && (std::abs(diffAngle) < 0.1745)) {
        control = TurnLeftMediumCtrl;
        haveAvoidLeft = false;

        adjust = false;  // we need reset #adjust, since the control has changed from 'Much' to 'Medium'.
    }
    if(control != TurnLeftMuchCtrl && haveAvoidRight && (std::abs(diffAngle) < 0.1745)) {
        control = TurnRightMediumCtrl;
        haveAvoidRight = false;

        adjust = false;  // we need reset #adjust, since the control has changed from 'Much' to 'Medium'.
    }

    static long adjustBegin = 0L;

    if(adjust == true) {
        if(std::abs(diffAngle) < rotateFiniAngle) {
            VLOG(1)<<ctrl_index<<" - result: Adjust FINISH!!!";
            DebugStdOut(std::cout<<ctrl_index<<" - result: Adjust FINISH!!!"<<std::endl);
            adjust = false;
            SetLinearAndAugularVelocity(0.10, 0);
        } else {
            #ifdef MONITOR_INTERMEDIATE
              VLOG(1)<<"Adjusting...!!!";
              DebugStdOut(std::cout<<"Adjusting...!!!"<<" curTh: "<<curTh<<", lastDesiredTh: "<<lastDesiredTh<<std::endl);
            #endif
            if(adjustBegin > 0 && ((getTimeMs() - adjustBegin) >= ADJUST_TIMEOUT)) {
                VLOG(1)<<"Adjust TIMEOUT, continuing moving...";
                DebugStdOut(std::cout<<"Adjust TIMEOUT, continuing moving..."<<std::endl);
                adjust = false;
                adjustBegin = 0L;
                SetLinearAndAugularVelocity(0.10, 0);
            }
        }
        return;
    }

    switch(control) {
        case TurnLeftLittleCtrl: {
            SetDeltaDistAndAngle(0, turnLittleAngle);
            desiredTh = curTh + turnLittleAngle;
            break;
        }
        case TurnLeftSideLittleCtrl: {
            SetDeltaDistAndAngle(0, turnSideLittleAngle);
            desiredTh = curTh + turnSideLittleAngle;
            break;
        }
        case TurnLeftMediumCtrl: {
            SetDeltaDistAndAngle(0, turnMediumAngle);
            desiredTh = curTh + turnMediumAngle;
            break;
        }
        case TurnLeftMuchCtrl: {
            SetDeltaDistAndAngle(0, turnMuchAngle);
            desiredTh = curTh + turnMuchAngle;
            haveAvoidRight = true;
            VLOG(1)<<"Have avoid RIGHT!!!";
            DebugStdOut(std::cout<<"Have avoid RIGHT!!!"<<std::endl);
            break;
        }
        case TurnRightMuchCtrl: {
            SetDeltaDistAndAngle(0, -turnMuchAngle);
            desiredTh = curTh - turnMuchAngle;
            haveAvoidLeft = true;
            VLOG(1)<<"Have avoid LEFT!!!";
            DebugStdOut(std::cout<<"Have avoid LEFT!!!"<<std::endl);
            break;
        }
        case TurnRightMediumCtrl: {
            SetDeltaDistAndAngle(0, -turnMediumAngle);
            desiredTh = curTh - turnMediumAngle;
            break;
        }
        case TurnRightSideLittleCtrl: {
            SetDeltaDistAndAngle(0, -turnSideLittleAngle);
            desiredTh = curTh - turnSideLittleAngle;
            break;
        }
        case TurnRightLittleCtrl: {
            SetDeltaDistAndAngle(0, -turnLittleAngle);
            desiredTh = curTh - turnLittleAngle;
            break;
        }
        default: {
          LOG(ERROR)<<"invalid control: "<<control;
          break;
        }
    } // end 'switch(control)'

    restrict_pi_rad(desiredTh);
    lastDesiredTh = desiredTh;

    isRotating = true;
    adjust = true;
    adjustBegin = getTimeMs();

    VLOG(1)<<ctrl_index++<<"curTh: "<<curTh<<", control: "<<controls[control]<<", desiredTh: "<<desiredTh;
    DebugStdOut(std::cout<<ctrl_index++<<"curTh: "<<curTh<<", control: "<<controls[control]<<", desiredTh: "<<desiredTh<<std::endl);
}

void OurBot::RunOnce() {
  unsigned char buf[1024];
  int readSize;
  serial.Read(buf, &readSize);
  circleBuf.Push(buf, readSize);
// OutputSerialRead this macro is just used as debug, to output the raw data that serial has read.  
#ifdef OutputSerialRead
  std::stringstream ss;
  for(int i=0; i<readSize; i++) {
      ss<<std::hex<<"0x"<<std::setfill('0')<<std::setw(2)<<(int)buf[i]<<" ";
  }
  if(readSize > 0) {
    VLOG(1)<<"read: "<<ss.str();
  }
#endif  
  // Note: dataBuf must static, bcz may call AnalysisPackage multiple times to get a complete package.
  static unsigned char dataBuf[OurMaxPackageSize];
  int32_t dataLen;
  if(AnalysisPackage(&circleBuf, dataBuf, &dataLen)) {
    ProcessPackageData(dataBuf, dataLen);
  }
  if(bNewBotDataSensorInternal) {
      std::lock_guard<std::mutex> lock(mutexNewBotDataSensor);
      bNewBotDataSensorInternal = false;
      botDataSensor = botDataSensorInternal;
      curTh = botDataSensor.posTh;
      recvTimestamp = recvTimestampInternal;
      bNewBotDataSensor = true;
  }

  if(bNewSonarInternal) {
      std::lock_guard<std::mutex> lock(mutexNewSonarData);
      bNewSonarInternal = false;
      sonarData = sonarInternal;
      #ifdef MONITOR_INTERMEDIATE
        std::stringstream sonarStr;
        for(int z = 0; z < OurSonarNum; z++) {
          sonarStr<<sonarData.sonar[z]<<" ";
        }
        VLOG(1)<<sonarStr.str();
        DebugStdOut(std::cout<<sonarStr.str()<<std::endl);
      #endif
      if(movDir == PIRoBot::MoveForeward || movDir == PIRoBot::MoveBackward || isRotating == true) {
        //ProcessSonarData(&sonarData);
      }
      recvTimestamp = recvTimestampInternal;
      bNewSonar = true;
  }
}
void OurBot::ProcessPackageData(const unsigned char * dataBuf, int32_t dataLen) {
  unsigned char fc = dataBuf[0];
  if(FC_OurBotDataSensor == fc) {
    if(dataLen != sizeof(OurBotDataSensor) + 1) {
      LOG(ERROR)<<"wrong bot data sensor len: "<<dataLen<<" "<<sizeof(OurBotDataSensor) + 1;
      return;
    }
    {
      botDataSensorInternal = * (OurBotDataSensor*)(&dataBuf[1]);
      get13bitTimestamp(&recvTimestampInternal);
      bNewBotDataSensorInternal = true;
    }
  } else if (FC_OurBotSonar == fc) {
    if(dataLen != sizeof(Sonar) + 1) {
      LOG(ERROR)<<"wrong bot data Sonar len: "<<dataLen<<" "<<(sizeof(Sonar) + 1);
      return;
    }
      sonarInternal = *(Sonar *)(&dataBuf[1]);

      get13bitTimestamp(&recvTimestampInternal);
      bNewSonarInternal = true;
  }
}
void OurBot::ConvertSonarToSensorScan(const Sonar & _sonar, SensorScan & _scanVec) {
  _scanVec.clear();
  for(int i=0; i<OurSonarNum; i++) {
    ScanData _scan;
    _scan.sx = sonarPos[i].x;
    _scan.sy = sonarPos[i].y;
    _scan.th = sonarPos[i].th;
    _scan.len = _sonar.sonar[i];
    _scanVec.push_back(_scan);
  }
  CHECK_EQ(_scanVec.size(), OurSonarNum);
}

}
