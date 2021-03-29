//============================================================================
// Copyright   : PerceptIn
//============================================================================
#ifndef  __our_bot_H__
#define  __our_bot_H__

#include <string>
#include "bot.h"
#include "our_bot_comm.h"
#include "serial.h"
#include "circle_buffer.h"
#include "sensor_data.h"

namespace PIRoBot {
enum ControlType {
    GoAheadCtrl = 0,
    TurnLeftLittleCtrl, TurnLeftSideLittleCtrl, TurnLeftMediumCtrl, TurnLeftMuchCtrl,
    TurnRightLittleCtrl, TurnRightSideLittleCtrl, TurnRightMediumCtrl, TurnRightMuchCtrl,
    StopMoveCtrl, CTRL_TYPE_NUM,
};

static const std::string controls[CTRL_TYPE_NUM] = {"GoAhead",
    "LeftLittle", "LeftSideLittle", "LeftMedium", "LeftMuch",
    "RightLittle", "RightSideLittle", "RightMedium", "RightMuch", "Stop"};

enum RangeType {
    RangeFar = 0,
    RangeMid = 1,
    RangeNear = 2
};

class OurBot : public Bot {
  public:
    OurBot();
    bool InitBot(std::string serialName);
    void SetLinearAndAugularVelocity(double _linearV, double _angularV);
    void SetDeltaDistAndAngle(float _delDist, float _delAngle);
    void StopBot();
    void Finish();
    void RunOnce();
    void ConvertSonarToSensorScan(const Sonar & _sonar, SensorScan & _scanVec);

    double gyrZ;
    double headingFromImu;
    std::string timestampStr;

    std::mutex mutexNewBotDataSensor;
    OurBotDataSensor botDataSensor;
    bool bNewBotDataSensor;

    std::mutex mutexNewSonarData;
    Sonar sonarData;
    struct PosHead sonarPos[OurSonarNum];
    bool bNewSonar;
    size_t recvTimestamp;
  private:
    const float rangeFarThresh = 0.70;
    const float rangeMidThresh = 0.40;
    const float rangeStopThresh = 0.20;

    const double sonarX[OurSonarNum] = {0.078, 0.108, 0.14, 0.156, 0.156, 0.143, 0.112, 0.075, -0.185, -0.221, -0.252, -0.267, -0.268, -0.275, -0.22, -0.184};
    const double sonarY[OurSonarNum] = {0.115, 0.155, 0.045, 0.02, -0.02, -0.0505, -0.103, -0.111, -0.106, -0.102, -0.051, -0.017, 0.016, 0.0735, 0.111, 0.115};
    const double sonarTh[OurSonarNum] = {1.5708, 1.0297, 0.6842, 0.2269, -0.2409, -0.733, -1.0647, -1.5708, -1.5708, -2.0944, -2.5133, -2.9496, 2.8972, 2.4435, 1.9809, 1.5708};


    void InitSonarPos();
    RangeType getRangeType(float distance);
    void WriteCmd(OurFuncCode fc, unsigned char * cmdBuf, int32_t cmdLen);
    void ProcessPackageData(const unsigned char * dataBuf, int32_t dataLen);
    void ProcessSonarData(const Sonar *curSonar);
    float curTh;
    Serial serial;
    Sonar sonarInternal;
    OurBotDataSensor botDataSensorInternal;
    bool bNewBotDataSensorInternal;
    bool bNewSonarInternal;
    size_t recvTimestampInternal;
    CircleBuffer circleBuf;
};
}

#endif   /* ----- #ifndef __our_bot_H__  ----- */
