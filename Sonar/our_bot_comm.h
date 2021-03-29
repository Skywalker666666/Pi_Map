//============================================================================
// Copyright   : PerceptIn
//============================================================================
#ifndef  __our_bot_comm_H__
#define  __our_bot_comm_H__

#include "circle_buffer.h"
#include <cstdint>
#include <cstddef> // for size_t
#include "swap_endian.h"

#define OurMaxPackageSize 1024
#define OurFrameSync1 0xb5
#define OurFrameSync2 0x62

#define OurSonarNum    16

enum OurFuncCode {
  FC_OurBotData = 0x01,
  FC_OurBotPoseSet = 0x02,
  FC_OurBotDataSensor = 0x03,
  FC_OurBotSetVel = 0x04,
  FC_OurBotSetDeltaPos = 0x05,
  FC_OurBotStop = 0x06,
  FC_OurBotSonar = 0x07,
};

#pragma pack(push, 1)
typedef struct {
  unsigned char sync1;
  unsigned char sync2;
  int32_t len;
  unsigned char lenCheckSum;
} OurFrameHead;

// Note: use float in struct, the definition of float in send and receive must be consist.
// size_t must be 8 bytes, namely only support 64bit OS currently!
typedef struct {
  size_t timestamp;
  float posX;
  float posY;
  float posTh;
} OurBotData;

typedef struct {
  size_t timestamp;
  float posX;
  float posY;
  float posTh;
  float leftWheelVel;
  float rightWheelVel;
  float leftEncoder;
  float rightEncoder;
  float gyrYaw;
} OurBotDataSensor;

typedef struct {
  size_t timestamp;
  float sonar[OurSonarNum]; //detect distance
} Sonar;


typedef struct {
  float posX;
  float posY;
  float posTh;
} OurBotPoseSet;

typedef struct {
  float linearV;
  float angularV;
} OurBotSetVel;

typedef struct {
    float dDist;
    float dAngle;
} OurBotDeltaPos;

#pragma pack(pop)

void PackageData(OurFuncCode fc, unsigned char * data, int32_t len, unsigned char * sendBuf, int32_t * sendLen);
bool AnalysisPackage(CircleBuffer * circleBuf, unsigned char * dataBuf, int32_t * dataLen);
uint16_t CalcCRC16(const unsigned char *input_str, size_t num_bytes);

#endif   /* ----- #ifndef __our_bot_comm_H__  ----- */
