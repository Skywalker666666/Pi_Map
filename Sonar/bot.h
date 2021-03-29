//============================================================================
// Copyright   : PerceptIn
//============================================================================
#ifndef  __bot_H__
#define  __bot_H__

#include<thread>
#include<mutex>

namespace PIRoBot {

#define IRMaxNum 16
#define DropSensorMaxNum 16
#define CollisionSensorMaxNum 16

  enum MoveDirection {
    MoveBackward = 0,
    MoveStop = 1,
    MoveForeward = 2
  };

  struct Pos {
    double x;
    double y;
  };
  struct PosHead {
    double x;
    double y;
    double th;
  };
  struct IRSensor {
    PosHead posHead;
    double data;
  };
  struct DropSensor {
    Pos pos;
    double data; // int?
  };
  struct CollisionSensor {
    Pos pos;
    double data; // int?
  };

  class Bot {
    public:
      Bot();
      virtual bool InitBot(std::string serialName) = 0;
      virtual void Finish() = 0;
      virtual void RunOnce()= 0;
      void StartBotThread();
      void EndBotThread();
      void Run();
      virtual void SetLinearAndAugularVelocity(double _linearV, double _angularV) = 0;
      IRSensor ir[IRMaxNum];
      DropSensor dropSensor[DropSensorMaxNum];
      CollisionSensor collisionSensor[CollisionSensorMaxNum];

      MoveDirection movDir;
      bool isRotating;

      int sonarNum;
      int irNum;
      int dropSensorNum;
      int collisionSensorNum;
      //
      double linearVel;
      double angularVel;
      double leftEncoderPos;
      double rightEncoderPos;
      double heading; // TODO remove?
      PosHead pos;

      // command
      double linearVelCmd;
      double angularVelCmd;
      double leftWheelVelCmd;
      double rightWheelVelCmd;

      std::mutex mutexExitThread;
      bool isExitThread;
      std::mutex mutexStateUpdate;
      bool isNewState;
      std::mutex mutexNewVel;
      bool isNewVel;
    //protect:
      //double linearVelFromImpl;
    private:
      std::thread * pBotThread;
  };

}
#endif   /* ----- #ifndef __bot_H__  ----- */


