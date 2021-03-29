//============================================================================
// Copyright   : PerceptIn
//============================================================================
#ifndef  __sonar_mapping_H__
#define  __sonar_mapping_H__
#include "mapping.h"
#include "sensor_data.h"
#include "pose_2d.h"
#include <msg_utils/pi_msg_adaptor.h>

namespace PIRoBot {
class SonarMapping : public Mapping {
  public:
    SonarMapping();
    ~SonarMapping(){};
    void UpdateSensor(const SensorScan & _scan, const RobotPose & _pose);
    //void RunOnce();
    void Run();

    void StartMappingThread();
    void EndMappingThread();

    void AddCallback();

    std::mutex mutexSonarScan;
    bool bNewSensorData;
    SensorScan sonar_scan;
    RobotPose robot_pose;

    std::mutex mutexExitThread;
    bool isExitThread;
  private:
    std::shared_ptr<PIAUTO::msg::PIMsgAdaptor> msgPubAdp;
    std::shared_ptr<PIAUTO::msg::PIMsgAdaptor> msgSubAdp;
    int posPubHandler;
    int posSubHandler;
    void CalcLinePassTwoPt(const PntIdx & start, const PntIdx & end, std::vector<PntIdx> & line);
    void UpdateMap(const SensorScan & _scan, const RobotPose & _pose);
    std::thread * pMappingThread;
};

}
#endif   /* ----- #ifndef __sonar_mapping_H__  ----- */

