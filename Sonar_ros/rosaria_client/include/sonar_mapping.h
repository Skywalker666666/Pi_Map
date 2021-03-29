//============================================================================
// Copyright   : PerceptIn
//============================================================================
#ifndef  __sonar_mapping_H__
#define  __sonar_mapping_H__
#include "mapping.h"
#include "sensor_data.h"
#include "pose_2d.h"

#define OurSonarNum    16
#define M_PI           3.14159265358979323846	/* pi */

//#define DIRECT_CENTRAL_LINE_ALGORITHM
//#define DIRECT_CONE_ALGORITHM
//#define BAYESIAN_ALGORITHM
//#define DS_ALGORITHM

//CLD
//CoD
//Bay
//DS

namespace PIRoBot {
struct FEProbVal{
  // probability of full
  double F;
  // probability of empty
  double E;

  FEProbVal(): F(0), E(0) {}
  FEProbVal(const double F, const double E) : F(F), E(E) {}
};

class SonarMapping : public Mapping {
  public:
    SonarMapping(std::string sonar_map_algo_option); 
    //~SonarMapping();
    
    void UpdateSensor(const SensorScan & _scan, const RobotPose & _pose);
    void RunOnce();
    std::mutex mutexSonarScan;
    bool bNewSensorData;
    SensorScan sonar_scan;
    RobotPose robot_pose;
  private:
    void CalcLinePassTwoPt(const PntIdx & start, const PntIdx & end, std::vector<PntIdx> & line);
    void UpdateMap( const SensorScan & _scan, const RobotPose & _pose);
    void UpdateMap2(const SensorScan & _scan, const RobotPose & _pose);
    void UpdateMap3(const SensorScan & _scan, const RobotPose & _pose);
    void UpdateMap4(const SensorScan & _scan, const RobotPose & _pose);
    double sonarInvModel(const double & GridFromend);
    double sonarInvModel2(const double & GridFromend);//change distance d1 d2 d3
    double sonarInvModel3(const double & GridFromend);//change distance 0 and others
    void getGridLogBel(const double & ogmVal, double & log_bel);
    void setGridLogBel(double & new_bel, const double & log_bel);
    void updateGrid(const double & ogmVal, const double & Psim, double & new_bel);
    void DempsterShaferCombine(const FEProbVal & PriorOgmVal, const FEProbVal & SensorVal, FEProbVal & PostOgmVal);
    std::string sonar_map_algo_option_;
};

}
#endif   /* ----- #ifndef __sonar_mapping_H__  ----- */

