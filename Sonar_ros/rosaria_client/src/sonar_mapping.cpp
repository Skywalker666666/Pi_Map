//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include "sonar_mapping.h"
#include <glog/logging.h>


#include <iostream>
#include <fstream>
#include <iomanip>
#include <sys/time.h>
  
#include <ctime>
#include <chrono>

#include <algorithm>
#include <string>
std::ofstream lidarPosFile;
//std::string our_lidar_pose_path("/home/ubuntu/our_lidar_data.txt");
//std::string our_lidar_pose_path("/home/ubuntu/Documents/Tools/scripts/inputs/0519_from_May/our_lidar_data.txt");
char* our_lidar_pose_path = (char *)"/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/our_lidar_data.txt";

namespace PIRoBot {
  
SonarMapping::SonarMapping(std::string sonar_map_algo_option) 
          : sonar_map_algo_option_(sonar_map_algo_option) {} 
 
void SonarMapping::UpdateSensor(const SensorScan & _scan, const RobotPose & _pose) {
  std::lock_guard<std::mutex> lock(mutexSonarScan);
  sonar_scan = _scan;
  robot_pose = _pose;
  bNewSensorData = true;
}

void SonarMapping::RunOnce() {
  SensorScan _scan;
  RobotPose _pose;
  bool _isNewSensorData;
  {
    std::lock_guard<std::mutex> lock(mutexSonarScan);
    _isNewSensorData = bNewSensorData;
    if(bNewSensorData) {
      bNewSensorData = false;
      _scan = sonar_scan;
      _pose = robot_pose;
    }
  }
  if(!_isNewSensorData) {
    return;
  }

/*
#ifdef DIRECT_CENTRAL_LINE_ALGORITHM
  UpdateMap(_scan, _pose);
#endif

#ifdef DIRECT_CONE_ALGORITHM
  UpdateMap2(_scan, _pose);
#endif

#ifdef BAYESIAN_ALGORITHM
  UpdateMap3(_scan, _pose);
#endif

#ifdef DS_ALGORITHM
  UpdateMap4(_scan, _pose);
#endif
*/
  // changed to command arguments mode.
  if (sonar_map_algo_option_ == "CLD" ) {
    // "DIRECT_CENTRAL_LINE_ALGORITHM"
    UpdateMap(_scan, _pose);
  }
  else if (sonar_map_algo_option_ == "CoD" ) {
    // "DIRECT_CONE_ALGORITHM"
    UpdateMap2(_scan, _pose);    
  }
  else if (sonar_map_algo_option_ == "Bay" ) {
    // "BAYESIAN_ALGORITHM"
    UpdateMap3(_scan, _pose);
  }
  else if (sonar_map_algo_option_ == "DS" ) {
    // "DS_ALGORITHM"
    UpdateMap4(_scan, _pose);  
  }
  else{
    std::cout << "sonar mapping algorithm is wrong." << std::endl;
  }
}

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


//sonar direct method line model
void SonarMapping::UpdateMap(const SensorScan & _scan, const RobotPose & _pose) {

  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  // millisecond

  lidarPosFile.open(our_lidar_pose_path, std::fstream::out | std::fstream::app);
  lidarPosFile << ms << " " << std::fixed << std::setprecision(10) << _pose.x << " " << _pose.y << " " << _pose.th << std::endl;
  lidarPosFile.close();

  //const double max_sonar_dist_threshold = 4.9;
  //const double max_sonar_dist_threshold = 4.5;
  //const double max_sonar_dist_threshold = 3.0;
  //const double max_sonar_dist_threshold = 2.0;
  //const double max_sonar_dist_threshold = 1.5;
  //const double max_sonar_dist_threshold = 1.4;
  const double max_sonar_dist_threshold = 1.3;
  const double min_sonar_dist_threshold = 0.2;
  VLOG(1)<<"sonar update map";
  for(int i=0; i<_scan.size(); i++) {
    ScanData _sonar = _scan[i];
    //if(_sonar.len > max_sonar_dist_threshold) {
    // only open two four directions
    //if(_sonar.len > max_sonar_dist_threshold || !(i == 0 || i == 7 || i == 8 || i == 15) ) {
    //if(_sonar.len > max_sonar_dist_threshold || !(i == 0 || i == 7 ) ) {
    if( (_sonar.len < min_sonar_dist_threshold ) || (_sonar.len > max_sonar_dist_threshold ) || !(i == 0 || i == 7) ) {         
      continue;
    }
    double startPtMapX, startPtMapY;
    double endPtMapX, endPtMapY;
    double curHeading;
    startPtMapX = (_sonar.sx * cos(_pose.th) - _sonar.sy * sin(_pose.th)) + _pose.x;
    startPtMapY = (_sonar.sx * sin(_pose.th) + _sonar.sy * cos(_pose.th)) + _pose.y;
    curHeading = _sonar.th + _pose.th;
    const double expandLen = 0.05 * 5;
    //const double expandLen = 0.05 * 0;
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
      } if(std::abs(ogmVal - 0.5) <  1e-6) {
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


//sonar direct method cone model
void SonarMapping::UpdateMap2(const SensorScan & _scan, const RobotPose & _pose) {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  // millisecond

  lidarPosFile.open(our_lidar_pose_path, std::fstream::out | std::fstream::app);
  lidarPosFile << ms << " " << std::fixed << std::setprecision(10) << _pose.x << " " << _pose.y << " " << _pose.th << std::endl;
  lidarPosFile.close();

  //const double max_sonar_dist_threshold = 2.0;
  //const double max_sonar_dist_threshold = 1.4;
  const double max_sonar_dist_threshold = 1.3;
  const double min_sonar_dist_threshold = 0.2;
  //angle increase/decrease step value, 0.6 degree to rad
  const double theta_step = 0.6 * M_PI / 180;
  // record all updated cells in one sonar of one scan.
  std::vector<PntIdx>  cone;
  // Iterator used to store the position of searched element 
  std::vector<PntIdx>::iterator kt;  

  VLOG(1)<<"sonar update map 2";
  for(int i=0; i<_scan.size(); i++) {
    cone.clear();
    for(int j = -15; j < 16; j++) {
      ScanData _sonar = _scan[i];
      if( (_sonar.len < min_sonar_dist_threshold ) || (_sonar.len > max_sonar_dist_threshold ) || !(i == 0 || i == 7) ) {         
      //if(_sonar.len > max_sonar_dist_threshold) {
        continue;
      }
      double startPtMapX, startPtMapY;
      double endPtMapX, endPtMapY;
      double curHeading;
      startPtMapX = (_sonar.sx * cos(_pose.th) - _sonar.sy * sin(_pose.th)) + _pose.x;
      startPtMapY = (_sonar.sx * sin(_pose.th) + _sonar.sy * cos(_pose.th)) + _pose.y;
      curHeading = (_sonar.th + j * theta_step) + _pose.th;

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


        PntIdx cell_candidate = *jt;
        // search for skip duplication
        // skip if cell_candidate has been updated in this round for this sonar
        kt = std::find(cone.begin(), cone.end(), cell_candidate); 
        
        if(kt != cone.end()) {
          VLOG(1) << "Cell" << jt->x_idx << ", " << jt->y_idx << "found being updated alreaday";
          continue;
        }
        else {
          cone.push_back(cell_candidate);
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
  }
  VLOG(1)<<"sonar update map 2 finish";
}//UpdateMap2



//sonar bayesian filtering method cone model
void SonarMapping::UpdateMap3(const SensorScan & _scan, const RobotPose & _pose) {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  // millisecond

  lidarPosFile.open(our_lidar_pose_path, std::fstream::out | std::fstream::app);
  lidarPosFile << ms << " " << std::fixed << std::setprecision(10) << _pose.x << " " << _pose.y << " " << _pose.th << std::endl;
  lidarPosFile.close();

  //const double max_sonar_dist_threshold = 2.0;
  //const double max_sonar_dist_threshold = 1.0;
  const double max_sonar_dist_threshold = 1.3;
  const double min_sonar_dist_threshold = 0.2;
  const double theta_step               = 0.6 * M_PI / 180;
  //angle increase/decrease step value, 0.6 degree to rad
  const double d3                       = 0; //sonar model extend, 0.4m when resolution is 0.05m// default is 2
  // record all updated cells in one sonar of one scan.
  std::vector<PntIdx>  cone;
  // Iterator used to store the position of searched element 
  std::vector<PntIdx>::iterator kt;  

 
  VLOG(1)<<"sonar update map 3";
  for(int i=0; i<_scan.size(); i++) {
    cone.clear();
    ScanData _sonar = _scan[i];
    //if(_sonar.len > max_sonar_dist_threshold) {
    //if(_sonar.len > max_sonar_dist_threshold || !(i == 0 || i == 3 || i == 4 || i == 7 || i == 8
    //   || i == 11 || i == 12 || i == 15) ) {
    //if(_sonar.len > max_sonar_dist_threshold || !(i == 0 || i == 3 || i == 4 || i == 7) ) {    
    //if(_sonar.len > max_sonar_dist_threshold || !(i == 0 || i == 7 || i == 8 || i == 15) ) {  
    if( (_sonar.len < min_sonar_dist_threshold ) || (_sonar.len > max_sonar_dist_threshold ) || !(i == 0 || i == 7) ) {         
      //TO DO, minimum
      continue;
    }
    for(int j = -15; j < 16; j++) {
      double startPtMapX, startPtMapY;
      double endPtMapX, endPtMapY;
      double curHeading;
      startPtMapX = (_sonar.sx * cos(_pose.th) - _sonar.sy * sin(_pose.th)) + _pose.x;
      startPtMapY = (_sonar.sx * sin(_pose.th) + _sonar.sy * cos(_pose.th)) + _pose.y;
      curHeading = (_sonar.th + j * theta_step) + _pose.th;

      const double expandLen = 0.05 * d3;
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

      //const PntIdx & range;
      for(auto jt=linePtIdx.begin(); jt < linePtIdx.end(); jt++) {
        double ogmVal = pOgm->GetVal(*jt);
        if(ogmVal < 0) {
          LOG(ERROR)<<"point on line out bound of ogm"<<jt->x_idx<<" "<<jt->y_idx;
          continue;
        }

        //double curPtMapX, curPtMapY;
        //pOgm->CalcCoord(*jt, &curPtMapX, &curPtMapY); // may return False??


        PntIdx cell_candidate = *jt;
        // search for skip duplication
        // skip if cell_candidate has been updated in this round for this sonar
        kt = std::find(cone.begin(), cone.end(), cell_candidate); 
        
        if(kt != cone.end()) {
          VLOG(1) << "Cell" << jt->x_idx << ", " << jt->y_idx << "found being updated alreaday";
          continue;
        }
        else {
          cone.push_back(cell_candidate);
        }

        //double DistFromstart = pow(curPtMapX - startPtMapX, 2) + pow(curPtMapY - startPtMapY, 2);

        double GridFromend = linePtIdx.end() - jt;

        //double expectVal = sonarInvModel2(GridFromend); 
        double expectVal = sonarInvModel3(GridFromend); 
        
        double newVal; 
        updateGrid(ogmVal, expectVal, newVal);
        // weighted 
        double newVal_weighted;
        //newVal_weighted = newVal * (1 - pow((j * theta_step) / (15 * theta_step), 2));
        newVal_weighted = newVal;
        pOgm->SetVal(*jt, newVal_weighted);
      }// for linePtIdx
    }// for j
  }// for i
  VLOG(1)<<"sonar update map 3 finish";
}//UpdateMap3


double SonarMapping::sonarInvModel(const double & GridFromend) {
  const double p_occ     = 0.60;
  const double p_free    = 0.40;
  const double p_prior   = 0.50;
  const double d3        =    4; //8, sonar model extend, 0.4m when resolution is 0.05m
  const double d2        =    3; //6
  const double d1        =    2; //4
  if ( GridFromend >=  d3 + d1 ){
    // jt <= end - d3 - d1
    return p_free;
  }
  else if( (GridFromend < d3 + d1) & (GridFromend >= d3 - d1) ){
    // end - d3 - d1 < jt <= end - d3 + d1
    return (((p_occ - p_free) / (2 * d1)) * ((d3 + d1) - GridFromend) + p_free);
  }
  else if( (GridFromend < d3 - d1) & (GridFromend >= d3 - d2) ){
    // end - d3 + d1 < jt <= end - d3 + d2
    return p_occ;
  }
  else if( (GridFromend < d3 - d2) & (GridFromend >= 0) ){
    // end - d3 + d2 <= jt < end
    return ((p_occ - p_prior) / (d3 - d2)) * GridFromend + p_prior;
  }
  else{
    return p_prior;
  }
}//sonarInvModel


// shifted inside a little bit, use d1 only @ one side
double SonarMapping::sonarInvModel2(const double & GridFromend) {
  const double p_occ     = 0.60;
  const double p_free    = 0.40;
  const double p_prior   = 0.50;
  const double d3        =    2; //8, sonar model extend, 0.4m when resolution is 0.05m
  const double d2        =    1; //6
  const double d1        =    2; //4
  if ( GridFromend >=  d3 + d1 ){
    // jt <= end - d3 - d1
    return p_free;
  }
  else if( (GridFromend < d3 + d1) & (GridFromend >= d3) ){
    // end - d3 - d1 < jt <= end - d3
    return (((p_occ - p_free) / d1) * ((d3 + d1) - GridFromend) + p_free);
  }
  else if( (GridFromend < d3 ) & (GridFromend >= d3 - d2) ){
    // end - d3 < jt <= end - d3 + d2
    return p_occ;
  }
  else if( (GridFromend < d3 - d2) & (GridFromend >= 0) ){
    // end - d3 + d2 <= jt < end
    return ((p_occ - p_prior) / (d3 - d2)) * GridFromend + p_prior;
  }
  else{
    return p_prior;
  }
}//sonarInvModel

// only a single line
double SonarMapping::sonarInvModel3(const double & GridFromend) {
  const double p_occ     = 0.60;
  const double p_free    = 0.40;
  const double p_prior   = 0.50;
  const double d3        =    2; //8, sonar model extend, 0.4m when resolution is 0.05m
  const double d2        =    1; //6
  const double d1        =    2; //4
  if ( GridFromend >= 2 ){
    return p_free;
  }
  else if( (GridFromend == 1) ){
    return p_occ;
  }
  else{
    return p_prior;
  }
}//sonarInvModel



//bool SonarMapping::getGridBel(const double & ogmVal , double & bel) {
//}//getGridBel

void SonarMapping::getGridLogBel(const double & ogmVal, double & log_bel) {
  double bel;
  bel = ogmVal;
  log_bel = log( bel / (1.0 - bel ));
}//getGridLogBel

//bool SonarMapping::setGridBel(double & new_bel, const double & log_bel) {
//}//setGridBel

void SonarMapping::setGridLogBel(double & new_bel, const double & log_bel) {
  new_bel = 1.0 - 1.0/ ((1.0 + exp(log_bel)));
}// setGridLogBel

void SonarMapping::updateGrid(const double & ogmVal, const double & Psim, double & new_bel) {
  double log_bel;
  getGridLogBel(ogmVal, log_bel);
  log_bel = log_bel + log( Psim / ( 1.0 - Psim));
  setGridLogBel(new_bel, log_bel);
}// updateGrid




//sonar Dempster-Shafer method cone model
void SonarMapping::UpdateMap4(const SensorScan & _scan, const RobotPose & _pose) {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  // millisecond

  //lidarPosFile.open("/home/ubuntu/Documents/Tools/scripts/inputs/0519_from_May/our_lidar_data.txt", std::fstream::out | std::fstream::app);
  lidarPosFile.open(our_lidar_pose_path, std::fstream::out | std::fstream::app);
  lidarPosFile << ms << " " << std::fixed << std::setprecision(10) << _pose.x << " " << _pose.y << " " << _pose.th << std::endl;
  lidarPosFile.close();

  //const double max_sonar_dist_threshold = 2.0;
  //const double max_sonar_dist_threshold = 1.4;
  const double max_sonar_dist_threshold = 1.3;
  const double min_sonar_dist_threshold = 0.2;
  const double theta_step               = 0.6 * M_PI / 180;
  //angle increase/decrease step value, 0.6 degree to rad
  const double d3                       = 0; //sonar model extend, 0.4m when resolution is 0.05m
  
  //// record all updated cells in one sonar of one scan.
  //std::vector<PntIdx>  cone;

  // Iterator used to store the position of searched element 
  std::vector<PntIdx>::iterator kt;  
  // Iterator used to store the position of searched element 
  std::vector<PntIdx>::iterator nt;  

  // Iterator used to traverse arc and sector 
  std::vector<PntIdx>::iterator qt;  
  std::vector<PntIdx>::iterator tt;  

  // arc, record all arc cells in one sonar of one scan.
  std::vector<PntIdx>  arc;
  // sector, record all cells inside ars in one sonar of one scan.
  std::vector<PntIdx>  sector;
 
  VLOG(1)<<"sonar update map 4";
  for(int i=0; i<_scan.size(); i++) {
    arc.clear();
    sector.clear();
    ScanData _sonar = _scan[i];
    //if(_sonar.len > max_sonar_dist_threshold) {
    if( (_sonar.len < min_sonar_dist_threshold ) || (_sonar.len > max_sonar_dist_threshold ) || !(i == 0 || i == 7) ) {         
      //TO DO, minimum
      continue;
    }

    for(int j = -15; j < 16; j++) {
      double startPtMapX, startPtMapY;
      double endPtMapX, endPtMapY;
      double curHeading;
      startPtMapX = (_sonar.sx * cos(_pose.th) - _sonar.sy * sin(_pose.th)) + _pose.x;
      startPtMapY = (_sonar.sx * sin(_pose.th) + _sonar.sy * cos(_pose.th)) + _pose.y;
      curHeading  = (_sonar.th + j * theta_step) + _pose.th;

      const double expandLen = 0.05 * d3;
      endPtMapX = startPtMapX + (_sonar.len + expandLen) * cos(curHeading);
      endPtMapY = startPtMapY + (_sonar.len + expandLen) * sin(curHeading);
      PntIdx startPtOgmIdx, endPtOgmIdx;
      pOgm->CalcIdx(startPtMapX, startPtMapY, &startPtOgmIdx);
      pOgm->CalcIdx(endPtMapX, endPtMapY, &endPtOgmIdx);
      std::vector<PntIdx> linePtIdx;
      CalcLinePassTwoPt(startPtOgmIdx, endPtOgmIdx, linePtIdx);
      // search for skip duplication
      kt = std::find(arc.begin(), arc.end(), endPtOgmIdx); 
      if(kt != arc.end()) {
        VLOG(1) << "Arc Cell" << kt->x_idx << ", " << kt->y_idx << "found being updated alreaday";
        continue;
      }
      else {
        arc.push_back(endPtOgmIdx);
      }

      //const PntIdx & range;
      for(auto jt=linePtIdx.begin(); jt < linePtIdx.end() - 1; jt++) {
        // skip the last element in the vector
        PntIdx cell_candidate = *jt;
        // search for skip duplication
        // skip if cell_candidate has been updated in this round for this sonar
        nt = std::find(sector.begin(), sector.end(), cell_candidate); 
        if(nt != sector.end()) {
          VLOG(1) << "Sector Cell" << nt->x_idx << ", " << nt->y_idx << "found being updated alreaday";
          continue;
        }
        else {
          sector.push_back(cell_candidate);
        }
      }// for jt
    }// for j

    double arc_grid_n    = arc.size();
    double sector_grid_n = sector.size();
    if(arc_grid_n < 0) {
      LOG(ERROR)<<"point on line out bound of ogm"<< arc_grid_n << "is less than 0.";
      continue;
    }

    for(auto qt=arc.begin(); qt < arc.end(); qt++) {
      //occupied (or full), F
      double ogmVal = pOgm->GetVal(*qt);
      if(ogmVal < 0) {
        LOG(ERROR)<<"point on line out bound of ogm"<<qt->x_idx<<" "<<qt->y_idx;
        continue;
      }
      //free (or empty), E
      double ogm2Val = pOgm2->GetVal(*qt);
      if(ogm2Val < 0) {
        LOG(ERROR)<<"point on line out bound of ogm2"<<qt->x_idx<<" "<<qt->y_idx;
        continue;
      }
      // F, E
      FEProbVal PriorOgmVal(ogmVal, ogm2Val);
      FEProbVal SensorVal(1/arc_grid_n, 0.0);
      FEProbVal PostOgmVal;
      DempsterShaferCombine(PriorOgmVal, SensorVal, PostOgmVal);
      pOgm ->SetVal(*qt, PostOgmVal.F);
      pOgm2->SetVal(*qt, PostOgmVal.E);
    }// for qt

    for(auto tt=sector.begin(); tt < sector.end(); tt++) {
      //occupied (or full), F
      double ogmVal = pOgm->GetVal(*tt);
      if(ogmVal < 0) {
        LOG(ERROR)<<"point on line out bound of ogm"<<tt->x_idx<<" "<<tt->y_idx;
        continue;
      }
      //free (or empty), E
      double ogm2Val = pOgm2->GetVal(*tt);
      if(ogm2Val < 0) {
        LOG(ERROR)<<"point on line out bound of ogm2"<<tt->x_idx<<" "<<tt->y_idx;
        continue;
      }
      // F, E
      double rho = 1/arc_grid_n;
      FEProbVal PriorOgmVal(ogmVal, ogm2Val);
      FEProbVal SensorVal(0.0, rho);
      FEProbVal PostOgmVal;
      
      DempsterShaferCombine(PriorOgmVal, SensorVal, PostOgmVal);
      pOgm ->SetVal(*tt, PostOgmVal.F);
      pOgm2->SetVal(*tt, PostOgmVal.E);
    }// for tt

  }// for i
  VLOG(1)<<"sonar update map 4 finish";
}//UpdateMap4

// Dempster-Shafer core
void SonarMapping::DempsterShaferCombine(const FEProbVal & PriorOgmVal, const FEProbVal & SensorVal, FEProbVal & PostOgmVal) {
  double K = 1 - PriorOgmVal.F * SensorVal.E - PriorOgmVal.E * SensorVal.F;
  PostOgmVal.F = (PriorOgmVal.F * SensorVal.F + PriorOgmVal.F * (1 - SensorVal.F - SensorVal.E) + (1 - PriorOgmVal.F - PriorOgmVal.E)) * SensorVal.F / K;
  PostOgmVal.E = (PriorOgmVal.E * SensorVal.E + PriorOgmVal.E * (1 - SensorVal.E - SensorVal.F) + (1 - PriorOgmVal.E - PriorOgmVal.F)) * SensorVal.E / K;
}

}//namespace PIRoBot
