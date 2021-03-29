//============================================================================
// Copyright   : PerceptIn
//============================================================================
#ifndef  __mapping_H__
#define  __mapping_H__

#include "occupancy_grid_map.h"
#include<thread>
#include<mutex>

namespace PIRoBot {

class Mapping {
  public:
    Mapping();
    ~Mapping();
    void Init(double _ogm_res, double _ogm_sq_size);
    virtual void RunOnce()= 0;
    void StartMappingThread();
    void EndMappingThread();
    void Run();
    std::mutex mutexExitThread;
    bool isExitThread;
    OccupancyGridMap * pOgm;
    OccupancyGridMap * pOgm2;
  private:
    std::thread * pMappingThread;

};

}
#endif   /* ----- #ifndef __mapping_H__  ----- */

