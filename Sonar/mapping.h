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
    virtual ~Mapping();
    void Init(double _ogm_res, double _ogm_sq_size);
    //virtual void RunOnce() = 0;
    virtual void StartMappingThread() = 0;
    virtual void EndMappingThread() = 0;
    virtual void Run() = 0;
    OccupancyGridMap * pOgm;
};

}
#endif   /* ----- #ifndef __mapping_H__  ----- */

