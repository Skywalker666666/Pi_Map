//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include "mapping.h"
#include <glog/logging.h>

namespace PIRoBot {

void Mapping::Init(double _ogm_res, double _ogm_sq_size) {
  VLOG(1)<<"mapping Init: "<<_ogm_res<<", "<<_ogm_sq_size;
  pOgm = new OccupancyGridMap(_ogm_res, _ogm_sq_size);
  pOgm2 = new OccupancyGridMap(_ogm_res, _ogm_sq_size);
}

Mapping::~Mapping() {
  if(pOgm) {
    delete pOgm;
    pOgm = nullptr;
  }

  if(pOgm2) {
    delete pOgm2;
    pOgm2 = nullptr;
  }
}

Mapping::Mapping(): pMappingThread(nullptr), isExitThread(false), pOgm(nullptr), pOgm2(nullptr) {};

void Mapping::StartMappingThread() {
  LOG(INFO)<<"start mapping thread";
  pMappingThread = new std::thread(&Mapping::Run, this);
}

void Mapping::EndMappingThread() {
  LOG(INFO)<<"end mapping thread";
  pMappingThread->join();
  delete pMappingThread;
  pMappingThread = nullptr;
}

void Mapping::Run() {
  while(1) {
    mutexExitThread.lock();
    if(isExitThread) {
      mutexExitThread.unlock();
      break;
    }
    mutexExitThread.unlock();

    this->RunOnce();
    usleep(1000);
  }
  LOG(INFO)<<"exit mapping thread";
}

}

