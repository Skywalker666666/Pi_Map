//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include "mapping.h"
#include <glog/logging.h>

namespace PIRoBot {

void Mapping::Init(double _ogm_res, double _ogm_sq_size) {
  VLOG(1)<<"mapping Init: "<<_ogm_res<<", "<<_ogm_sq_size;
  pOgm = new OccupancyGridMap(_ogm_res, _ogm_sq_size);
}

Mapping::~Mapping() {
  if(pOgm) {
    delete pOgm;
    pOgm = nullptr;
  }
}

Mapping::Mapping():pOgm(nullptr) {};

}

