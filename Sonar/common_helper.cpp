//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include "common_helper.h"
#include <sys/time.h>
#include <sstream>
#include <iomanip>
void get13bitTimestamp(std::string * timestamp) {
  struct timeval t_val;
  gettimeofday(&t_val, NULL);
  std::stringstream ss;
  ss<<std::setw(10)<<t_val.tv_sec<<std::setw(3)<<std::setfill('0')<<static_cast<unsigned int>(t_val.tv_usec/1000.0);
  *timestamp = ss.str();
}

void get13bitTimestamp(uint64_t * timestamp) {
  struct timeval t_val;
  gettimeofday(&t_val, NULL);
  *timestamp = t_val.tv_sec * 1000llu + t_val.tv_usec / 1000llu;
}
