//============================================================================
// Copyright   : PerceptIn
//============================================================================
#ifndef _CRC16_h_
#define _CRC16_h_
#include<cstdint>
#include<cstdlib>
uint16_t CalcCRC16(const unsigned char *input_str, uint64_t num_bytes);
#endif
