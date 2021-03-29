//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include<CRC16.h>
#define		CRC_START_16	0x0000
#define		CRC_POLY_16		0xA001
uint16_t CalcCRC16(const unsigned char *input_str, uint64_t num_bytes) {
    static uint16_t crc_tab16[256];
    static bool crc_tab16_init = false;
    uint16_t i;
    uint16_t j;
    uint16_t crc_init;
    uint16_t c;
    uint16_t crc;
    uint16_t tmp;
    uint16_t short_c;
    const unsigned char *ptr;
    uint64_t a;

  if(!crc_tab16_init) {
    /*
     * For optimal performance uses the CRC16 routine a lookup table with values
     * that can be used directly in the XOR arithmetic in the algorithm. This
     * lookup table is calculated the first time the CRC function is called.
     */
    
    for(i=0; i<256; i++) {
      crc_init = 0;
      c = i;

      for(j=0; j<8; j++) {
        if((crc_init ^ c) & 0x0001) {
          crc_init = ( crc_init >> 1 ) ^ CRC_POLY_16;
        } else {
          crc_init = crc_init >> 1;
        }
        c = c >> 1;
      }

      crc_tab16[i] = crc_init;
    }

    crc_tab16_init = true;
  }

 
  crc = CRC_START_16;
  ptr = input_str;

  if(ptr != NULL) {
    for (a=0; a<num_bytes; a++) {
      short_c = 0x00ff & (uint16_t) *ptr;
      tmp = crc ^ short_c;
      crc = (crc >> 8) ^ crc_tab16[tmp&0xff];
      ptr++;
    }
  }

  return crc;
}  /* CalcCRC16 */
