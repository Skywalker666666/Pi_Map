//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include "our_bot_comm.h"
#include <glog/logging.h>
#include <iomanip>        // setfill && setw
#include "CRC16.h"

extern uint16_t CalcCRC16(const unsigned char *input_str, uint64_t num_bytes);

inline unsigned char CalcLenCheckSum(int32_t len) {
  unsigned char * p = reinterpret_cast<unsigned char*>(&len);
  unsigned char res = p[0] + p[1] + p[2] + p[3];
  return res;
}
// FrameSync1, FrameSync2, Len, LenCheckSum, ID, Content, CheckSum
// Data means ID+Content, Len and CheckSum just think Data, that is Len is ID+Content,
// CheckSum is ID+Content
// Len and CheckSum are little endian
void PackageData(OurFuncCode fc, unsigned char * data, int32_t len, unsigned char * sendBuf, int32_t * sendLen) {
  if(data == NULL || len <= 0) {
    return ;
  }
  int32_t fcDataLen = len + 1; // plus FuncCode
  OurFrameHead * head = (OurFrameHead*)sendBuf;
  head->sync1 = OurFrameSync1;
  head->sync2 = OurFrameSync2;
  head->len = fcDataLen;
  head->lenCheckSum = CalcLenCheckSum(fcDataLen);

  size_t fcPos = sizeof(OurFrameHead);
  sendBuf[fcPos] = fc;
  memcpy(&sendBuf[fcPos + 1], data, len);
  uint16_t checkSum = CalcCRC16(sendBuf + fcPos, fcDataLen);
  unsigned char curPos = sizeof(OurFrameHead) + fcDataLen;
  sendBuf[curPos] = (unsigned char)(checkSum & 0xff);
  sendBuf[curPos+1] = (unsigned char)(checkSum >> 8);
  *sendLen = curPos + 2;
}
bool AnalysisPackage(CircleBuffer * circleBuf, unsigned char * dataBuf, int32_t * dataLen) {
  // static is essential, bcz a complete package may be divide some segments when calling AnalysisPackage.
	static unsigned char USART_LastByte = 0;
	static bool USART_BeginFlag = false;
	static unsigned int USART_RevOffset = 0;
  static unsigned int dataOffset = 0;
  static int32_t USART_DataLen = -1;
  static int32_t len = 0;
  static uint16_t checkSum = 0;
	
  while(circleBuf->DataLength() > 0) {
    unsigned char data;
    circleBuf->Pop(&data);
    if (((data == OurFrameSync2) && (USART_LastByte == OurFrameSync1)) || (USART_RevOffset > OurMaxPackageSize)) {
      //RESET
      USART_RevOffset = 0;
      USART_BeginFlag = true;
      USART_LastByte = data;
      USART_DataLen = -1;
      len = 0;
      dataOffset = 0;
      continue;
    }
    USART_LastByte = data;
    if (USART_BeginFlag) {
      // len is 4 bytes.
      if(USART_RevOffset < 4) {
        int bit_move = USART_RevOffset * 8;
        CHECK_LE(bit_move, 24);
        len += (int)(data<<bit_move); // don't use USART_LastByte when calcute USART_DataLen because USART_LastByte already be data.
      }
      else if(USART_RevOffset == 4) { // verify Len checksum
        unsigned char check = CalcLenCheckSum(len);
        if(check != data) {
          USART_BeginFlag = false;
          LOG(ERROR)<<"verify Len checksum failed: "<<len<<" "<<std::hex<<"0x"<<std::setfill('0')<<std::setw(2)<<(int)check<<" "<<std::hex<<"0x"<<std::setfill('0')<<std::setw(2)<<(int)data;
          continue;
        }
        USART_DataLen = len;
        dataOffset = 0;
      }
      else {
        // dataBuf is ID + Content, Not including Len or Checksum
        if(dataOffset < len) {
          dataBuf[dataOffset]= data;
        }
        else if(dataOffset == len) {
          checkSum = data;
        }
        else if(dataOffset == len+1) {
          checkSum += (uint16_t)(data<<8);
          uint16_t check = CalcCRC16(dataBuf, len);
          if(check != checkSum) {
            USART_BeginFlag = false;
            LOG(ERROR)<<"verify data checksum failed";
            continue;
          }
          *dataLen = len;
          USART_BeginFlag = false;
          return true;
        }
        dataOffset ++;
      }

      USART_RevOffset ++;
    }
  }
	
	return false;
}

