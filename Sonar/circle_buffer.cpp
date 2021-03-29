//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include "circle_buffer.h"
#include <glog/logging.h>

CircleBuffer::CircleBuffer(int _bufSize) {
  buf = new unsigned char [_bufSize];
  bufSize = _bufSize;
  head = 0;
  tail = 0;
}
CircleBuffer::~CircleBuffer() {
  if(buf) {
    delete buf;
    buf = nullptr;
  }
}
int CircleBuffer::DataLength() {
  if(tail >= head) {
    return tail - head;
  }
  else {
    return tail + bufSize - head;
  }
}
bool CircleBuffer::Push(const unsigned char * _buf, int _len) {
  int leftSize = bufSize - 1 - DataLength();
  if(_len > leftSize) {
    LOG(ERROR)<<"push data too much: "<<_len<<" "<<leftSize;
    return false;
  };
  for(int i=0; i<_len; i++) {
    buf[tail] = _buf[i];
    tail ++;
    if(tail >= bufSize) {
      tail -= bufSize;
    }
  }
  return true;
}
bool CircleBuffer::Push(unsigned char _b) {
  return Push(&_b, 1);
}
bool CircleBuffer::Pop(unsigned char * _buf, int _len) {
  int dataLen = DataLength();
  if(_len > dataLen) {
    LOG(ERROR)<<"pop data too much: "<<_len<<" "<<dataLen;
    return false;
  }
  for(int i=0; i<_len; i++) {
    _buf[i] = buf[head];
    head ++;
    if(head >= bufSize) {
      head -= bufSize;
    }
  }
  return true;
}
bool CircleBuffer::Pop(unsigned char * _b) {
  return Pop(_b, 1);
}

