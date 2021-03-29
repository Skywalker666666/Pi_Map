//============================================================================
// Copyright   : PerceptIn
//============================================================================
#ifndef  __circle_buffer_H__
#define  __circle_buffer_H__

class CircleBuffer {
  public:
    CircleBuffer(int _bufSize);
    ~CircleBuffer();
    bool Push(unsigned char _b);
    bool Push(const unsigned char * _buf, int _len);
    bool Pop(unsigned char * _b);
    bool Pop(unsigned char * _buf, int _len);
    int DataLength();
  private:
    unsigned char * buf;
    int bufSize;
    // head point to the start of current data, tail point to the end data's next.
    int head;
    int tail;
};
#endif   /* ----- #ifndef __circle_buffer_H__  ----- */

