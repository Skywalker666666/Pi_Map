//============================================================================
// Copyright   : PerceptIn
//============================================================================
#ifndef  __serial_H__
#define  __serial_H__
// not return bool, because navipack_comm.c is C not C++
typedef unsigned char(*ProcessRecvByteCBFunc)(unsigned char c);
typedef void(*ProcessRecvPackageCBFunc)();

class Serial {
  public:
    Serial(): processRecvByteFunc(nullptr), processRecvPackageFunc(nullptr) {};

    bool Open(const char * devName );
    void set_speed(int8_t fd, int speed);
    int set_Parity(int fd,int databits,int stopbits,int parity);
    int flushInCache();
    void Close();
    void RecvAndParse();
    void Read(unsigned char * buf, int * readSize);
    void Write(const unsigned char * buf, int len);
    ProcessRecvByteCBFunc processRecvByteFunc;
    ProcessRecvPackageCBFunc processRecvPackageFunc;

  private:
    int fd;
    int readSize;
};

#endif   /* ----- #ifndef __serial_H__  ----- */
