//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <iostream>
#include <iomanip>
        // setfill && setw
#include <string>
#include <sstream>
#include "serial.h"

void Serial::set_speed(int8_t fd, int speed)
{
#ifdef __APPLE__
  // OS X don't have definition of some high baud rate
    speed_t speed_arr[] = {B230400, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
    int name_arr[] = {230400, 115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300};
#else    
    speed_t speed_arr[] = {B921600, B460800, B230400, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
    int name_arr[] = {921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300};
#endif    

    struct termios Opt;
    tcgetattr(fd, &Opt);

    uint8_t i;
    for(i= 0;i<sizeof(name_arr)/sizeof(name_arr[0]);i++)
    {
        if(speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);

            int status = tcsetattr(fd, TCSANOW, &Opt);
            if (status != 0)
            {
                return;
            }
            tcflush(fd, TCIOFLUSH);
        }
    }
}

int Serial::set_Parity(int fd,int databits,int stopbits,int parity) {
    struct termios options;

    if(tcgetattr(fd,&options) != 0)
    {
        return 1;
    }

    options.c_cflag &= ~CSIZE;
    /* Set time out */
    options.c_cc[VTIME] = 1;   //wait for 5s at most
    options.c_cc[VMIN] = 0;

    switch (databits)
    {
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            return 1;
    }

    options.c_cflag &= ~(PARENB|PARODD);
    switch (parity)
    {
        case 'n':
        case 'N':
            //options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 'S':
        case 's':
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            return 1;
    }

    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            return 1;
    }

    options.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF | IXANY);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);/*raw input*/
    options.c_oflag &= ~OPOST;/*raw output*/

    /* Apply new settings */
    if(-1==tcsetattr(fd, TCSANOW, &options))
    {
      LOG(ERROR)<<"serial config failed";
        return -1;
    }

    tcflush(fd,TCIOFLUSH);
    return 0;
}

int Serial::flushInCache()
{
    return tcflush(fd, TCIFLUSH);
}

// 2017.03.28 this function coming from https://github.com/wjwwood/serial, may still have some problems but it works now.
static void ReconfigurePort (int fd_) {
  if (fd_ == -1) {
    LOG(ERROR)<<"invalid file descriptor";
    return;
  }

  struct termios options; // The options for the file descriptor

  if (tcgetattr(fd_, &options) == -1) {
    LOG(ERROR)<<"failed to tcgetattr";
    return;
  }

  // set up raw mode / no echo / binary
  options.c_cflag |= (tcflag_t)  (CLOCAL | CREAD);
  options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
      ISIG | IEXTEN); //|ECHOPRT

  options.c_oflag &= (tcflag_t) ~(OPOST);
  options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);
#ifdef IUCLC
  options.c_iflag &= (tcflag_t) ~IUCLC;
#endif
#ifdef PARMRK
  options.c_iflag &= (tcflag_t) ~PARMRK;
#endif

  // setup baud rate
  speed_t baud = B115200;
#ifdef _BSD_SOURCE
  ::cfsetspeed(&options, baud);
#else
  ::cfsetispeed(&options, baud);
  ::cfsetospeed(&options, baud);
#endif

  // setup char len
  options.c_cflag &= (tcflag_t) ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= (tcflag_t) ~(CSTOPB);
  // setup parity
  options.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
  options.c_cflag &= (tcflag_t) ~(PARENB | PARODD);
  bool xonxoff_ = false;
  bool rtscts_ = false;
#ifdef IXANY
  if (xonxoff_)
    options.c_iflag |=  (IXON | IXOFF); //|IXANY)
  else
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
#else
  if (xonxoff_)
    options.c_iflag |=  (IXON | IXOFF);
  else
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF);
#endif
  // rtscts
#ifdef CRTSCTS
  if (rtscts_)
    options.c_cflag |=  (CRTSCTS);
  else
    options.c_cflag &= (unsigned long) ~(CRTSCTS);
#elif defined CNEW_RTSCTS
  if (rtscts_)
    options.c_cflag |=  (CNEW_RTSCTS);
  else
    options.c_cflag &= (unsigned long) ~(CNEW_RTSCTS);
#else
#error "OS Support seems wrong."
#endif

  // http://www.unixwiz.net/techtips/termios-vmin-vtime.html
  // this basically sets the read call up to be a polling read,
  // but we are using select to ensure there is data available
  // to read before each call, so we should never needlessly poll
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  // activate settings
  ::tcsetattr (fd_, TCSANOW, &options);
}

bool Serial::Open(const char * devName) {
  fd = open(devName, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
  if (fd == -1)
  {
    std::cout<<"ERROR: Can't open serial. \n"<<std::endl;
    return false;
  }
  else
  {
    std::cout<<"serial opened correctly.\n"<<std::endl;
    LOG(INFO)<<"open serial OK";
  };
// 2017.03.28 there are some unknown problems to send data on my OS X, temporarily have to call another config here.
#ifdef __APPLE__
  ReconfigurePort(fd);
#else  
  set_speed(fd, 115200);
  if(set_Parity(fd,8,1,'N'))
  {
    std::cout<<"ERROR: set parity error. \n"<<std::endl;
    return false;
  }
#endif
  flushInCache();
  return true;
}
void Serial::Close() {
  close(fd);
}
void Serial::Write(const unsigned char * buf, int len) {
#if 0
    std::stringstream ss;
    ss<<"*******************send_BEGIN************"<<std::endl;
    ss<<"buf["<<len<<"]={";
    for(int i=0;i<len;i++) {
        ss<<std::hex<<"0x"<<std::setfill('0')<<std::setw(2)<<(int)buf[i]<<", ";
    }
    ss<<"}"<<std::endl;
    ss<<"*******************send_END*************"<<std::endl;
    LOG(INFO)<<ss.str();
#endif
  write(fd, buf, len);
}
void Serial::Read(unsigned char * buf, int * readSize) {
  struct timeval tv = { 0, 10 * 1000 };
  fd_set readfd;
  int rc;
  FD_ZERO(&readfd);
  FD_SET(fd, &readfd);
  rc = select(fd + 1, &readfd, NULL, NULL, &tv);
  *readSize = 0;
  if(rc > 0) {
    *readSize = read(fd, buf, 256);
  }
}

