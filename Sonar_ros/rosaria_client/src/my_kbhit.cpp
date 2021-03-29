//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include "my_kbhit.h"

#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

static struct termios ori_attr, cur_attr;

static __inline int tty_reset(void)
{
  if (tcsetattr(STDIN_FILENO, TCSANOW, &ori_attr) != 0)
    return -1;

  return 0;
}


static __inline int tty_set(void)
{

  if ( tcgetattr(STDIN_FILENO, &ori_attr) )
    return -1;

  memcpy(&cur_attr, &ori_attr, sizeof(cur_attr) );
  cur_attr.c_lflag &= ~ICANON;
  // cur_attr.c_lflag |= ECHO;
  cur_attr.c_lflag &= ~ECHO;
  cur_attr.c_cc[VMIN] = 1;
  cur_attr.c_cc[VTIME] = 0;

  if (tcsetattr(STDIN_FILENO, TCSANOW, &cur_attr) != 0)
    return -1;

  return 0;
}

int kbhit(void)
{

  fd_set rfds;
  struct timeval tv;
  int retval;

  FD_ZERO(&rfds);
  FD_SET(0, &rfds);

  tv.tv_sec= 0;
  tv.tv_usec = 0;

  retval = select(1, &rfds, NULL, NULL, &tv);

  if (retval == -1) {
    perror("select()");
    return 0;
  } 
  else if (retval)
    return 1;
  else
    return 0;
  return 0;
}

static int tty_set_flag;
void init_kbhit() {
  tty_set_flag = tty_set();
}
void finish_kbhit() {
  if(tty_set_flag == 0)
    tty_reset();
}

