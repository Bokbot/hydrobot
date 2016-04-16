#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

// The arduino board i2c address
#define ADDRESS 0x6b

// The I2C bus: This is for V2 pi's. For V1 Model B you need i2c-0
static const char *devName = "/dev/i2c-1";

int retrieveValue(int call, int *pointfile) {
  // initialize command to get test
  // these other bytes are irrelevant at the moment
  unsigned int cmd[16];

  a = 105;
  b = 2350;
  c = 4587;
  d = 12587;
  e = 12;

  cmd[0] = call;
  cmd[1] = a >>  8;
  cmd[2] = a & 255;
  cmd[3] = b >>  8;
  cmd[4] = b & 255;
  cmd[5] = c >>  8;
  cmd[6] = c & 255;
  cmd[7] = d >>  8;
  cmd[8] = d & 255;
  cmd[9] = e >>  8;
  cmd[10]= e & 255;


  if (write(*pointfile, cmd, 11) == 11) {
    // As we are not talking to direct hardware but a microcontroller we
    // need to wait a short while so that it can respond.
    //
    // 1ms seems to be enough but it depends on what workload it has
    usleep(10000);

    char buf[3];
    if (read(*pointfile, buf, 2) == 2) {
  int resultant = buf[0] << 8 | buf[1];

  return resultant;
    }
    else{
      return -1;
    }
  }
}

int main(int argc, char** argv) {

  int file;
  int a, b, c, d, e; // arguments

  if ((file = open(devName, O_RDWR)) < 0) {
    fprintf(stderr, "I2C: Failed to access %d\n", devName);
    exit(1);
  }

  //printf("I2C: acquiring buss to 0x%x\n", ADDRESS);

  if (ioctl(file, I2C_SLAVE, ADDRESS) < 0) {
    fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);
    exit(1);
  }

  if( retrieveValue(4, &file) == 366 ){
    printf("ok", 0 );
  }
  else {
    printf("bad", 0 );
  }

  close(file);
  return (EXIT_SUCCESS);
}
