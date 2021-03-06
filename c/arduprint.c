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

  unsigned int cmd[16];
  // initialize command to get moisture value 1
  cmd[0] = call;
  // these other bytes are irrelevant at the moment
  cmd[1] = 1;
  cmd[2] = 2;
  cmd[3] = 3;
  cmd[4] = 4;
  cmd[5] = 5;
  cmd[6] = 6;
  cmd[7] = 7;
  cmd[8] = 8;
  cmd[9] = 9;
  cmd[10] = 10;

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

  if ((file = open(devName, O_RDWR)) < 0) {
    fprintf(stderr, "I2C: Failed to access %d\n", devName);
    exit(1);
  }

  //printf("I2C: acquiring buss to 0x%x\n", ADDRESS);

  if (ioctl(file, I2C_SLAVE, ADDRESS) < 0) {
    fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);
    exit(1);
  }


  printf("Watchdog %d\n", retrieveValue(3, &file));
  printf("Moisture Value 1 %d\n", retrieveValue(4, &file));
  printf("Moisture Value 2 %d\n", retrieveValue(5, &file));
  printf("Moisture Value 3 %d\n", retrieveValue(6, &file));
  printf("ok2 %d\n", retrieveValue(7, &file));
  printf("celsius %d\n", retrieveValue(8, &file));
  printf("farenheit %d\n", retrieveValue(9, &file));
  printf("humidity %d\n", retrieveValue(10, &file));
  printf("ok %d\n", retrieveValue(11, &file));
  printf("pumpOn %d\n", retrieveValue(12, &file));
  printf("pumpOff %d\n", retrieveValue(13, &file));
  printf("sensorHighValue %d\n", retrieveValue(14, &file));
  printf("sensorLowValue %d\n", retrieveValue(15, &file));
  printf("AveON %d\n", retrieveValue(16, &file));
  printf("AveOff %d\n", retrieveValue(17, &file));
  printf("dryLimit %d\n", retrieveValue(18, &file));
  printf("wetLimit %d\n", retrieveValue(19, &file));
  printf("OnCountMin %d\n", retrieveValue(20, &file));
  printf("pumpOnCount %d\n", retrieveValue(21, &file));
  printf("pumpOffCount %d\n", retrieveValue(22, &file));

  close(file);
  return (EXIT_SUCCESS);
}
