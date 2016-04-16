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

  printf("Monitaur-MoistureValue1 %d\n", retrieveValue(4, &file));
  float myfloat = (( retrieveValue(8, &file)) * 0.1 );
  printf("DHT1122-Celsius %d\n", myfloat);
  myfloat = (( retrieveValue(9, &file)) * 0.1 );
  printf("DHT1122-Fahrenheit %d\n", myfloat);
  printf("DHT1122-Humidity %d\n", retrieveValue(10, &file));
  printf("Monitaur-pumpAveON %d\n", retrieveValue(16, &file));
  printf("Monitaur-pumpAveOff %d\n", retrieveValue(17, &file));
  printf("Monitaur-dryLimit %d\n", retrieveValue(18, &file));
  printf("Monitaur-wetLimit %d\n", retrieveValue(19, &file));

  close(file);
  return (EXIT_SUCCESS);
}
