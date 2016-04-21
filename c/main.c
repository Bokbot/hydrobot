#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
 
// The PiWeather board i2c address
#define ADDRESS 0x6b
 
// The I2C bus: This is for V2 pi's. For V1 Model B you need i2c-0
static const char *devName = "/dev/i2c-1";
 
int main(int argc, char** argv) {
 
  if (argc == 1) {
    printf("Supply one or more commands to send to the Arduino\n");
    exit(1);
  }
 
  printf("I2C: Connecting\n");
  int file;
 
  if ((file = open(devName, O_RDWR)) < 0) {
    fprintf(stderr, "I2C: Failed to access %d\n", devName);
    exit(1);
  }
 
  printf("I2C: acquiring buss to 0x%x\n", ADDRESS);
 
  if (ioctl(file, I2C_SLAVE, ADDRESS) < 0) {
    fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);
    exit(1);
  }
 
  int arg;
 
  for (arg = 1; arg < argc; arg++) {
    int val;
    unsigned int cmd[16];
 
    if (0 == sscanf(argv[arg], "%d", &val)) {
      fprintf(stderr, "Invalid parameter %d \"%s\"\n", arg, argv[arg]);
      exit(1);
    }
 
    printf("Sending %d\n", val);
 
    cmd[0] = val;
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
    if (write(file, cmd, 11) == 11) {

      // As we are not talking to direct hardware but a microcontroller we
      // need to wait a short while so that it can respond.
      //
      // 1ms seems to be enough but it depends on what workload it has
      usleep(10000);
 
      char buf[3];
      if (read(file, buf, 2) == 2) {
    int temp = (int) buf[0];
    int temp2 = (int) buf[1];
    int resultant = buf[0] << 8 | buf[1];
 
    printf("temp %d\n", temp);
    printf("temp2 %d\n", temp2);
    printf("Received %d\n", resultant);
      }
      usleep(10000);
    }
 
    // Now wait else you could crash the arduino by sending requests too fast
    usleep(10000);
  }
 
  close(file);
  return (EXIT_SUCCESS);
}
