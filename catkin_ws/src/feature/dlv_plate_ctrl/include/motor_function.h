#ifndef __MOTOR_FUNCTION_H__
#define __MOTOR_FUNCTION_H__

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

static uint8_t CRCHighTable[256];
static uint8_t CRCLowTable[256];

extern int serialPort;  //serial port object

typedef struct {
  uint8_t data[20];
  int length;
}serialData;

typedef struct {
  serialData right_wheel;
  serialData left_wheel;
  double linear_x;
  double angular_z;
}carInfo;

void serialInit();
void transmitData(serialData *transmitMsg);
void receiveData(serialData *receiveMsg);
void CRC16Generate(serialData *msg);

#endif