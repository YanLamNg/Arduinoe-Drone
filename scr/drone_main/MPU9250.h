#ifndef MPU9250_h
#define MPU9250_h

#include "Arduino.h"
#include "Wire.h"

uint8_t readByte(int address, int subAddress);
void readBytes_f(int address, int subAddress, int count, uint8_t * dest);
void I2C_read(int dev_addr, int reg_addr, int len, int16_t* data);
void I2C_writeByte(int dev_addr, int reg_addr, int8_t data) ;

#endif