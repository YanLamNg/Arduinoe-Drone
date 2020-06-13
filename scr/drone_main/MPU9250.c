
#include "MPU9250.h"


uint8_t readByte(int address, int subAddress)
{
    uint8_t data = 0; // `data` will store the register data
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                   // Put slave register address in Tx buffer
    Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, 1, true);  // Read one byte from slave register address
    if (Wire.available()) 
      data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void readBytes_f(int address, int subAddress, int count, uint8_t * dest)
{
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, count, true);  // Read bytes from slave register address
    while (Wire.available())
    {
        dest[i++] = Wire.read();
    } // Put read results in the Rx buffer
}


void I2C_read(int dev_addr, int reg_addr, int len, int16_t* data) {
  uint16_t* next = data;
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(dev_addr, len, true);
  while (Wire.available() && len--) {
    *next = Wire.read() << 8 | Wire.read();
    next++;
  }
}

void I2C_writeByte(int dev_addr, int reg_addr, int8_t data) {
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(data);
  Wire.endTransmission(true);
}