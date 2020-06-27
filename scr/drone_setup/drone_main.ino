#include <EEPROM.h>
#include <Wire.h>
#include "MPU9250.h"


double acc[3], tmp, gyro[3];
double gyro_error[3] = {0};
double gyro_angle[3] = {0};
double acc_error[3] = {0};
double acc_angle[3] = {0};
double last_gyro_timer = millis();
double asax, asay, asaz;
int16_t mgnt[3];
double  Mgnt[3];


void setup_MPU9250() {
  Serial.print(" initize MPU9250 \n");
  I2C_writeByte(MPU9250_ADDRESS, 0x6B, 0);
  calibrate_gyro(1000); 
  for(int i = 0; i < 3; i++){
      gyro_angle[i] = 0;
  }
  last_gyro_timer = millis();
}

void setup_AK8963() {
  I2C_writeByte(MPU9250_ADDRESS, 0x37, 0x22);
  delay(500);
  I2C_writeByte(AK8963_ADDRESS, 0x0A, 0x00);
  delay(500);
  I2C_writeByte(AK8963_ADDRESS, 0x0A, 0x1F);
  delay(500);
  int8_t data[3];
  readBytes_f(AK8963_ADDRESS,0x10 ,3, data);
  asax = (data[0]-128)*0.5/128+1;
  asay = (data[1]-128)*0.5/128+1;
  asaz = (data[2]-128)*0.5/128+1;

  I2C_writeByte(AK8963_ADDRESS, 0x0A, 0x00);
  delay(500);
  I2C_writeByte(AK8963_ADDRESS, 0x0A, 0x16);
  delay(500);
}

void calibrate_gyro(double calibration_size) {
  int16_t data[7];
  
  for (int i = 0; i < calibration_size; i++) {
    I2C_read(MPU9250_ADDRESS, 0x3B, 14, data);

    //we assume z acc is = 1G = 16384LSB ,x & y = 0
    acc_error[0] += (double)data[0] / calibration_size;
    acc_error[1] += (double)data[1] / calibration_size;
    acc_error[2] += ((double)data[2] - 16384.0) / calibration_size;
    
    for(int i = 0; i < 3; i++){
      gyro_error[i] += (double)data[i+4] / calibration_size;
    }
    
  }
}

void read_gyro_data() {
  //  Wire.beginTransmission(MPU9250_ADDRESS);
  //  Wire.write(0x3B);
  //  Wire.endTransmission(false);
  //  Wire.requestFrom(MPU9250_ADDRESS, 14, true);
  //  int count = 0;
  //  acc[0] = Wire.read()<<8|Wire.read();
  //  acc[1] = Wire.read()<<8|Wire.read();
  //  acc[2] = Wire.read()<<8|Wire.read();
  //  tmp = Wire.read()<<8|Wire.read();
  //  gyro[0] = Wire.read()<<8|Wire.read() ;//* (gyro_timer - last_gyro_timer)/ 10000.0;
  //  gyro[1] = Wire.read()<<8|Wire.read() ;//* (gyro_timer - last_gyro_timer)/ 10000.0;
  //  gyro[2] = Wire.read()<<8|Wire.read();

  int16_t data[7] = {0};
  I2C_read(MPU9250_ADDRESS, 0x3B, 14, data);
  int theTime = millis();
  double period = (theTime - last_gyro_timer) / 1000.0/131;

  acc[0] = (data[0] - acc_error[0]) / 16384.0;
  acc[1] = (data[1] - acc_error[1]) / 16384.0;
  acc[2] = (data[2] - acc_error[2]) / 16384.0;
  tmp = data[3];
  gyro[0] = (data[4] - gyro_error[0]) * period;
  gyro[1] = (data[5] - gyro_error[1]) * period;
  gyro[2] = (data[6] - gyro_error[2]) * period;

  last_gyro_timer = theTime;
}

void update_gyro_angle() {
  acc_angle[0] = atan(acc[1] / sqrt(acc[0]*acc[0] + acc[2]*acc[2])) * 180 / PI;
  acc_angle[1] = -atan(acc[0] / sqrt(acc[1]*acc[1] + acc[2]*acc[2])) * 180 / PI;
  
  gyro_angle[0] += gyro[0] ;
  gyro_angle[1] += gyro[1] ;
  gyro_angle[2] += gyro[2] ;

  gyro_angle[0] = gyro_angle[0] * GYRO_ACC_ANGLE_RATIO + acc_angle[0] * (1 - GYRO_ACC_ANGLE_RATIO);
  gyro_angle[1] = gyro_angle[1] * GYRO_ACC_ANGLE_RATIO + acc_angle[1] * (1 - GYRO_ACC_ANGLE_RATIO);
  //acc.angle
}

void read_mgnt_data(){
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(readByte(AK8963_ADDRESS, 0x02) & 0x01) { // wait for magnetometer data ready bit to be set
      readBytes_f(AK8963_ADDRESS, 0x03, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
      uint8_t c = rawData[6]; // End data read by reading ST2 register
      if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
          mgnt[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
          mgnt[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
          mgnt[2] = ((int16_t)rawData[5] << 8) | rawData[4];

          Mgnt[0] = mgnt[0]*0.15;  // Turn the MSB and LSB into a signed 16-bit value
          Mgnt[1] = mgnt[1]*0.15;  // Data stored as little Endian
          Mgnt[2] = mgnt[2]*0.15;
      }
  }
//   I2C_writeByte(AK8963_ADDRESS, 0x0A, 0x12);
//  delay(500);
//  int16_t data[3];
//  I2C_read(AK8963_ADDRESS, 0x03, 6, data);
//  for (int i = 0; i < 3; i++) {
//    mgnt[i] = data[i];
//   
//  }

  
//  Wire.beginTransmission(AK8963_ADDRESS);
//  Wire.write(0x03);
//  Wire.endTransmission(false);
//  Wire.requestFrom(AK8963_ADDRESS, 6, true);
//
//  for (int i = 0; i < 3; i++) {
//    mgnt[i] = Wire.read() << 8 | Wire.read();
//    //mgnt[i] *= 0.15;
//  }
}
/*
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
*/

void print_gyro_data() {
      Serial.print(" Acce: x=");
      Serial.print(acc[0]);
      Serial.print(" y=");
      Serial.print(acc[1]);
      Serial.print(" z=");
      Serial.print(acc[2]);
  //    Serial.print(" tmp=");
  //    Serial.print(((float)tmp-21) / 333.87 + 21.0);
//  Serial.print(" Gyro x=");
//  Serial.print(gyro[0]);
//  Serial.print(" y=");
//  Serial.print(gyro[1]);
//  Serial.print(" z=");
//  Serial.print(gyro[2]);
  Serial.print("\n");
}

void print_gyro_angle() {
  Serial.print(" gyro_angle: x= ");
  Serial.print(gyro_angle[0]);
  Serial.print(" y= ");
  Serial.print(gyro_angle[1]);
  Serial.print(" z= ");
  Serial.print(gyro_angle[2]);
  Serial.print("  Acce: x=");
  Serial.print(acc_angle[0]);
  Serial.print(" y=");
  Serial.print(acc_angle[1]);
  Serial.print("\n");
}

void print_mgnt_data() {
  Serial.print("Mgnt: x= ");
  Serial.print(Mgnt[0]);
  Serial.print(" y= ");
  Serial.print(Mgnt[1]);
  Serial.print(" z= ");
  Serial.print(Mgnt[2]);
  // Serial.print("\n");
  //  Serial.print(" total: ");
  //  Serial.print(sqrt(double(mgnt[0]*mgnt[0]+mgnt[1]*mgnt[1]+mgnt[2]*mgnt[2])));
}

void setup() {
  Wire.begin();
  Serial.begin(SERIAL_FREQUENCE);
  setup_MPU9250();
  setup_AK8963();

}

void loop() {
  read_gyro_data();
  update_gyro_angle();
  read_mgnt_data();
  
  print_mgnt_data();
  print_gyro_angle();
  //print_gyro_data();

}
