#include <EEPROM.h>
#include <Wire.h>
#include <ArduinoJson.h>

#define MPU9250_ADDRESS 0x68
#define AK8963_ADDRESS 0x0C
#define AK8963_CNTL 0x0A

#define MFS_16BITS 0x01    // 0.15 mG per LSB
#define M_100Hz 0x06
#define SERIAL_FREQUENCE 57600
#define GYRO_ACC_ANGLE_RATIO 0.95


//For PWM
#define ESC_1_PIN 4
#define ESC_2_PIN 5
#define ESC_3_PIN 6
#define ESC_4_PIN 7
#define SGINAL_LED 12

#define THROTTLE_CH ch3_PWM
#define YAW_CH ch4_PWM
#define PITCH_CH ch2_PWM
#define ROLL_CH ch1_PWM

#define pitchAngle droneAngle[1]
#define yawAngle droneAngle[2]
#define rollAngle droneAngle[0]

#define pitchGyro gyro[1]
#define yawGyro gyro[2]
#define rollGyro gyro[0]

#define MAX_YAW_SPEED 5  //  degree per second
#define MAX_PITCH_ANGLE 20 // in degree
#define MAX_ROLL_ANGLE 20 // in degree

#define DEBUG_MODE true


#define I_YAW 0.0
#define P_YAW 1.0
#define D_YAW 0.0//3.0
#define I_PITCH 0.5//0.5
#define P_PITCH 3.0
#define D_PITCH 15.0//3.0
#define I_ROLL 0.0//0.5
#define P_ROLL 1.0
#define D_ROLL 0.0 //3.0



int16_t EEPROM_ERR_ADDR = 0;

boolean CALIBRATION = false;
double acc[3], tmp, gyro[3];
float gyroError[3] = {0};
double droneAngle[3] = {0};
float accError[3] = {0};
double accAngle[3] = {0};
unsigned long lastGyroTimer = millis();
double asax, asay, asaz;
int16_t mgnt[3];
double  Mgnt[3];


double prevYawAngle, prevPitchAngle, prevRollAngle;

float yawTarget, pitchTarget, rollTarget;

DynamicJsonDocument jsonDoc(512);
//variable for PWM receiver


boolean channel_1, channel_2, channel_3, channel_4;
unsigned long ch1_timer, ch2_timer, ch3_timer, ch4_timer;
long ch1_PWM, ch2_PWM, ch3_PWM, ch4_PWM;



double yawOutput, pitchOutput, rollOutput;
float motorFL, motorFR, motorBL, motorBR;
float totalYawError, totalPitchError, totalRollError;

struct DroneError {
  float gyro_err[3];
  float acc_err[3];
};

uint8_t I2C_readByte(int address, int subAddress)
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

void I2C_readBytes(int address, int subAddress, int count, uint8_t * dest)
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

void I2C_read_16bits(int dev_addr, int reg_addr, int len, int16_t* data) {
  uint16_t* next = data;
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(dev_addr, len*2, true);
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



void setup_MPU9250() {
 
  I2C_writeByte(MPU9250_ADDRESS, 0x6B, 0);
  if(CALIBRATION){
    calibrate_gyro(1000);
  }
  else{
    get_eepromError(EEPROM_ERR_ADDR);
  }
  for (int i = 0; i < 3; i++) {
    droneAngle[i] = 0;
  }
  lastGyroTimer = millis();
}

void setup_AK8963() {
  I2C_writeByte(MPU9250_ADDRESS, 0x37, 0x22);
  delay(500);
  I2C_writeByte(AK8963_ADDRESS, 0x0A, 0x00);
  delay(500);
  I2C_writeByte(AK8963_ADDRESS, 0x0A, 0x1F);
  delay(500);
  int8_t data[3];
  I2C_readBytes(AK8963_ADDRESS, 0x10 , 3, data);
  asax = (data[0] - 128) * 0.5 / 128 + 1;
  asay = (data[1] - 128) * 0.5 / 128 + 1;
  asaz = (data[2] - 128) * 0.5 / 128 + 1;

  I2C_writeByte(AK8963_ADDRESS, 0x0A, 0x00);
  delay(500);
  I2C_writeByte(AK8963_ADDRESS, 0x0A, 0x16);
  delay(500);
}


void put_eepromError(int16_t eeAddress, float gyro_e[3], float acc_e[3]){
  DroneError errorData;
  for(int i = 0; i < 3; i++){
    errorData.gyro_err[i] = gyro_e[i];
    errorData.acc_err[i] = acc_e[i];
  }
  
  EEPROM.put(eeAddress, errorData);
}

void get_eepromError(int16_t eeAddress){
  DroneError errorData;
  EEPROM.get(0, errorData);
  for(int i = 0; i < 3; i++){
    gyroError[i] = errorData.gyro_err[i];
    accError[i] = errorData.acc_err[i];
  }
}

void calibrate_gyro(double calibration_size) {
  int16_t data[14];

  for (int i = 0; i < calibration_size; i++) {
    I2C_read_16bits(MPU9250_ADDRESS, 0x3B, 7, data);

    //we assume z acc is = 1G = 16384LSB ,x & y = 0

    accError[0] += data[0] / calibration_size;
    accError[1] += data[1] / calibration_size;
    accError[2] +=(data[2]  - 16384.0) / calibration_size;
//   Serial.print(data[0] / calibration_size);
   
    for (int i = 0; i < 3; i++) {
    gyroError[i] += data[4+i] / calibration_size;
    }
  }
  put_eepromError(EEPROM_ERR_ADDR, (int16_t)gyroError, (int16_t)accError);
}


void read_gyro_data() {
  int16_t data[7] ;
  I2C_read_16bits(MPU9250_ADDRESS, 0x3B, 7, data);
  
  unsigned long theTime = millis();
  double period = (theTime - lastGyroTimer) / 1000.0 / 131;

  acc[0] = (data[0] - accError[0]) / 16384.0;
  acc[1] = (data[1] - accError[1]) / 16384.0;
  acc[2] = (data[2] - accError[2]) / 16384.0;
  tmp = data[3] ;
  gyro[0] = (data[4]  - gyroError[0]) * period;
  gyro[1] = (data[5]  - gyroError[1]) * period;
  gyro[2] = (data[6]  - gyroError[2]) * period;

  lastGyroTimer = theTime;
}

void update_droneAngle() {
  accAngle[0] = atan(acc[1] / sqrt(acc[0] * acc[0] + acc[2] * acc[2])) * 180 / PI;
  accAngle[1] = -atan(acc[0] / sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * 180 / PI;

  droneAngle[0] = (droneAngle[0]+gyro[0]) * GYRO_ACC_ANGLE_RATIO + accAngle[0] * (1 - GYRO_ACC_ANGLE_RATIO);
  droneAngle[1] = (droneAngle[1]+gyro[1]) * GYRO_ACC_ANGLE_RATIO + accAngle[1] * (1 - GYRO_ACC_ANGLE_RATIO);
  droneAngle[2] += gyro[2] ;

  //acc.angle
}



void calculateMoterOutput(){
  float deltaYaw = yawAngle - prevYawAngle;
  float deltaPitch = pitchAngle - prevPitchAngle;
  float deltaRoll = rollAngle - prevRollAngle;


  if(THROTTLE_CH > 2000){
    THROTTLE_CH = 2000;
  }
  else if(THROTTLE_CH < 1000){
    THROTTLE_CH = 1000;
  }

  if(YAW_CH > 2000){
    YAW_CH = 2000;
  }
  else if(YAW_CH < 1000){
    YAW_CH = 1000;
    
  }
  if(PITCH_CH > 2000){
    PITCH_CH = 2000;
  }
  else if(PITCH_CH < 1000){
    PITCH_CH = 1000;
    
  }
  if(ROLL_CH > 2000){
    ROLL_CH = 2000;
  }
  else if(ROLL_CH < 1000){
    ROLL_CH = 1000;
  }
  
  yawTarget = ((YAW_CH - 1500)/ 500.0) * MAX_YAW_SPEED;
  pitchTarget = ((PITCH_CH - 1500)/ 500.0) * MAX_PITCH_ANGLE;
  rollTarget = ((ROLL_CH - 1500)/ 500.0) * MAX_ROLL_ANGLE;
  
  
  
  float yawError = yawTarget + yawAngle ;
  float pitchError = pitchTarget - pitchAngle;
  float rollError = rollTarget - rollAngle;

  totalYawError += yawError;
  totalPitchError += pitchError;
  totalRollError += rollError;
  
  yawOutput = I_YAW * totalYawError + P_YAW * yawError + D_YAW * deltaYaw;
  pitchOutput = I_PITCH * totalPitchError + P_PITCH * pitchError + D_PITCH * deltaPitch;
  rollOutput = I_ROLL * totalRollError + P_ROLL * rollError + D_ROLL * deltaRoll;

  motorFL = THROTTLE_CH + pitchOutput - rollOutput + yawOutput;
  motorFR = THROTTLE_CH + pitchOutput + rollOutput - yawOutput;
  motorBL = THROTTLE_CH - pitchOutput - rollOutput - yawOutput;
  motorBR = THROTTLE_CH - pitchOutput + rollOutput + yawOutput;

  prevYawAngle = yawAngle;
  prevPitchAngle = pitchAngle;
  prevRollAngle = rollAngle;
  Serial.print("pitchOutput:");
  Serial.print(pitchOutput);
  Serial.print("     totalPitchError:");
  Serial.print(totalPitchError*I_PITCH);
  Serial.print("     pitchError:");
  Serial.print(pitchError*P_PITCH);
  Serial.print("     deltaPitch:");
  Serial.print(deltaPitch*D_PITCH);  
  Serial.print("     pitchTarget:");
  Serial.print(pitchTarget);
  Serial.print("     pitchAngle:");
  Serial.print(pitchAngle);
  Serial.println();

//  
//  Serial.print("ACC[0]:");
//  Serial.print(acc[0]);
//  
//  Serial.print("     gyr[0]:");
//  Serial.print(gyro[0]);
//  Serial.print("     pitchAngle:");
//  Serial.print(pitchAngle);
//  Serial.println();
  
}

void read_mgnt_data() {
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if (I2C_readByte(AK8963_ADDRESS, 0x02) & 0x01) { // wait for magnetometer data ready bit to be set
    I2C_readBytes(AK8963_ADDRESS, 0x03, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      mgnt[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
      mgnt[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
      mgnt[2] = ((int16_t)rawData[5] << 8) | rawData[4];

      Mgnt[0] = mgnt[0] * 0.15; // Turn the MSB and LSB into a signed 16-bit value
      Mgnt[1] = mgnt[1] * 0.15; // Data stored as little Endian
      Mgnt[2] = mgnt[2] * 0.15;
    }
  }
}


void setup_interrupt(){
  PCICR |= (1 << PCIE0);      //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);    //Set input 8 to trigger an interrupt
  PCMSK0 |= (1 << PCINT1);    //Set input 9 to trigger an interrupt
  PCMSK0 |= (1 << PCINT2);    //Set input 10 to trigger an interrupt
  PCMSK0 |= (1 << PCINT3);    //Set input 11 to trigger an interrupt
}

//interrupt for PWD 
ISR(PCINT0_vect) {
  long curr = micros();

  if(PINB & B00000001){           //channal 1
    if(channel_1 == false){
      channel_1 = true;
      ch1_timer = curr;
    }
  }
  else if(channel_1 == true){
    channel_1 = false;
    ch1_PWM = curr - ch1_timer;
  }

  
  if(PINB & B00000010){          //channal 2
    if(channel_2 == false){
      channel_2 = true;
      ch2_timer = curr;
    }
  }
  else if(channel_2 == true){
    channel_2 = false;
    ch2_PWM = curr - ch2_timer;
  }
  
  if(PINB & B00000100){          //channal 3
    if(channel_3 == false){
      channel_3 = true;
      ch3_timer = curr;
    }
  }
  else if(channel_3 == true){
    channel_3 = false;
    ch3_PWM = curr - ch3_timer;
  }
  
  
  if(PINB & B00001000){          //channal 4
    if(channel_4 == false){
      channel_4 = true;
      ch4_timer = curr;
    }
  }
  else if(channel_4 == true){
    channel_4 = false;
    ch4_PWM = curr - ch4_timer;
  }

}


void print_gyro_data() {
  Serial.print(" Acce: x=");
  Serial.print(acc[0]);
  Serial.print(" y=");
  Serial.print(acc[1]);
  Serial.print(" z=");
  Serial.print(acc[2]);
  Serial.print(" tmp=");
  Serial.print(((float)tmp-21) / 333.87 + 21.0);
  Serial.print(" Gyro x=");
  Serial.print(gyro[0]);
  Serial.print(" y=");
  Serial.print(gyro[1]);
  Serial.print(" z=");
  Serial.print(gyro[2]);
  Serial.print("\n");
}

void print_droneAngle() {
  Serial.print(" droneAngle: x= ");
  Serial.print(droneAngle[0]);
  Serial.print(" y= ");
  Serial.print(droneAngle[1]);
  Serial.print(" z= ");
  Serial.print(droneAngle[2]);

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


void printPWD(){
  Serial.print("Throttle: ");
  Serial.print(THROTTLE_CH);
  Serial.print("\tPitch: ");
  Serial.print(PITCH_CH);
  Serial.print("\tRoll: ");
  Serial.print(ROLL_CH);
  Serial.print("\tYaw: ");
  Serial.print(YAW_CH);
  Serial.println();
}

void printMotor(){
  Serial.print("FL: ");
  Serial.print(motorFL);
  Serial.print("\tFR: ");
  Serial.print(motorFR);
  Serial.print("\tBL: ");
  Serial.print(motorBL);
  Serial.print("\tBR: ");
  Serial.print(motorBR);
  Serial.println();
}



void printTargetAngle(){
  
  Serial.print("yawTarget: ");
  Serial.print(yawTarget);
  Serial.print("\tpitchTarget: ");
  Serial.print(pitchTarget);
  Serial.print("\trollTarget: ");
  Serial.print(rollTarget);

  Serial.println();
}
int countJson = 0;  //testing
void sendJsonData(){

    if(countJson % 20 == 0){
      jsonDoc["yawTarget"] = yawTarget;
      jsonDoc["pitchTarget"] = pitchTarget;
      jsonDoc["rollTarget"] = rollTarget;
      jsonDoc["motor_FL"] = motorFL;
      jsonDoc["motor_FR"] = motorFR;
      jsonDoc["motor_BL"] = motorBL;
      jsonDoc["motor_BR"] = motorBR;
      jsonDoc["throttle_ch"] = THROTTLE_CH;
      jsonDoc["pitch_ch"] = PITCH_CH;
      jsonDoc["roll_ch"] = ROLL_CH;
      jsonDoc["x_angle"] = droneAngle[0];
      jsonDoc["z_angle"] = droneAngle[2];
      jsonDoc["y_angle"] = droneAngle[1];
      jsonDoc["gyro_x"] = gyro[0];
      jsonDoc["gyro_y"] = gyro[1];
      jsonDoc["gyro_z"] = gyro[2];
      jsonDoc["acc_x"] = acc[0];
      jsonDoc["acc_y"] = acc[1];
      jsonDoc["acc_z"] = acc[2];
      serializeJson(jsonDoc,Serial);
      Serial.println();
  
      countJson = 0;
    }
    countJson++;
    
    
    
}


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(SERIAL_FREQUENCE);

  
//  Serial.println("Initialize drone.");
  yawOutput = 0;
  pitchOutput = 0;
  rollOutput = 0;
//  Serial.println("Setting up MPU925.");
  setup_MPU9250();
//  Serial.println("Setting up AK8963.");
  setup_AK8963();
//  Serial.println("Setting up interrup for PWM");
  setup_interrupt();
//  Serial.println("finish Setup");
}

void loop() {
   
  read_gyro_data();
  update_droneAngle();

  calculateMoterOutput();
 // print_droneAngle();
//  printMotor();
  //sendJsonData();
  
//  read_mgnt_data();
//  printPWD();
//  print_mgnt_data();
//  print_droneAngle();
//  print_gyro_data();
  // put your main code here, to run repeatedly:

}
