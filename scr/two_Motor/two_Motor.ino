
#include <EEPROM.h>
#include <Wire.h>
//#include <ArduinoJson.h>

#define DEBUG false
#define SEND_JSON true

#define MPU9250_ADDRESS 0x68
#define AK8963_ADDRESS 0x0C
#define AK8963_CNTL 0x0A

#define MFS_16BITS 0x01 // 0.15 mG per LSB
#define M_100Hz 0x06
#define SERIAL_FREQUENCE 57600
#define GYRO_ACC_ANGLE_RATIO 0.99

//For PWM
#define ESC_1_PIN 4
#define ESC_2_PIN 5
#define ESC_3_PIN 6
#define ESC_4_PIN 7
#define SGINAL_LED 12
#define MAX_ESC_OUTPUT 2000
#define MIN_ESC_OUTPUT 1050

#define throttle_ch ch3_PWM
#define yaw_ch ch4_PWM
#define pitch_ch ch2_PWM
#define roll_ch ch1_PWM

#define pitchAngle droneAngle[1]
#define yawAngle droneAngle[2]
#define rollAngle droneAngle[0]

#define pitchGyro gyro[1]
#define yawGyro gyro[2]
#define rollGyro gyro[0]

#define MAX_YAW_SPEED 5    //  degree per second
#define MAX_PITCH_ANGLE 20 // in degree
#define MAX_ROLL_ANGLE 20  // in degree

#define DEBUG_MODE false
#define CALIBRATION false

#define I_YAW i_yaw
#define P_YAW p_yaw
#define D_YAW d_yaw     //3.0
#define DD_YAW dd_yaw  //3.0
#define I_PITCH i_pitch//0.02   //0.02
#define P_PITCH p_pitch//2.0  //2.0 
#define D_PITCH d_pitch  //80.0
#define DD_PITCH dd_pitch  //-50.0 
#define I_ROLL i_roll    //0.5
#define P_ROLL p_roll
#define D_ROLL d_roll    //3.0
#define DD_ROLL dd_roll //3.0

float i_pitch = 0.001;
float p_pitch = 4.0;
float d_pitch = 80.0;
float dd_pitch = -200.0;


float i_roll = 0.001;
float p_roll = 4.0;
float d_roll = 80.0;
float dd_roll = -200.0;

float i_yaw = 0.0;
float p_yaw = 1.0;
float d_yaw = 10.0;
float dd_yaw = 0.0;

int16_t EEPROM_ERR_ADDR = 0;

double acc[3], tmp, gyro[3];
float gyroError[3] = {0};
double droneAngle[3] = {0};
float accError[3] = {0};
double accAngle[3] = {0};
unsigned long lastGyroTimer = millis();
double asax, asay, asaz;
int16_t mgnt[3];
double Mgnt[3];

double prevYawAngle, prevPitchAngle, prevRollAngle;
float prevDeltaYaw, prevDeltaPitch, prevDeltaRoll;

float yawTarget, pitchTarget, rollTarget;

//variable for PWM receiver

boolean channel_1, channel_2, channel_3, channel_4;
unsigned long ch1_timer, ch2_timer, ch3_timer, ch4_timer;
int ch1_PWM, ch2_PWM, ch3_PWM, ch4_PWM;

double yawOutput, pitchOutput, rollOutput;
float motorFL, motorFR, motorBL, motorBR;
float totalYawError, totalPitchError, totalRollError;

//ESC

int esc[4];
long escTimer[4];
long escEndTime[4];
long escCycleTime = 20000; //20ms
long prevEscTime;
int loopCount = 0;
struct DroneError
{
    float gyro_err[3];
    float acc_err[3];
};

uint8_t I2C_readByte(int address, int subAddress)
{
    uint8_t data = 0;                   // `data` will store the register data
    Wire.beginTransmission(address);    // Initialize the Tx buffer
    Wire.write(subAddress);             // Put slave register address in Tx buffer
    Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, 1, true); // Read one byte from slave register address
    if (Wire.available())
        data = Wire.read(); // Fill Rx buffer with result
    return data;            // Return data read from slave register
}

void I2C_readBytes(int address, int subAddress, int count, uint8_t *dest)
{
    Wire.beginTransmission(address); // Initialize the Tx buffer
    Wire.write(subAddress);          // Put slave register address in Tx buffer
    Wire.endTransmission(false);     // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, count, true); // Read bytes from slave register address
    while (Wire.available())
    {
        dest[i++] = Wire.read();
    } // Put read results in the Rx buffer
}

void I2C_read_16bits(int dev_addr, int reg_addr, int len, int16_t *data)
{
    uint16_t *next = data;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.endTransmission(false);
    Wire.requestFrom(dev_addr, len * 2, true);
    while (Wire.available() && len--)
    {
        *next = Wire.read() << 8 | Wire.read();
        next++;
    }
}

void I2C_writeByte(int dev_addr, int reg_addr, int8_t data)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.write(data);
    Wire.endTransmission(true);
}

void debug_message(const char *text)
{
    if (DEBUG)
    {
        Serial.println(text);
    }
}

void setup_MPU9250()
{
    I2C_writeByte(MPU9250_ADDRESS, 0x6B, 0);
    if (CALIBRATION)
    {
        calibrate_gyro(5000);
    }
    else
    {
        get_eepromError(EEPROM_ERR_ADDR);
    }

    for (int i = 0; i < 3; i++)
    {
        droneAngle[i] = 0;
    }
    lastGyroTimer = millis();
}

void setup_AK8963()
{
    I2C_writeByte(MPU9250_ADDRESS, 0x37, 0x22);
    delay(500);
    I2C_writeByte(AK8963_ADDRESS, 0x0A, 0x00);
    delay(500);
    I2C_writeByte(AK8963_ADDRESS, 0x0A, 0x1F);
    delay(500);
    int8_t data[3];
    I2C_readBytes(AK8963_ADDRESS, 0x10, 3, data);
    asax = (data[0] - 128) * 0.5 / 128 + 1;
    asay = (data[1] - 128) * 0.5 / 128 + 1;
    asaz = (data[2] - 128) * 0.5 / 128 + 1;

    I2C_writeByte(AK8963_ADDRESS, 0x0A, 0x00);
    delay(500);
    I2C_writeByte(AK8963_ADDRESS, 0x0A, 0x16);
    delay(500);
}

void put_eepromError(int16_t eeAddress, float gyro_e[3], float acc_e[3])
{
    DroneError errorData;
    for (int i = 0; i < 3; i++)
    {
        errorData.gyro_err[i] = gyro_e[i];
        errorData.acc_err[i] = acc_e[i];
    }

    EEPROM.put(eeAddress, errorData);
}

void get_eepromError(int16_t eeAddress)
{
    DroneError errorData;
    EEPROM.get(0, errorData);
    for (int i = 0; i < 3; i++)
    {
        gyroError[i] = errorData.gyro_err[i];
        accError[i] = errorData.acc_err[i];
    }
}

void calibrate_gyro(double calibration_size)
{
    int16_t data[14];

    for (int i = 0; i < calibration_size; i++)
    {
        I2C_read_16bits(MPU9250_ADDRESS, 0x3B, 7, data);

        //we assume z acc is = 1G = 16384LSB ,x & y = 0

        accError[0] += data[0] / calibration_size;
        accError[1] += data[1] / calibration_size;
        accError[2] += (data[2] - 16384.0) / calibration_size;

        for (int i = 0; i < 3; i++)
        {
            gyroError[i] += data[4 + i] / calibration_size;
        }
    }
    put_eepromError(EEPROM_ERR_ADDR, (int16_t)gyroError, (int16_t)accError);
}

void read_gyro_data()
{
    int16_t data[7];
    I2C_read_16bits(MPU9250_ADDRESS, 0x3B, 7, data);

    unsigned long theTime = millis();
    double period = (theTime - lastGyroTimer) / 1000.0 / 131;

    acc[0] = (data[0] - accError[0]) / 16384.0;
    acc[1] = (data[1] - accError[1]) / 16384.0;
    acc[2] = (data[2] - accError[2]) / 16384.0;
    tmp = data[3];
    gyro[0] = (data[4] - gyroError[0]) * period;
    gyro[1] = (data[5] - gyroError[1]) * period;
    gyro[2] = (data[6] - gyroError[2]) * period;

    lastGyroTimer = theTime;
}

void update_droneAngle()
{
    accAngle[0] = atan(acc[1] / sqrt(acc[0] * acc[0] + acc[2] * acc[2])) * 180 / PI;
    accAngle[1] = -atan(acc[0] / sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * 180 / PI;

    droneAngle[0] = (droneAngle[0] + gyro[0]) * GYRO_ACC_ANGLE_RATIO + accAngle[0] * (1 - GYRO_ACC_ANGLE_RATIO);
    droneAngle[1] = (droneAngle[1] + gyro[1]) * GYRO_ACC_ANGLE_RATIO + accAngle[1] * (1 - GYRO_ACC_ANGLE_RATIO);
    // droneAngle[0] += droneAngle[1] * sin(gyro[2] * 3.142 / 180);
    // droneAngle[1] -= droneAngle[0] * sin(gyro[2] * 3.142 / 180);
    droneAngle[2] = gyro[2];

    //acc.angle
}

void calculateMoterOutput()
{
    // float deltaYaw = yawAngle - prevYawAngle;
    // float deltaPitch = pitchAngle - prevPitchAngle;
    // float deltaRoll = rollAngle - prevRollAngle;

    float deltaSquareYaw = prevDeltaYaw - gyro[2];
    float deltaSquarePitch = prevDeltaPitch -  gyro[0];
    float deltaSquareRoll = prevDeltaRoll -  gyro[1];

    if (throttle_ch > 2000)
    {
        throttle_ch = 2000;
    }
    else if (throttle_ch < 1050)
    {
        throttle_ch = 990;
        motorFL = throttle_ch;
        motorFR = throttle_ch;
        motorBL = throttle_ch;
        motorBR = throttle_ch;
        return;
    }

    if (yaw_ch > 2000)
    {
        yaw_ch = 2000;
    }
    else if (yaw_ch < 1000)
    {
        yaw_ch = 1000;
    }
    if (pitch_ch > 2000)
    {
        pitch_ch = 2000;
    }
    else if (pitch_ch < 1000)
    {
        pitch_ch = 1000;
    }
    if (roll_ch > 2000)
    {
        roll_ch = 2000;
    }
    else if (roll_ch < 1000)
    {
        roll_ch = 1000;
    }

    yawTarget = ((yaw_ch - 1500) / 500.0) * MAX_YAW_SPEED;
    pitchTarget = ((pitch_ch - 1500) / 500.0) * MAX_PITCH_ANGLE;
    rollTarget = ((roll_ch - 1500) / 500.0) * MAX_ROLL_ANGLE;



    float yawError =  gyro[2] - yawTarget;
    float pitchError = pitchAngle - pitchTarget;
    float rollError = rollAngle - rollTarget;

    totalPitchError += pitchError;
    totalRollError += rollError;

    yawOutput =  D_YAW * yawError + DD_YAW * deltaSquareYaw;
    pitchOutput = I_PITCH * totalPitchError + P_PITCH * pitchError + D_PITCH * gyro[1] + DD_PITCH * deltaSquarePitch;
    rollOutput = I_ROLL * totalRollError + P_ROLL * rollError + D_ROLL * gyro[0] + DD_ROLL * deltaSquareRoll;

    motorFL = throttle_ch + pitchOutput - rollOutput;// + yawOutput;
    motorFR = throttle_ch + pitchOutput + rollOutput;// - yawOutput;
    motorBL = throttle_ch - pitchOutput - rollOutput;// - yawOutput;
    motorBR = throttle_ch - pitchOutput + rollOutput;// + yawOutput;

    if (motorFL > MAX_ESC_OUTPUT)
    {
        motorFL = MAX_ESC_OUTPUT;
    }
    else if (motorFL < 1020)
    {
        motorFL = 1020;
    }

    if (motorFR > MAX_ESC_OUTPUT)
    {
        motorFR = MAX_ESC_OUTPUT;
    }
    else if (motorFR < 1020)
    {
        motorFR = 1020;
    }
    if (motorBL > MAX_ESC_OUTPUT)
    {
        motorBL = MAX_ESC_OUTPUT;
    }
    else if (motorBL < 1020)
    {
        motorBL = 1020;
    }
    if (motorBR > MAX_ESC_OUTPUT)
    {
        motorBR = MAX_ESC_OUTPUT;
    }
    else if (motorBR < 1020)
    {
        motorBR = 1020;
    }

    // prevYawAngle = yawAngle;
    // prevPitchAngle = pitchAngle;
    // prevRollAngle = rollAngle;

    prevDeltaYaw = gyro[2];//*0.4 + prevDeltaYaw*0.6;
    prevDeltaPitch = gyro[0];//*0.4 + prevDeltaPitch*0.6;
    prevDeltaRoll = gyro[1];//*0.4 + prevDeltaRoll*0.6;

   
}
void setup_esc()
{
    debug_message("Starting ESC");

    pinMode(ESC_1_PIN, OUTPUT);
    pinMode(ESC_2_PIN, OUTPUT);
    pinMode(ESC_3_PIN, OUTPUT);
    pinMode(ESC_4_PIN, OUTPUT);
    pinMode(SGINAL_LED, OUTPUT);

    prevEscTime = micros();

    debug_message("Waiting 1 seconds");
    delay(1000);
    debug_message("Start 20000ms pulse ");
    for (int i = 0; i < 150; i++)
    {
        PORTD |= B11110000;
        delayMicroseconds(1000);
        PORTD &= B00001111;
        delay(19);
    }
    debug_message("end 1000ms pulse ");
}

void send_esc()
{

    while (prevEscTime + escCycleTime > micros())
    {
    }
    long currTime;
    long timer = micros();
    PORTD |= B11110000;
    prevEscTime = timer;
    long t_1 = timer + motorFL;
    long t_2 = timer + motorFR;
    long t_3 = timer + motorBL;
    long t_4 = timer + motorBR;
    while (PORTD > B00001111)
    {
        currTime = micros();

        if (currTime >= t_1)
        {
            PORTD &= B11101111;
        }
        if (currTime >= t_2)
        {
            PORTD &= B11011111;
        }
        if (currTime >= t_3)
        {
            PORTD &= B10111111;
        }
        if (currTime >= t_4)
        {
            PORTD &= B01111111;
        }
    }
}

void setup_interrupt()
{
    PCICR |= (1 << PCIE0);   //Set PCIE0 to enable PCMSK0 scan.
    PCMSK0 |= (1 << PCINT0); //Set input 8 to trigger an interrupt
    PCMSK0 |= (1 << PCINT1); //Set input 9 to trigger an interrupt
    PCMSK0 |= (1 << PCINT2); //Set input 10 to trigger an interrupt
    PCMSK0 |= (1 << PCINT3); //Set input 11 to trigger an interrupt
}

//interrupt for PWD
ISR(PCINT0_vect)
{
    long curr = micros();

    if (PINB & B00000001)
    { //channal 1
        if (channel_1 == false)
        {
            channel_1 = true;
            ch1_timer = curr;
        }
    }
    else if (channel_1 == true)
    {
        channel_1 = false;
        ch1_PWM = curr - ch1_timer;
    }

    if (PINB & B00000010)
    { //channal 2
        if (channel_2 == false)
        {
            channel_2 = true;
            ch2_timer = curr;
        }
    }
    else if (channel_2 == true)
    {
        channel_2 = false;
        ch2_PWM = curr - ch2_timer;
    }

    if (PINB & B00000100)
    { //channal 3
        if (channel_3 == false)
        {
            channel_3 = true;
            ch3_timer = curr;
        }
    }
    else if (channel_3 == true)
    {
        channel_3 = false;
        ch3_PWM = curr - ch3_timer;
        if(ch3_PWM<1030){
            ch3_PWM = 970;
        }
    }

    if (PINB & B00001000)
    { //channal 4
        if (channel_4 == false)
        {
            channel_4 = true;
            ch4_timer = curr;
        }
    }
    else if (channel_4 == true)
    {
        channel_4 = false;
        ch4_PWM = curr - ch4_timer;
    }
}

void print_gyro_data()
{
    if (DEBUG)
    {

        Serial.print(" Acce: x=");
        Serial.print(acc[0]);
        Serial.print(" y=");
        Serial.print(acc[1]);
        Serial.print(" z=");
        Serial.print(acc[2]);
        Serial.print(" tmp=");
        Serial.print(((float)tmp - 21) / 333.87 + 21.0);
        Serial.print(" Gyro x=");
        Serial.print(gyro[0]);
        Serial.print(" y=");
        Serial.print(gyro[1]);
        Serial.print(" z=");
        Serial.print(gyro[2]);
        Serial.print("\n");
    }
}

void print_droneAngle()
{
    if (DEBUG)
    {

        Serial.print(" droneAngle: x= ");
        Serial.print(droneAngle[0]);
        Serial.print(" y= ");
        Serial.print(droneAngle[1]);
        Serial.print(" z= ");
        Serial.print(droneAngle[2]);

        Serial.print("\n");
    }
}

void print_mgnt_data()
{
    if (DEBUG)
    {

        Serial.print("Mgnt: x= ");
        Serial.print(Mgnt[0]);
        Serial.print(" y= ");
        Serial.print(Mgnt[1]);
        Serial.print(" z= ");
        Serial.print(Mgnt[2]);
    }
    // Serial.print("\n");
    //  Serial.print(" total: ");
    //  Serial.print(sqrt(double(mgnt[0]*mgnt[0]+mgnt[1]*mgnt[1]+mgnt[2]*mgnt[2])));
}

void printPWD()
{
    if (DEBUG)
    {

        Serial.print("Throttle: ");
        Serial.print(throttle_ch);
        Serial.print("\tPitch: ");
        Serial.print(pitch_ch);
        Serial.print("\tRoll: ");
        Serial.print(roll_ch);
        Serial.print("\tYaw: ");
        Serial.print(yaw_ch);
        Serial.println();
    }
}

void printMotor()
{
    if (DEBUG)
    {

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
}

void printTargetAngle()
{
    if (DEBUG)
    {
        Serial.print("yawTarget: ");
        Serial.print(yawTarget);
        Serial.print("\tpitchTarget: ");
        Serial.print(pitchTarget);
        Serial.print("\trollTarget: ");
        Serial.print(rollTarget);

        Serial.println();
    }
}

int countJson = 0; //testing

void sendJsonData()
{
    if (countJson % 10 == 0)
    {
        Serial.print("{");
        Serial.print("\"yawTarget\":" + String(yawTarget, 2));
        Serial.print(",\"pitchTarget\":" + String(pitchTarget, 2));
        Serial.print(",\"rollTarget\":" + String(rollTarget, 2));

        Serial.print(",\"motor\":[");
        Serial.print(String(motorFL, 2));
        Serial.print("," + String(motorFR, 2));
        Serial.print("," + String(motorBL, 2));
        Serial.print("," + String(motorBR, 2));
        Serial.print("]");

        Serial.print(",\"receiver\":{");
        Serial.print("\"throttle\":" + String(throttle_ch));
        Serial.print(",\"pitch\":" + String(pitch_ch));
        Serial.print(",\"roll\":" + String(roll_ch));
        Serial.print(",\"yaw\":" + String(yaw_ch));
        Serial.print("}");

        Serial.print(",\"drone_angle\":{");
        Serial.print("\"x\":" + String(droneAngle[0], 2));
        Serial.print(",\"y\":" + String(droneAngle[1], 2));
        Serial.print(",\"z\":" + String(droneAngle[2], 2));
        Serial.print("}");

        // Serial.print(",\"target_angle\":{");
        // Serial.print("\"x\":" + String(yawTarget, 2));
        // Serial.print(",\"y\":" + String(pitchTarget, 2));
        // Serial.print(",\"z\":" + String(rollTarget, 2));
        // Serial.print("}");

        Serial.print(",\"gyro\":{");
        Serial.print("\"x\":" + String(gyro[0], 2));
        Serial.print(",\"y\":" + String(gyro[1], 2));
        Serial.print(",\"z\":" + String(gyro[2], 2));
        Serial.print("}");

        Serial.print(",\"acc\":{");
        Serial.print("\"x\":" + String(acc[0], 2));
        Serial.print(",\"y\":" + String(acc[1], 2));
        Serial.print(",\"z\":" + String(acc[2], 2));
        Serial.print("}");

        Serial.print("}");
        Serial.println();

        countJson = 0;
    }
    countJson++;
}

void readSerial(){
    if (Serial.available() > 0) {
    // read the incoming byte:
        int comingByte = Serial.read();

        // say what you got:
        if(comingByte == 'p'){
            float data = Serial.parseFloat();
            if(data > -000 && data < 100 ){
                p_pitch = data;
                p_roll = data;
                Serial.println("set p_pitch to:" + String(data));
            }
        }
        else if(comingByte == 'd'){
            float data = Serial.parseFloat();
            if(data > -200 && data < 200 ){
                d_pitch = data;
                d_roll = data;
                Serial.println("set d_pitch to:" + String(data));
            }
        }
        else if(comingByte == 'b'){
            float data = Serial.parseFloat();
            if(data > -500 && data < 500 ){
                dd_pitch = data;
                dd_roll = data;
                Serial.println("set dd_pitch to:" + String(data));
            }

        }
        else if(comingByte == 'i'){
            float data = Serial.parseFloat();
            if(data > -100 && data < 100 ){
                i_pitch = data;
                Serial.println("set i_pitch to:" + String(data));
            }
        }
       
    }
}

void init_drone_val()
{

    for (int i = 0; i < 5; i++)
    {
        droneAngle[i] = 0;
    }

    throttle_ch = 1000;
    pitch_ch = 1500;
    roll_ch = 1500;
    yaw_ch = 1500;

    prevYawAngle = 0;
    prevPitchAngle = 0;
    prevRollAngle = 0;

    prevDeltaYaw = 0;
    prevDeltaPitch = 0;
    prevDeltaRoll = 0;
}

void setup()
{
    // put your setup code here, to run once:
    Wire.begin();
    Serial.begin(SERIAL_FREQUENCE);

    //  Serial.println("Initialize drone.");
    yawOutput = 0;
    pitchOutput = 0;
    rollOutput = 0;
    debug_message("Setting up esc.");
    setup_esc();

    debug_message("Setting up MPU925.");

    setup_MPU9250();
    debug_message("Setting up AK8963.");

    setup_AK8963();

    debug_message("Setting up interrup for PWM");
    setup_interrupt();

    init_drone_val();
    debug_message("finish Setup");
}

void loop()
{

    read_gyro_data();
    update_droneAngle();

    calculateMoterOutput();
    // print_droneAngle();
    //  printMotor();
    if (SEND_JSON)
    {
        sendJsonData();
        readSerial();
    }
    send_esc();
    //  read_mgnt_data();
    //  printPWD();
    //  print_mgnt_data();
    //  print_droneAngle();
    //  print_gyro_data();
    // put your main code here, to run repeatedly:
}
