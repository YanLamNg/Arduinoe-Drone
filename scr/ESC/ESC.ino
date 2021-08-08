#include <Wire.h>
#include <EEPROM.h>

#define ESC_1_PIN 4
#define ESC_2_PIN 5
#define ESC_3_PIN 6
#define ESC_4_PIN 7
#define SGINAL_LED 12

int esc[4];
long escTimer[4];
long escEndTime[4];
long escCycleTime = 20000; //20ms
long prevEscTime;
int loopCount = 0;
bool isIncreasing = true;

void setup() {
  Serial.begin(57600);
  Wire.begin();
  Serial.println("Start ESC");

  pinMode(ESC_1_PIN, OUTPUT);
  pinMode(ESC_2_PIN, OUTPUT);

  pinMode(ESC_3_PIN, OUTPUT);
  pinMode(ESC_4_PIN, OUTPUT);
  pinMode(SGINAL_LED, OUTPUT);

  prevEscTime = micros();
  esc[0] = 1000;
  esc[1] = 1000;
  esc[2] = 1000;
  esc[3] = 1000;

  Serial.println("Waiting 3 seconds");
  delay(3000);
  Serial.println("Start 1000ms pulse ");
  for(int i = 0; i < 500; i++ ){
    PORTD |= B11110000;
    delayMicroseconds(1000);
    PORTD &= B00001111;
    delay(19);
  }
  Serial.println("end 1000ms pulse ");
   
}

void loop() {
  printESC();
  loopCount++;
  if(loopCount > 3){
    loopCount = 0;
    if(isIncreasing){
      esc[0]++;
    }
    else{
      esc[0]--;
    }
    
    if(esc[0]>=1400){
      isIncreasing = false;
    }
    else if(esc[0] <= 1100){
      isIncreasing = true;
    }
  }
  esc[1] = 1000;
  esc[2] =1000;
  esc[3] =1000;
 
  while(prevEscTime + escCycleTime > micros()){
  }
  prevEscTime =micros();
  long timer = prevEscTime;
  PORTD |= B11110000;



  while(PORTD > B00001111){
    long currTime = micros();
    if(timer + esc[0] < currTime){
      PORTD &= B11101111;
    }
    if(timer + esc[2] < currTime){
      PORTD &= B11011111;
    }
    if(timer + esc[3] < currTime){
      PORTD &= B10111111;
    }
    if(timer + esc[4] < currTime){
      PORTD &= B01111111;
    }
  }
  
}



void printESC(){
  Serial.print("ESC_1: ");
  Serial.print(esc[0]);
  Serial.print("\tESC_2: ");
  Serial.print(esc[1]);
  Serial.print("\tESC_3: ");
  Serial.print(esc[2]);
  Serial.print("\tESC_4: ");
  Serial.print(esc[3]);
  Serial.println();
}
