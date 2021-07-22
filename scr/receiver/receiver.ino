#include <Wire.h>
#include <EEPROM.h>

#define ESC_1_PIN 4
#define ESC_2_PIN 5
#define ESC_3_PIN 6
#define ESC_4_PIN 7
#define SGINAL_LED 12

#define THROTTLE_CH ch3_PWM
#define YAW_CH ch4_PWM
#define PITCH_CH ch2_PWM
#define ROLL_CH ch1_PWM

boolean channel_1, channel_2, channel_3, channel_4;
long ch1_timer, ch2_timer, ch3_timer, ch4_timer;
long ch1_PWM, ch2_PWM, ch3_PWM, ch4_PWM;



void setup() {
  Serial.begin(57600);
  Wire.begin();
  Serial.println("start Interrupt");
 // TIMSK0 &= ~_BV(TOIE0); //disable timer 0

  setup_interrupt();
}

void loop() {
  printPWD();
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


void printPWD(){
  Serial.print("thorttle: ");
  Serial.print(THROTTLE_CH);
  Serial.print("\tPitch: ");
  Serial.print(PITCH_CH);
  Serial.print("\tRoll: ");
  Serial.print(ROLL_CH);
  Serial.print("\tYaw: ");
  Serial.print(YAW_CH);
  Serial.println();
}
