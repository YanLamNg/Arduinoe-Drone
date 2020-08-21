#include <EEPROM.h>
#include <Wire.h>

#define SERIAL_FREQUENCE 19200
#define pinBlink 8
double last_timer = millis();

void setup() {
  Serial.begin(SERIAL_FREQUENCE);
  Serial.println("set");

  // set pin to input with a pullup, led to output
  pinMode(pinBlink, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  PCICR |= (1 << PCIE0);                                       //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);
  // Manually blink once to test if LED is functional
 
  Serial.println("set done");

  // Attach the new PinChangeInterrupt and enable event function below

}

ISR(PCINT0_vect) {
  // Switch Led state
  if(digitalRead(pinBlink) == HIGH){

    last_timer = micros();
  }
  else{
    int val = millis() - last_timer;
    Serial.println( val );
  }

}


void loop() {

}
