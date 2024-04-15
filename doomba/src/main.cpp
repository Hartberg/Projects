#include <Arduino.h>
int relepin = 2;


void taze(){
  if(random(0, 10000) == 1){ 
    digitalWrite(relepin, HIGH);
    delay(500);
    digitalWrite(relepin, LOW);
  }
}

void setup() {
  pinMode(relepin, OUTPUT);
}

void loop() {
  taze();
}

