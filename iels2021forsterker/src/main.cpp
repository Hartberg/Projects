#include <Arduino.h>

int potPin = 32;
int dacPin = 25;
int adcPin = 34;
float potReading; // avlesning pot
float adcReading; 
float amp = 1; // amplifikering

void readPot(int pin)
{
  potReading = analogRead(pin);
}



void setup()
{
  Serial.begin(9600);
}

void loop()
{
  readPot(potPin);
  adcReading = analogRead(adcPin);
  dacWrite(dacPin, adcReading);
}
