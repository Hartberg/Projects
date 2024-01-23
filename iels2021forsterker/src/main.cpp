#include <Arduino.h>
int potPin = 32;
int VoutPin = 25;
float potReading; // avlesning pot

void readPot(int pin)
{
  potReading = analogRead(pin);
}

void setup()
{
}

void loop()
{
  readPot(potPin);
  Serial.println(potReading);
}
