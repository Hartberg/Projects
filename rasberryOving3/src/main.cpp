#include "arduino.h"
#include "HardwareSerial.h"

//HardwareSerial SerialPort(0);
int ledPin = 35;
String recievedData;  

void setup()
{
  //Serial.begin(9600);
  Serial.begin(9600,SERIAL_8N1,1,3);
  delay(100);

  
  pinMode(ledPin, OUTPUT);
}

const String on = "On\n";

void loop()
{
/*
  if (SerialPort.available())
  {
    recievedData = SerialPort.readStringUntil('\n'); 
    if (recievedData == on){
      Serial.print("LIGHT ON");
      digitalWrite(ledPin, HIGH);
    }
    else if (strcmp(recievedData.c_str(), "Off\n")){
      digitalWrite(ledPin, LOW);
      Serial.print("LIGHTS OFF");
    }
    //Serial.print(recievedData);
    Serial.println(recievedData.length());
    Serial.println(strlen("Off\n"));
    Serial.println(strlen(recievedData.c_str()));
  }*/
  if (Serial.available()){

    char number = Serial.read();
    Serial.print(number);
  }


}