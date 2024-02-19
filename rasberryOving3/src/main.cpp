#include <Arduino.h>
#include <HardwareSerial.h>

#define TX_PIN 17
#define RX_PIN 16

const int LED = 32;
unsigned long lastPrint = 0;

HardwareSerial SerialPort(2); // use UART2

void setup()
{
  Serial.begin(9600);
  SerialPort.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Fixed order of RX and TX pins
  pinMode(LED, OUTPUT);
}

void loop()
{
  //Serial.println("readyToReceive\n");
  //SerialPort.println("readyToReceive\n");
  if (SerialPort.available() > 0)
  {
    Serial.println("readyToReceive");
    String ledRec = SerialPort.readString();

    // SI IFRA AT JEG ER FERDIG
    Serial.println(ledRec);
    if (ledRec == "Off")
    {
      digitalWrite(LED, LOW);
    }
    if (ledRec == "On")
    {
      digitalWrite(LED, HIGH);
    }
  }
}
