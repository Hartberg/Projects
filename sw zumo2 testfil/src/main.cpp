
#include <Arduino.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4Encoders encoders;
Zumo32U4OLED oled;
Zumo32U4LineSensors lineSensors;

// batterivariabler
float battery_level = 100; // batteri prosent

// for linjefølger
unsigned int lineSensorArray[5]; // array til sensorveridiene

int speedLeft = 0;        // venstre hjulhastigehet
int speedRight = 0;       // høyre hjulhastighet
int normalSpeed = 225;    // basisfart for linjefølging
float lineMultiplier = 0; // tallet hjulene skal ganges med for linjefølging
int position = 0;         // linejposijon 0-4000

// tick telling
unsigned long timeNow = 0;        // tiden i perioden
float ticksDuringPeriod = 0;      // antall ticks for en periode. er float for mattematikken den brukes i
float timeElapsed = 0;            // tiden på periode.  er float for mattematikken den brukes i
unsigned long lastTimePeriod = 0; // tiden sist periode var

// printer/oled verdier
unsigned long lastTimePrint = 0;
int displayMode = 1; // caser til displayet

// speedometer
float speed = 0;
unsigned long lastMinuteUpdate = 0;        // timer for siste hele minuteUpdate reset
unsigned long timeElapsedMinuteUpdate = 0; // hvor lenge siden sist minute update loop. aka lengden på denne perioden
unsigned long timeLastMinuteUpdateEnd = 0; // tiden siste minute opdate ble ferdig
float maxSpeedMinute = 0;                  // max hastighet i perioden minuteupdate
int timeOver70 = 0;                        // tiden i ms bilen har kjørt over 70% hastighet
int displayTimeOver70 = 0;                 // displayverdi oppgitt i sekunder
float displayMaxSpeedMinute = 0;           // displayverdi til maxspeed
// speedometer - average
float displayAverageSpeedMinute = 0; // gjennomsnittshastighet i minuteUpdate
long displayTicksDuringMinute = 0; // viser sist periodes totale ticks
long displaytimeElapsedMinuteUpdate = 0; // sist periodes beregning av hastighet
long sampleSizeSpeedometer = 0; // hvor mange loops i perioden
// long display
unsigned long ticksDuringMinuteUpdate = 0;     // antall ticks per minuteUpdate
unsigned long timeElapsedTickMinuteUpdate = 0; // total tid i MinuteUpdate fra ticksensor. bruker ikke LastMinuteUpdate. finn bedre navn

void calibrate()
{ // kalibrere sensorene
  oled.clear();
  oled.gotoXY(0, 0);
  oled.print("calibrating");
  delay(500);
  for (int i = 0; i < 90; i++)
  {
    if (i < 90)
    {
      motors.setSpeeds(200, -200); // kjør i sirkel
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
  oled.clear();
}

void updateSensors()
{ // leser sensor til linjefølger
  position = lineSensors.readLine(lineSensorArray);
}

void drive(int motorSpeedParameter)
{ // kjører begge motor i arg1 hastighet
  motors.setSpeeds(motorSpeedParameter, motorSpeedParameter);
}

void followLineP()
{ // følger linje med p regulering
  lineMultiplier = (map(position, 0, 4000, 100.0, -100.0) / 100.0);
  speedLeft = normalSpeed * (1 - lineMultiplier);
  speedRight = normalSpeed * (1 + lineMultiplier);
  motors.setSpeeds(speedLeft, speedRight);
}

void printValues()
{
  switch (displayMode)
  {
  case 0:
    if (millis() - lastTimePrint > 100) // tidsdelay så skjermen ikke klikker
    {
      oled.clear();
      oled.setLayout8x2();
      oled.gotoXY(0, 0);
      oled.print(speed); // printer ut det inni print. bytt ved behov
      oled.gotoXY(0, 1);
      oled.print(2);            // printer hastighet
      lastTimePrint = millis(); // for tidskjøret
    }
    break;

  case 1:
  if (millis() - lastTimePrint > 100){ // tidsdelay så skjermen ikke klikker
    oled.clear();
    oled.setLayout21x8();
    oled.gotoXY(0,0);
    oled.print("Avg. speed:");
    oled.print(displayAverageSpeedMinute);
    oled.print("cm/s");
    oled.gotoXY(0,2);
    oled.print("Ticks:");
    oled.print(ticksDuringMinuteUpdate);
    oled.gotoXY(0,3);
    oled.print("Time:");
    oled.print(timeElapsedTickMinuteUpdate);
    oled.gotoXY(0,4);
    oled.print("speed");
    oled.print(speed);
    oled.gotoXY(0,5);
    oled.print("ticks sist:");
    oled.print(displayTicksDuringMinute);
    oled.gotoXY(0,6);
    oled.print("tid sist:");
    oled.print(speed);

    lastTimePrint = millis();
  }

    break;
  }
}

void tickSensor()
{
  if (millis() - lastTimePeriod > 15) // får mer nøyaktige verider når decoderen får tellt en del rotasjoner
  {
    int leftEncoder = encoders.getCountsAndResetLeft();
    leftEncoder = abs(leftEncoder); // får absoluttverdi
    int rightEncoder = encoders.getCountsAndResetRight();
    rightEncoder = abs(rightEncoder);
    ticksDuringPeriod = (leftEncoder + rightEncoder) / 2;

    /*
    Serial.print(" leftEncoder ");
    Serial.print(leftEncoder);
    Serial.print(" rightEncoder ");
    Serial.print(rightEncoder);
    */

    timeElapsed = millis() - lastTimePeriod; // tid siden sist periode. aka denne periodens tid
    lastTimePeriod = millis();
  }
}

void speedometer()
{
  speed = (ticksDuringPeriod / timeElapsed) * 13.617;

  if (millis() - timeLastMinuteUpdateEnd > 25) // mer nøyaktige verdier dersom den oppdateres med litt mer mellomrom. er orignalt 4ms perioder
  {
    if (millis() - lastMinuteUpdate < 10000)
    {
      timeElapsedMinuteUpdate = millis() - timeLastMinuteUpdateEnd; // periodens varighet
      if (speed > maxSpeedMinute)
      { // max hastighetoppdatering
        maxSpeedMinute = speed;
      }

      if (speed > 45.5)
      { // tid bilen har kjørt over 7% hastighet denne perioden
        timeOver70 = timeOver70 + timeElapsedMinuteUpdate;
      }

      // average speed
      sampleSizeSpeedometer ++;
      ticksDuringMinuteUpdate += ticksDuringPeriod;
      timeElapsedTickMinuteUpdate += timeElapsed;

      Serial.println(timeElapsedMinuteUpdate);

      timeLastMinuteUpdateEnd = millis();
    }
    else
    {                                                                                           // først lagre verdiene i displayverdiene som brukes i skjermen så reset de
      displayAverageSpeedMinute = (ticksDuringMinuteUpdate / timeElapsedMinuteUpdate ) * 13.617; // displayverdi til monitor
      displayTimeOver70 = timeOver70 / 1000;
      displayMaxSpeedMinute = maxSpeedMinute;
      displayTicksDuringMinute = ticksDuringMinuteUpdate;
      // resett verdier til ny 60s periode
      timeOver70 = 0;
      maxSpeedMinute = 0;
      ticksDuringMinuteUpdate = 0;
      timeElapsedTickMinuteUpdate = 0;
      lastMinuteUpdate = millis();
    }
  }
}
/*
Serial.print(" timeElacpes ");
Serial.print(timeElapsed);
Serial.print(" speed ");
Serial.print(speed);
Serial.print(" ticksDuringPeriod ");
Serial.println(ticksDuringPeriod);
*/

void setup()
{
  Serial.begin(9600);
  lineSensors.initFiveSensors();
  calibrate();
}

void loop()
{
  tickSensor();
  speedometer();
  printValues();
  updateSensors();
  // followLineP();
   drive(400);
}