#include <Arduino.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4Encoders encoders;
Zumo32U4OLED oled;
Zumo32U4LineSensors lineSensors;

// batterivariabler
float battery_level = 100;               // batteri prosent
int tick_this_period = 0;                // tikk i en periode
unsigned long int total_ticks = 0;       // total antall ticks, unsigned long for større størrelse
unsigned long int last_check_period = 0; // når sist periode ble tellt
float avrage_speed_this_period = 0;      // gjennomnsnittsv

// speedometer
float speed = 0;

// for linjefølger
#define NUM_SENSORS 5
unsigned int lineSensorArray[5]; // array til sensorveridiene

int speedLeft = 0;        // venstre hjulhastigehet
int speedRight = 0;       // høyre hjulhastighet
int normalSpeed = 225;    // basisfart for linjefølging
float lineMultiplier = 0; // tallet hjulene skal ganges med for linjefølging
int position = 0;         // linejposijon 0-4000

// tickrate konstanter
uint16_t tickDuringPeriod; // antall tick per tidsperiode mellom ticksensor kjører
uint16_t totalLenghtDriven;
uint32_t totalTicks;
unsigned long timeElapsed;
unsigned long lastTime;

// lenge speedometer
uint16_t LenghtCentimeter;

void calibrate()
{ // kalibrere sensorene
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
}

void updateSensors()
{ // leser sensor til linjefølger
  position = lineSensors.readLine(lineSensorArray);
}

void followLineP()
{ // følger linje med p regulering
  lineMultiplier = (map(position, 0, 4000, 100.0, -100.0) / 100.0);
  speedLeft =  normalSpeed * (1 - lineMultiplier);
  speedRight = normalSpeed * (1 + lineMultiplier);
  motors.setSpeeds(speedLeft, speedRight);
}

void tickSensor()
{
  
  if ((millis() % 100) == 0)
  
  {
    unsigned long timeNow = millis();
    int leftEncoder = encoders.getCountsAndResetLeft();
    leftEncoder = abs(leftEncoder);
    int rightEncoder = encoders.getCountsAndResetRight();
    rightEncoder = abs(rightEncoder);
    uint16_t LeftPlussRight = leftEncoder + rightEncoder;
    tickDuringPeriod = (leftEncoder + rightEncoder) / 2;
    Serial.print(" tickDuringPeriod ");
    Serial.print(tickDuringPeriod);
    Serial.print(" leftEncoder ");
    Serial.print(leftEncoder);
    Serial.print(" rightEncoder ");
    Serial.print(rightEncoder);

    timeElapsed = timeNow - lastTime;
    Serial.print(" timeElacpes ");
    Serial.println(timeElapsed);
    lastTime = millis();
  }
}

void TotalLenght()
{
  totalTicks += tickDuringPeriod;
  LenghtCentimeter = totalTicks / 74;
}
/*
void utladningseffekt()
{
  utlandingseffekt =  (avraagespeedthisperiod *  antallticks)* en konstant for å få fornuftig verdi
}
*/
void printValues()
{
  oled.clear();
  oled.gotoXY(0, 0);
  oled.print(tickDuringPeriod); // printer ut det inni print. bytt ved behov
  oled.gotoXY(0, 1);
  oled.print(LenghtCentimeter); // printer hastighet
}

// Hastighet m/s
void speedometer()
{
  /*
   HjulHastighetVenstre = (AntallImpulserVenstre/ImpulserPerOmdreiningVenstre)* OmkretsHjul/ tid
   HjulHastighetHøyre = (AntallImpulserHøyre/ImpulserPerOmdreiningHøyre)* OmkretsHjul/ tid
   */
  speed = (tickDuringPeriod / timeElapsed) * 13.617;
}

void setup()
{
  Serial.begin(9600);
  lineSensors.initFiveSensors();
  calibrate();
}

void loop()
{
  tickSensor();
  TotalLenght();
  updateSensors();
  followLineP();
  speedometer();
  printValues();
}