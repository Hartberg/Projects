// EEPROM FUNKER IKKE
#include <Arduino.h>
#include <EEPROM.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;
Zumo32U4OLED oled;
Zumo32U4LineSensors lineSensors;

// batterivariabler
float battery_level;                // batteri prosent
float batteryDrainAvgSpeed = 0;     // batteri utladning fra gjennomsnittsfart * avstand
int battery_health;                 // batterihelse 0-100 lagres i eeprom
bool currentlyUnder5 = false;       // en state detection for om batteriet er under 5%
int timesUnder5 = 0;                // antall ganger bilen utladet under 5%
unsigned long batterySuperLowTimer; // timer til warningsystem

// --oppladningsvariabler
float distanceSincelastReverseCharge = 0; // avstand siden siste reversecharge
float totalBatteryCharge = 0;             // total batteri oppladning i antall prosent
int charging_cycles = 0;                  // antall ladesykluser
bool emergancyCharge = false;             // om nødlading er på eller ikke
unsigned long emergancyChargeTimer = 0;   // når emergancymode blir aktivert
unsigned long cPressedTime = 0;           // når c ble trykket

// for linjefølger
unsigned int lineSensorArray[5]; // array til sensorveridiene
int speedLeft = 0;               // venstre hjulhastigehet
int speedRight = 0;              // høyre hjulhastighet
int normalSpeed = 225;           // basisfart for linjefølging
float lineMultiplier = 0;        // tallet hjulene skal ganges med for linjefølging
int16_t position = 0;            // linejposijon 0-4000

// tick telling
unsigned long timeNow = 0;        // tiden i perioden
float absTicksDuringPeriod = 0;   // absoluttverdi av antall ticks for en periode. er float for mattematikken den brukes i. avstand uabhengig av retning
float ticksDuringPeriod = 0;      // antall ticks i
float timeElapsed = 0;            // tiden på periode.  er float for mattematikken den brukes i
unsigned long lastTimePeriod = 0; // tiden sist periode var

// EEPROM adresser
uint8_t eeBatteryLevel = 0;
uint8_t eeBatteryHealth = 1;
uint8_t eeFundAmount = 2;
uint8_t eeChargeCyclesCount = 3;
uint8_t eeLowBatteryOccurences = 4;

// printer/oled verdier
unsigned long lastTimePrint = 0; // sist displaey ble oppdatert: for tidsdelay
int screenNR = 0;                // caser til displayet

// speedometer
float speed = 0;                           // utregnet absolutt momentanfart
float signedSpeed = 0;                     // momentanfart med fortegn
unsigned long totalDistance = 0;           // total distanse kjørt i meter
unsigned long lastMinuteUpdate = 0;        // timer for siste hele minuteUpdate reset
unsigned long timeElapsedMinuteUpdate = 0; // hvor lenge siden sist minute update loop. aka lengden på denne perioden
unsigned long timeLastMinuteUpdateEnd = 0; // tiden siste minute opdate ble ferdig
bool periodFinished = LOW;                 // et flag for om en minuttperiode er fullført
float maxSpeedMinute = 0;                  // max hastighet i perioden minuteupdate
int timeOver70 = 0;                        // tiden i ms bilen har kjørt over 70% hastighet
int displayTimeOver70 = 0;                 // displayverdi oppgitt i sekunder
float displayMaxSpeedMinute = 0;           // displayverdi til maxspeed

// speedometer - average
float displayAverageSpeedMinute = 0;       // gjennomsnittshastighet i minuteUpdate
long displayTicksDuringMinute = 0;         // viser sist periodes totale ticks
long displaytimeElapsedMinuteUpdate = 0;   // sist periodes beregning av hastighet
float sampleSizeSpeedometer = 0;           // hvor mange loops i perioden
unsigned long speedSum = 0;                // summen av speed i en periode. deles på samplessize for gjennomsnittshastighet
long timeMotorsOnThisPeriod = 0;           // tiden i en periode motorene har vært på
unsigned long ticksDuringMinuteUpdate = 0; // antall ticks per minuteUpdate

// drive variabler
int driveMode = 0; // kjøremodus

// bank
int account_balance = 250; // bankkonto

// knappevariabler
bool aPressed = false;
bool bPressed = false;
bool cPressed = false;
unsigned long aTimePressed; // hvor lenge c ble holdt nede
unsigned long bTimePressed; // hvor lenge c ble holdt nede
unsigned long cTimePressed; // hvor lenge c ble holdt nede
int aHeldDown = 0;
int bHeldDown = 0;
int cHeldDown = 0;

// kalibrerrrer linjesensor
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

// leser av sensorverdiene til hjulene
void tickSensor()
{
  if (millis() - lastTimePeriod > 15) // får mer nøyaktige verider når decoderen får tellt en del rotasjoner
  {
    int leftEncoder = encoders.getCountsAndResetLeft();
    int rightEncoder = encoders.getCountsAndResetRight();

    ticksDuringPeriod = (leftEncoder + rightEncoder) / 2; // avstand med fortegn

    rightEncoder = abs(rightEncoder);
    leftEncoder = abs(leftEncoder); // får absoluttverdi
    absTicksDuringPeriod = (leftEncoder + rightEncoder) / 2;

    timeElapsed = millis() - lastTimePeriod; // tid siden sist periode. aka denne periodens tid
    lastTimePeriod = millis();
  }
}


void readButtons()
{
  if (buttonA.getSingleDebouncedPress())
  { // sjekker om knapp a er nede
    aPressed = true;
    aTimePressed = millis();
  }

  if (buttonA.getSingleDebouncedRelease())
  {
    aHeldDown = millis() - aTimePressed; // hvor lenge kanppen ble holdt nede
  }

  if (buttonB.getSingleDebouncedPress())
  { 
    bPressed = true;
    bTimePressed = millis();
  }
  if (buttonB.getSingleDebouncedRelease())
  {
    bHeldDown = millis() - bTimePressed; 
  }

  if (buttonC.getSingleDebouncedPress())
  { 
    cPressedTime = millis();
    cPressed = true;
  }
  if (buttonC.getSingleDebouncedRelease())
  {
    cHeldDown = millis() - cPressedTime; 
  }
}

void printToOled()
{
  if (aPressed == true) // rullerer skjermer ved knappetrykk
  {
    screenNR++;
    if (screenNR > 3)
    { 
      screenNR = 0;
    }
  }
  if (millis() - lastTimePrint > 100) // oppdater hver 100ms for smoothere skjerm
  {
    switch (screenNR)
    {
    case 0:
      oled.clear();
      oled.setLayout21x8();
      oled.gotoXY(0, 0);
      oled.print(aHeldDown);
      oled.println("A");
      break;

    case 1:
      oled.clear();
      oled.setLayout21x8();
      oled.gotoXY(0, 0);
      oled.print("Avg. speed:");
      oled.print(displayAverageSpeedMinute);
      oled.print("cm/s");
      oled.gotoXY(0, 2);
      oled.print("Ticks:");
      oled.print(ticksDuringMinuteUpdate);
      oled.gotoXY(0, 3);
      oled.print("Time:");
      oled.print(millis() - lastMinuteUpdate);
      oled.gotoXY(0, 4);
      oled.print("speed");
      oled.print(speed);
      oled.gotoXY(0, 5);
      oled.print("ticks sist:");
      oled.print(displayTicksDuringMinute);
      oled.gotoXY(0, 6);
      oled.print("tid sist:");
      oled.print(displaytimeElapsedMinuteUpdate);
      oled.gotoXY(0, 7);
      oled.print("max speed:");
      oled.print(displayMaxSpeedMinute);
      oled.print("cm/s");
      oled.gotoXY(0, 1);
      oled.print("over 70:");
      oled.print(displayTimeOver70);
      oled.print("s");
      break;

    case 2:
      oled.clear();
      oled.setLayout21x8();
      oled.gotoXY(0, 0);
      oled.print("battery drain");
      oled.print(batteryDrainAvgSpeed);
      oled.gotoXY(0, 1);
      oled.print("distance");
      oled.print(totalDistance);
      oled.gotoXY(0, 2);
      oled.print("motors on:");
      oled.print(timeMotorsOnThisPeriod);
      oled.gotoXY(0, 3);
      oled.print("clock:");
      oled.print(millis() - lastMinuteUpdate);
      oled.gotoXY(0, 4);
      oled.print("battery:");
      oled.print(battery_level);
      oled.gotoXY(0, 5);
      oled.print("unsigned speed:");
      oled.print(signedSpeed);
      oled.gotoXY(0, 6);
      oled.print("total lad:");
      oled.print(totalBatteryCharge);
      oled.gotoXY(0, 7);
      oled.print("charce cycle:");
      oled.print(charging_cycles);

      break;
    case 90: // batteriservice
      oled.setLayout21x8();
      oled.gotoXY(0, 0);
      oled.print("battery service");
      oled.gotoXY(0, 1);
      oled.print("health now:");
      oled.gotoXY(0, 2);
      oled.print(battery_health);

    default:
      break;
    }
    lastTimePrint = millis();
  }
}

// resetter verdier og klargjør for ny loop
void resetCleanup()
{
  
  aPressed = false;
  aHeldDown = 0;
  bPressed = false;
  bHeldDown = 0;
  cPressed = false; 
  cHeldDown = 0;
}

void readSensors()
{
  tickSensor();
  position = lineSensors.readLine(lineSensorArray); // les linjesensor
  readButtons();
}

void speedometer()
{
  speed = (absTicksDuringPeriod / timeElapsed) * 13.617;    // momentanfart
  signedSpeed = (ticksDuringPeriod / timeElapsed) * 13.617; // hastighet med fortegn

  if (millis() - timeLastMinuteUpdateEnd > 15) // mer nøyaktige verdier dersom den oppdateres med litt mer mellomrom. er orignalt 4ms perioder
  {
    if (timeMotorsOnThisPeriod < 8000)
    {
      timeElapsedMinuteUpdate = millis() - timeLastMinuteUpdateEnd; // periodens varighet

      if (speed > 0) // tiden motoren har kjørt denne perioden
      {
        timeMotorsOnThisPeriod += timeElapsedMinuteUpdate;

        if (speed > maxSpeedMinute)
        { // max hastighetoppdatering
          maxSpeedMinute = speed;
        }

        if (speed > 45.5)
        { // tid bilen har kjørt over 70% hastighet denne perioden
          timeOver70 += timeElapsedMinuteUpdate;
        }

        // average speed
        sampleSizeSpeedometer++;
        speedSum += speed;
        ticksDuringMinuteUpdate += absTicksDuringPeriod; // antall ticks i en periode

        // total avstand i cm
        totalDistance += absTicksDuringPeriod / 13.617;
      }

      timeLastMinuteUpdateEnd = millis();
    }

    else
    { // først lagre verdiene i displayverdiene som brukes i skjermen så reset de
      displayAverageSpeedMinute = speedSum / sampleSizeSpeedometer;
      displayTimeOver70 = timeOver70 / 1000;
      displayMaxSpeedMinute = maxSpeedMinute;
      displayTicksDuringMinute = ticksDuringMinuteUpdate;
      // resett verdier til ny 60s periode
      timeOver70 = 0;
      maxSpeedMinute = 0;
      ticksDuringMinuteUpdate = 0;
      speedSum = 0;
      sampleSizeSpeedometer = 0;
      timeMotorsOnThisPeriod = 0;
      periodFinished = HIGH; // markerer at en runde er fullført
      lastMinuteUpdate = millis();
    }
  }
}
// 1. arg bool emergancy. 2.arg allow charging
void batteryChargeDrain(bool emergancy, bool allowReverseCharge) // lader opp batteriet når den rygger. NB!: om vi skal utlade tregere må vi sette på et tidsdelay da opp/utladningsverdien nå er 0.01 som er minste verdi mulig
{
  float batteryCharge = 0;                            // batterioppladningsmengden
  if ((battery_level >= 0) && (battery_level <= 100)) // legg til && battery < 100 så den ikke lader over 100%
  {
    distanceSincelastReverseCharge = totalDistance - distanceSincelastReverseCharge;

    if (allowReverseCharge) // tillater lading og utlading
    {
      batteryCharge = (-1 * signedSpeed * distanceSincelastReverseCharge) / 10000000;
    }
    else if (signedSpeed > 0) // tilater kun utlading (nomralstatus)
    {
      batteryCharge = (-1 * speed * distanceSincelastReverseCharge) / 10000000;
    }

    if (emergancy == true && battery_level < 20 && batteryCharge < 0) // nødlading
    {
      batteryCharge = batteryCharge * 10;
    }

    battery_level += batteryCharge;               // ladning vi utfører denne perioden
    if (batteryCharge > 0 && battery_level < 100) // legg till totale lademengde
    {
      totalBatteryCharge += batteryCharge; // total mengde ladning
    }
  }
  if (battery_level > 100)
  { // passer på at batteri nivået ikke blir mer enn 100% og mindre enn 0%
    battery_level = 100;
  }
  else if (battery_level < 0)
  {
    battery_level = 0;
  }
}

void nrChargecycles() // oppdaterer ladesykluser
{
  if (totalBatteryCharge > 100 + charging_cycles * 100)
  {
    charging_cycles++;
  }
}

// antall ganger bilen er utladet under 5%
void chargedUnder5()
{
  if (currentlyUnder5 == false && battery_level < 5)
  {
    currentlyUnder5 = true;
    timesUnder5++;
  }
  if (battery_level > 5)
  { // state change detection
    currentlyUnder5 = false;
  }
}

void batteryLowWarning(int batteryHealth) // skrur på lys når battery_level er under arg.1
{
  if (battery_level < batteryHealth)
  {
    ledRed(HIGH);
  }
}

// ved 2sek trykk på c blir emergancyCharge= true
void emergancyCheck()
{
  if (cHeldDown > 2000)
  {
    emergancyCharge = true;
  }
  if (emergancyCharge == true && millis() - emergancyChargeTimer > 20000) // skrur av funksjonen etter 2 s
  {
    emergancyCharge = false;
  }

  if (emergancyCharge == true) // indikerer om emergancymode = true
  {
    ledYellow(true);
  }
  else
  {
    ledYellow(false);
  }
}



void batterySuperLow()
{
  if (millis() - batterySuperLowTimer > 15000)
  {
    motors.setSpeeds(0, 0);
    buzzer.playNote(NOTE_A(4), 100, 10);
    buzzer.playNote(NOTE_A(4), 100, 10);
    batterySuperLowTimer = millis();
  }
}

// kjører begge motor i arg1 hastighet
void drive(int motorSpeedParameter)
{
  motors.setSpeeds(motorSpeedParameter, motorSpeedParameter);
}

// sjekker om knapp b blir trykket og endrer kjøremodus
void updateDriveMode()
{
  if (bPressed == true)
  {
    driveMode++;

    if (driveMode == 4)
    {
      driveMode = 0;
    }
  }
}

// case med kjør frem tilbake og stå stille ved trykk på knapp B
void driveModeBased()
{
  switch (driveMode)
  {
  case 0:
    drive(0);
    break;

  case 1:
    drive(200);
    break;

  case 2:
    drive(0);
    break;

  case 3:
    drive(-200);
    break;
  }
}

// simulerer prroduksjonsfeil ved batteripakke
void ProductionFualt()
{
  int RandomHealthOne = random(0, 1000);
  int RandomHealthTwo = random(0, 1000);
  int RandomHealthThree = random(0, 1000);
  if (RandomHealthOne == RandomHealthTwo && RandomHealthOne == RandomHealthThree)
  {
    battery_health = battery_health / 2;
  }
}

void GetUpstartValuesFromEeprom()
{
  EEPROM.get(eeBatteryLevel, battery_level); // må bruke get for float
  battery_health = EEPROM.read(eeBatteryHealth);
  charging_cycles = EEPROM.read(charging_cycles);
  account_balance = EEPROM.read(eeFundAmount);
}

// batteribytte og service
void batteryHealthService(int repair)
{
  battery_health += repair;
  if (battery_health > 100)
  {
    battery_health = 100;
  }
  screenNR = 90; // endrer til batterihelsebytte skejrm
}

void batteryHealthUpdate()
{
  ProductionFualt();
  nrChargecycles();
  chargedUnder5();
  if (cHeldDown > 5000)
  {
    batteryHealthService(40);
  }
  else if (cHeldDown > 10000)
  {
    batteryHealthService(100);
  }
}

// oppdaterer eepromen med riktig informasjon
void updateEeprom()
{
  EEPROM.put(eeBatteryLevel, battery_level);
  EEPROM.update(eeBatteryHealth, battery_health);
  EEPROM.update(eeChargeCyclesCount, charging_cycles);
  EEPROM.update(eeFundAmount, account_balance);
}

// samlefunksjon for alarmer/advarsel
void warnings()
{
  batteryLowWarning(30);
  if (battery_level < 5){
    batterySuperLow();
  }
}

void setup()
{
  lineSensors.initFiveSensors(); // initeier linjeSensorer
  // calibrate();                   // kalibrer linjesensorer
  GetUpstartValuesFromEeprom();
  Serial.begin(9600);
}

void loop()
{

  readSensors(); // linje / ticks / knapper / opplading / batteribytte

  // --singalprosessering
  speedometer();                  // omgjør inngangsverdiene til brukbare
  batteryChargeDrain(true, true); // opp og utladning
  batteryHealthUpdate();
  emergancyCheck();
  updateDriveMode();

  warnings();
  // --utganger

  driveModeBased(); // kjørefunksjon
  printToOled();
  updateEeprom();

  // reset
  resetCleanup();
}