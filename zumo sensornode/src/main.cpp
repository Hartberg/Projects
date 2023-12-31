
#include <Arduino.h>
#include <EEPROM.h>
#include <Zumo32U4.h>
#include <IRremote.h>

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;
Zumo32U4OLED oled;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

// IR
const long RECV_PIN = A4; // pin til IR
unsigned long irNum = 0;      // IR signalet innkommet decodet

// batterivariabler
float battery_level = 100;      // batteri prosent
float batteryDrainAvgSpeed = 0; // batteri utladning fra gjennomsnittsfart * avstand

// --oppladningsvariabler
float distanceSincelastReverseCharge = 0; // avstand siden siste reversecharge
float totalBatteryCharge = 0;             // total batteri oppladning i antall prosent
int charging_cycles = 0;                  // antall ladesykluser
bool emergancyCharge = false;             // om nødlading er på eller ikke
unsigned long emergancyChargeTimer = 0;   // når emergancymode blir aktivert
unsigned long cPressed = 0;               // når c ble trykket

// for linjefølger
unsigned int lineSensorArray[5]; // array til sensorveridiene
int speedLeft = 0;               // venstre hjulhastigehet
int speedRight = 0;              // høyre hjulhastighet
int normalSpeed = 225;           // basisfart for linjefølging
float lineMultiplier = 0;        // tallet hjulene skal ganges med for linjefølging
int position = 0;                // linejposijon 0-4000

// tick telling
unsigned long timeNow = 0;        // tiden i perioden
float absTicksDuringPeriod = 0;   // absoluttverdi av antall ticks for en periode. er float for mattematikken den brukes i. avstand uabhengig av retning
float ticksDuringPeriod = 0;      // antall ticks i
float timeElapsed = 0;            // tiden på periode.  er float for mattematikken den brukes i
unsigned long lastTimePeriod = 0; // tiden sist periode var

// printer/oled verdier
unsigned long lastTimePrint = 0; // sist displaey ble oppdatert: for tidsdelay
int displayMode = 1;             // caser til displayet

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

// bank variabler
float bankBalance = 0.0; // Startsaldo for bankkonto

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

void ReadIR()
{
  if (IrReceiver.decode())
  {
    irNum = IrReceiver.decodedIRData.decodedRawData;
    Serial.println(irNum);
  }
  IrReceiver.resume();
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
  if (buttonA.getSingleDebouncedPress()) // endrer displaymodus ved å trykke på A
  {
    displayMode++;
    if (displayMode == 3)
    { // reset ved siste displaymodus
      displayMode = 0;
    }
  }
  switch (displayMode)
  {
  case 0:
    if (millis() - lastTimePrint > 100) // tidsdelay så skjermen ikke klikker
    {
      oled.clear();
      oled.setLayout21x8();
      oled.gotoXY(0, 0);
      oled.print(speed); // printer ut det inni print. bytt ved behov
      oled.gotoXY(0, 1);
      oled.print(3); // printer hastighet
      oled.gotoXY(0, 3);
      oled.print(irNum, HEX);        // printer ir ting
      lastTimePrint = millis(); // for tidskjøret
    }
    break;

  case 1:
    if (millis() - lastTimePrint > 125)
    { // tidsdelay så skjermen ikke klikker
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

      lastTimePrint = millis();
    }
    break;

  case 2:
    if (millis() - lastTimePrint > 125)
    {
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

      lastTimePrint = millis();
    }
  }
}

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

void batteryLowWarning(int batteryHealth) // skrur på lys når battery_level er under arg.1
{
  if (battery_level < batteryHealth)
  {
    ledRed(HIGH);
  }
}

void nrChargecycles() // oppdaterer ladesykluser
{
  if (totalBatteryCharge > 100 + charging_cycles * 100)
  {
    charging_cycles++;
  }
}

void batteryChargeDrain(bool emergancy, bool allowReverseCharge) // lader opp batteriet når den rygger. NB!: om vi skal utlade tregere må vi sette på et tidsdelay da opp/utladningsverdien nå er 0.01 som er minste verdi mulig
{

  batteryLowWarning(30);
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
  { // passer på at batteri nivået ikke blir mer enn 100%.
    battery_level = 100;
  }
  else if (battery_level < 0)
  {
    battery_level = 0;
  }
  nrChargecycles();
}

void emergancyCheck()
{
  if (buttonC.getSingleDebouncedPress()) // tar tiden på når knapp blir trykket
  {
    cPressed = millis();
  }

  if (buttonC.getSingleDebouncedRelease()) // // sjekker om knappen er trykket mer enn 2 sekunder
  {
    if (millis() - cPressed > 2000)
    {
      emergancyCharge = true;
      emergancyChargeTimer = millis();
    }
  }

  if (emergancyCharge == true && millis() - emergancyChargeTimer > 20000) // skrur av funksjonen etter
  {
    emergancyCharge = false;
  }

  if (emergancyCharge == true) // bare en indikator for om emergancy mode er på kan/burde endres f.eks noe beep
  {
    ledYellow(true);
  }
  else
  {
    ledYellow(false);
  }
}

void driveModeButton()
{ // knapp for kjøremodus
  if (buttonB.getSingleDebouncedPress())
  {
    driveMode++;
    if (driveMode == 4)
    {
      driveMode = 0;
    }
  }
}

void driveModeBased()
{ // case med kjør frem tilbake og stå stille ved trykk på knapp B
  switch (driveMode)
  {
  case 0:
    drive(250);
    break;

  case 1:
    drive(0);
    break;

  case 2:
    drive(-200);
    break;

  case 3:
    drive(0);
    break;
  }
}

void deposit(float amount)
{ //
  // Etter en jobb er blitt gjort
  bankBalance += amount; // Legg til mengden "amount" som skal legges til i balance
}
/*
// Simulere uttak
void withdraw(){
  // Når en besøker ladestasjon
  // Sjekker først om man har nok penger på konto
  if (amount <= balance){
    balance -= amount; // trekker fra amount penkger som trekkes fra balance
  } else { // Dersom man ikke har nok penger
  oled.clear();
  oled.setLayout21x8();
    oled.gotoXY(0,2);
    oled.print("U too broke bitch!");
    oled.gotoXY(0,4);
    oled.print("Current balance: ");
    oled.gotoXY(0,6);
    oled.print(balance);
  }
}
// Ladestasjon ikke ferdi!
void charging(float& battery, float chargeRate) {
  // Øker batterinivået basert på ladetakt
battery += chargeRate;
}
*/

// disse er per 18/11 19:49 ikke implementert inn i koden mer enn at funksjonene er skrevet inn. de blir ikke brukt
void levels()
{
  oled.print(F("Lev"));
  oled.println(EEPROM.read(0));
  delay(1000);
  oled.clear();
  oled.print(F("Hels"));
  oled.println(EEPROM.read(1));
  delay(1000);
  oled.clear();
}

void ProductionFualt()
{
  int BatHelseRand;
  int RandomHealthOne = random(0, 1000);
  int RandomHealthTwo = random(0, 1000);
  int RandomHealthThree = random(0, 1000);
  if (RandomHealthOne == RandomHealthTwo && RandomHealthOne == RandomHealthThree)
  {
    int BatHelseRand = EEPROM.read(1);
    int BatHelseNow = BatHelseRand / 2;
    EEPROM.write(1, BatHelseNow);
    oled.clear();
    oled.print(BatHelseNow);
  }
}

void writeUnsignedIntIntoEEPROM(int address, unsigned int number)
{
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}

unsigned int readUnsignedIntFromEEPROM(int address)
{
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}

// frem til disse

void setup()
{
  Serial.begin(9600);
  lineSensors.initFiveSensors();
  proxSensors.initFrontSensor();
  calibrate();
  EEPROM.write(0, 100);
  EEPROM.write(1, 100);
  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK); // for IR Recivier
  
}

void loop()
{
  tickSensor();
  speedometer();
  batteryChargeDrain(emergancyCharge, true); // 1. arg nødmus 2. arg om opplading lov
  printValues();
  updateSensors();
  ReadIR();
  followLineP();
  //driveModeButton(); // bruker b for å endre mellom kjør og ikke
  //emergancyCheck(); // samme som under
  //driveModeBased(); // burde nok puttes inn i en annen funksjon
}
