// Linjefølgerbane
// Bane bestående av:
// 1. Slakke/Krappe svinger.
// 2. rettvinklede svinger.
// 3. vei med manglende teip.
// 4. blindvei så bilen må snu. der skal blindveien illustreres med en T form, der rikrig vei er 90* venstre. bilsen skal her klare å snu etter å ha kjørt rett fram, tilbake dit den kom fra.
// (bare rygge?, så ta venstre. registrere at vi kjørte forbi kryss ut på )
// Break-beam(lyskryss) på start/finish linje
// Finne ladestasjonen

// Bibliotek
#include <Wire.h>
#include <Zumo32U4.h>
#include <Arduino.h>
// Hent fra bibliotek
Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4OLED oled;
#define NUM_SENSORS 5 // 5 Linjefølger-sensorer

// linjefølger
unsigned int lineSensorArray[NUM_SENSORS]; // Array for linjesensorverdiene
const uint16_t followSpeed = 250;          // Hastighet for linjefølging PID
int16_t lastError = 0;                     // Variabel for feilmargin
int16_t position;                          // bilens posisjon i forhold til linja
int speedLeft = 0;                         // venstre hjulhastigehet
int speedRight = 0;                        // høyre hjulhastighet
int normalSpeed = 225;                     // basisfart for linjefølging P
float lineMultiplier = 0;                  // tallet hjulene skal ganges med for linjefølging

// Variabler for kryss

bool crossRoad = false;    // Er det kjørt forbi kryss? ja/nei
bool tCrossRoad = false;   //
bool crFinished = false;   // om man er gjennom krysset og ferdig med endringer
int16_t numCrossRoads = 0; // Large antallet kryss kjørt fobi
int16_t numTCrossRoads;    // Lagre antallet T-kryss kjørt forbi
int16_t threshold = 200;   // Hva som regnes som mørk linje

// switch variabler
int switchMode; // casen til switchcase ved kryssbasert kjøring

int sensorZum;               // summen av alle sensorene
unsigned int lastCrossRoads; // tiden ved siste kryss

// printer variabler
unsigned long lastPrintTime; // tid ved sist print
int maxValue;                // høyeste verdi av valgfri ting

void calibrate()
{ // Kalibrere sensorene
  oled.clear();
  oled.gotoXY(0, 0);
  oled.print("calibrating");
  delay(500);
  for (int i = 0; i < 100; i++)
  {
    if (i < 180)
    {
      motors.setSpeeds(200, -200); // kjør i sirkel
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

int highestValue(int value)
{
  if (value > maxValue)
  {
    maxValue = value;
  }
}

int lineSensorZum()
{
  sensorZum = 0;
  for (uint8_t i = 0; i < 5; i++)
  {
    sensorZum += lineSensorArray[i];
  }
  highestValue(sensorZum);
  return sensorZum;
}

void updateSensors()
{ // leser posisjonen til linjefølger
  position = lineSensors.readLine(lineSensorArray);
  lineSensorZum();
}

void printLineSensorReadingsToSerial()
{ // Printer verdiene (0-1000) fra linjesensor til Serial Monitor
  // For hver sensor i sensorarrayet, oppdater og print den kalibrerte sensorverdien
  for (uint8_t i = 0; i < 5; i++)
  {
    Serial.print(lineSensorArray[i]);
    Serial.print(' '); // Mellomrom for lettere lesing
  }
  Serial.println(); // Ny linje for neste lesing
}

void calibratedLineSensorValues()
{ // Leser verdiene fra hver enkelt linjesensor og printer(legg inn enten printLineSensorReadingsToSerial eller printLineSensorReadingsToDisplay)
  // Gir verdier 0-1000, der 1000 er helt sort.
  static uint16_t lastSampleTime = 0;
  // Les hvert 0,1s
  if ((uint16_t)(millis() - lastSampleTime) >= 100)
  {
    lastSampleTime = millis();
    // Les verdien til linjesensorene, kalibrer tallene og gi ut tall mellom 0-1000
    lineSensors.readCalibrated(lineSensorArray);

    // Send resultatene til serial monitor, eller legg inn for display.
    printLineSensorReadingsToSerial();
  }
}

void printLineSensorReadingsToDisplay()
{ // Printer verdiene (0-1000) fra linjesensor til oled display
  oled.setLayout21x8();
  oled.clear();
  oled.gotoXY(0, 1);
  for (uint8_t i = 0; i < 5; i++)
  {
    oled.print(lineSensorArray[i]);
    oled.print(' ');
  }
}

void lineFollowPID()
{
  // leser sensor til linjefølger
  int16_t position = lineSensors.readLine(lineSensorArray);
  int16_t error = position - 2000;
  int16_t speedDifference = error / 4 + 6 * (error - lastError); // Proporsjonal term: error / 4 - Dette er en enkel proporsjonal komponent hvor feilen er delt på 4. Dette betyr at hastighetsforskjellen er proporsjonal med feilen, men skalert ned med 4.
  // Derivativ term: 6 * (error - lastError) - Dette er en derivativ komponent som er proporsjonal med endringen i feil over tid (derivasjon av feilen). Det multipliseres med 6 for å justere vektingen av denne termen.
  int16_t leftSpeed = followSpeed + speedDifference;
  int16_t rightSpeed = followSpeed - speedDifference;
  leftSpeed = constrain(leftSpeed, 0, followSpeed) + 50; // Constraining left and right speed to not be lower than 0 or higher than 400(followSpeed)
  rightSpeed = constrain(rightSpeed, 0, followSpeed) + 50;
  motors.setSpeeds(leftSpeed, rightSpeed); // Setting the speed for the motors
}

void lineFollowP()
{ // følger linje med p regulering
  lineMultiplier = (map(position, 0, 4000, 100.0, -100.0) / 100.0);
  speedLeft = normalSpeed * (1 - lineMultiplier);
  speedRight = normalSpeed * (1 + lineMultiplier);
  motors.setSpeeds(speedLeft, speedRight);
  lineMultiplier = (map(position, 0, 4000, 100.0, -100.0) / 100.0);
}

bool detectGap()
{                          // Sjekk om det dukker opp hull i teiplinja. bruker en "all" fuksjon som kun kicker inn dersom ALLE verdiene havner under treshold
  bool gapDetected = true; // Anntar at det er hull i linja til bevist motsatt
  // Sjekk om noen av sensorene havner under treshold, aka hull i linja
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    if (lineSensorArray[i] >= threshold) // om sensorene leser mer eller lik grensen
    {
      gapDetected = false; // Er det ikke hull, da er første påstand feil.
    }
  }
  return gapDetected; // Returner om det er True/false for om det er hull eller ikke
}

void gapControll()
{ // Sjekker om det har oppstått ett hull i løypa.
  if (detectGap())
  { // Hvis hull, print "Gap..." og stop. evt andre ting om ønskelig
    oled.clear();
    oled.setLayout21x8();
    oled.gotoXY(0, 4);
    oled.print("Gap detected!");
    motors.setSpeeds(0, 0);
  }
  else
  { // Hvis ikke hull, fortsett å følg linja
    oled.clear();
    oled.setLayout21x8();
    oled.gotoXY(0, 4);
    oled.print("You good!");
    lineFollowPID();
  }
}

void countCrossRoads()
{
  if (sensorZum > 2400 && millis() - lastCrossRoads > 2500)
  {
    switchMode = numCrossRoads;
    numCrossRoads++;

    lastCrossRoads = millis();
  }
}

void switchting()
{

  switch (switchMode)
  {
  case 0:
    motors.setLeftSpeed(150);
    motors.setRightSpeed(200);
    if (millis() - lastCrossRoads > 1000)
    { // går ut av case
      switchMode = 99;
    }
    break;

  case 1:
    motors.setLeftSpeed(200);
    motors.setRightSpeed(200);
    if (millis() - lastCrossRoads > 700)
    { // går ut av case
      switchMode = 99;
    }
    break;

  case 2:
    motors.setLeftSpeed(200);
    motors.setRightSpeed(200);
    if (millis() - lastCrossRoads > 700)
    { // går ut av case
      switchMode = 99;
    }
    break;

  case 3:
    motors.setLeftSpeed(150);
    motors.setRightSpeed(0);
    if (millis() - lastCrossRoads > 500)
    { // går ut av case
      switchMode = 99;
    }

  default:
    lineFollowPID();
  }
}

void oledPrinter()
{
  if (millis() - lastPrintTime > 20)
  {

    oled.clear();
    oled.setLayout21x8();
    for (uint8_t i = 0; i < 5; i++) // printer klaibrerte sensorverdier
    {
      oled.print(lineSensorArray[i]);
      oled.print(' ');
    }
    oled.gotoXY(0, 2);
    oled.print(sensorZum);
    oled.gotoXY(0, 3);
    oled.print("maxValue:");
    oled.print(maxValue); // per nå høyeste sensorzum verdi
    oled.gotoXY(0, 4);
    oled.print("crossroads:");
    oled.print(numCrossRoads);

    lastPrintTime = millis();
  }
}

void resetMaxValue()
{
  if (buttonB.getSingleDebouncedPress())
  {
    maxValue = 0;
  }
}

void setup()
{
  Serial.begin(9600);
  lineSensors.initFiveSensors(); // initier de 5 sensorene
  buttonA.waitForButton();       // Vent på knappen før kalibrering
  calibrate();
}

void loop()
{

  updateSensors();
  switchting();

  calibratedLineSensorValues();
  // gapControll();
  oledPrinter();
  resetMaxValue();
  countCrossRoads();
}

/*
bool detectCrossRoad()
{
  kryss = ja / nei
                   antallKryss++ 1
}

void blindvei()
{
  rygg tilbake til der krysset er
          snu 90 *
      venstre
          følg linjen igjen
}

void findChargingStation()
{
    kontinuerlig sjekk for ladestasjonens IR-signatur
    hvis IR-signatur er lest OG batteri er <5%, kjør innom ladestasjon
    velg mellom 6 alternativer
        1. Lad fullt opp. koster mest(200), påvirker battery_health
        2. Lad til 80%, koster mindre(100), påvirker ikke battery_health
        3. Lad for x mengde kroner, vil koste maks samme som full lading(200) ,kan påvire batteriet dersom det blir ladet >80%
        4. Lad hurtig, 10x normal hastighet, kun opp til 50%, koster 2x mer som å lade fullt(400), påvirker battery_health
        5. Battery service, increase battery health with 60, koster 1000
        6. Battery change, sets battery healt to 100, koster 3000
}

void dontCrash()
{
  ? ikke nødvendig
        sjekk om man er nært noe
            hvis litt nært,
      sakk ned
          hvis veeeeldig nært,
      stop, rygg, alternativ rute ? tut om noen kjører forran :)
}

void Taxi()
{
    random kick inn.
    hent passasjer etter random antall kryss
        tut når utenfor
    kjør passasjer til random kryss
    slipp av passasjer
        få penger
        caching lyd :)
} */
