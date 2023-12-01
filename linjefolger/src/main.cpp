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

// gap variabler
unsigned long timeSinceGapDetect = 0; // tiden gap ble oppdaget
bool onlyGapDetecedWithin10s = false; // et flagg for om gap har vært oppdaget siden ett sekund siden

// switch variabler
int switchMode;                   // casen til switchcase ved kryssbasert kjøring
unsigned long lastTurnAction = 0; // brukes som "klokke" i kryssene. noterer sist arbeid
bool notInOtherCase;              // sjekker om man er i en case

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

int highestValue(int value) // lagrer høyeste verdi av det du putter i 1. arg
{
  if (value > maxValue)
  {
    maxValue = value;
  }
}

int lineSensorZum() // gir ut summen av alle sensorene. lettere å jobb med enn 5 enkelt sensor
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

bool gapIsDetected()
{                          // Sjekk om det dukker opp hull i teiplinja. bruker en "all" fuksjon som kun kicker inn dersom ALLE verdiene havner under treshold
  bool gapDetected = true; // Anntar at det er hull i linja til bevist motsatt
  // Iterer gjennom alle 5 sensorene
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    if (lineSensorArray[i] >= threshold) // Dersom en av sensorene leser mer eller lik grensen
    {
      gapDetected = false; // Er det ikke hull, da er bilen på linja og første påstand feil.
    }
  }
  return gapDetected; // Returner om det er True/false for om det er hull eller ikke
}

// Se bort ifra midlertidig
void gapControll()
{ // Sjekker om det har oppstått ett hull i løypa.
  if (gapIsDetected())
  { // Hvis hull, switch case 4 (kjør over)
    switchMode = 4;
  }
}
void countCrossRoads() // teller om man er nådd ett kryss. tidsdelay så den ikke registrerer ett kryss flere ganger
{
  if (sensorZum > 2400 && millis() - lastCrossRoads > 2500)
  {
    switchMode = numCrossRoads; // setter hvilken sving det er til switch casen
    numCrossRoads++;            // setter neste kryss

    lastCrossRoads = millis(); // noterer når sist kryss var
  }

  if (gapIsDetected())
  {
    switchMode = 4; // vi kjører rett frem når det ikke oppdages noe strek
    if (onlyGapDetecedWithin10s == false)
    {
      timeSinceGapDetect = millis();
      onlyGapDetecedWithin10s = true;
    }

    if (millis() - timeSinceGapDetect > 200)
    { // endre gap bool etter 10s
      numCrossRoads++;
    }
  }
  else
  {
    onlyGapDetecedWithin10s = false;
    switchMode = 99;
  }
}

void switchting() // kjøremodus for sving basert kjøring. kjører pid til vanlig pg går inn i case når den møter kryss.
{

  switch (switchMode)
  {
  case 0: // første kryss
    notInOtherCase= true;
    motors.setLeftSpeed(150);
    motors.setRightSpeed(200);
    if (millis() - lastCrossRoads > 1000)
    { // går ut av case
      switchMode = 99;

    }
    break;

  case 1: // andre kryss
  notInOtherCase = true;
    motors.setLeftSpeed(200);
    motors.setRightSpeed(200);
    if (millis() - lastCrossRoads > 700)
    { // går ut av case
      switchMode = 99;
    }
    break;

  case 2: // tredje kryss
  notInOtherCase = true;
    motors.setLeftSpeed(200);
    motors.setRightSpeed(200);
    if (millis() - lastCrossRoads > 700) // gå tilbake til default(PID)
    {                                    // går ut av case
      switchMode = 99;                   // setter den til default
    }
    break;

  case 3: // Blindvei

    notInOtherCase = true;
    if (millis() - lastCrossRoads < 50) // skrur av bilen i 50ms for å spare motor. vil ikke gå fra positiv til negativ fart
    {
      motors.setSpeeds(0, 0);
    }
    else
    {
      motors.setLeftSpeed(-100);
      motors.setRightSpeed(100);
      if (millis() - lastCrossRoads > 500)
      { // går ut av case
        switchMode = 99;
      }
    }

  case 4: // gap i tape

    motors.setSpeeds(150, 150);

    // if sensorzum > 1000. {switchmode = 99}

  default:
    lineFollowPID();
    notInOtherCase = false; // 
  }
}

void oledPrinter() // printer ut verdier til skjermen
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

void resetMaxValue() // resetter max verdi
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
  gapControll();
  oledPrinter();
  resetMaxValue();
  countCrossRoads();
}
