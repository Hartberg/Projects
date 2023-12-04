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
bool inOtherCase = false;         // sjekker om man er under opperasjon av en annen case

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
  }
}
void countCrossRoads() // teller om man er nådd ett kryss. tidsdelay så den ikke registrerer ett kryss flere ganger
{
  if (sensorZum > 2400 && millis() - lastCrossRoads > 2500)
  {
    switchMode = numCrossRoads; // setter hvilken sving det er til switch casen
    numCrossRoads++;            // setter neste kryss

    if (numCrossRoads == 5)
    { // resetter etter siste kryss
      numCrossRoads = 0;
    }
    lastCrossRoads = millis(); // noterer når sist kryss var
  }

  if (gapIsDetected())
  {
    if (inOtherCase == false)
    {                 // passer på at man ikke holder på med en anne case før man hopper inn i gap detect
      switchMode = 4; // vi kjører rett frem når det ikke oppdages noe strek
      if (onlyGapDetecedWithin10s == false)
      {
        timeSinceGapDetect = millis();
        onlyGapDetecedWithin10s = true;
      }

      if (millis() - timeSinceGapDetect > 50 && millis() - lastCrossRoads > 10000) // sjekker at det ikke er oppdaget noe gap på 10 sek og at man har vært i en gap i mer en 0.2s. slik at den ikke registrerer feil numcrossroads++;
      {                                                                            // endre gap bool etter 10s
        numCrossRoads++;
        lastCrossRoads = millis();
      }
    }
    else
    {
      onlyGapDetecedWithin10s = false;
    }
  }
}

void switchting() // kjøremodus for sving basert kjøring. kjører pid til vanlig pg går inn i case når den møter kryss.
{

  switch (switchMode)
  {
  case 0: // første kryss
    inOtherCase = true;
    motors.setLeftSpeed(150);
    motors.setRightSpeed(200);
    if (millis() - lastCrossRoads > 800)
    { // går ut av case
      switchMode = 99;
    }
    break;

  case 1: // andre kryss
    inOtherCase = true;
    motors.setLeftSpeed(200);
    motors.setRightSpeed(200);
    if (millis() - lastCrossRoads > 500)
    { // går ut av case
      switchMode = 99;
    }
    break;

  case 2: // tredje kryss
    inOtherCase = true;
    motors.setLeftSpeed(200);
    motors.setRightSpeed(200);
    if (millis() - lastCrossRoads > 500) // gå tilbake til default(PID)
    {                                    // går ut av case
      switchMode = 99;                   // setter den til default
    }
    break;

  case 3:                                // Blindvei
    buzzer.playFrequency(6000, 250, 12); // tester om case aktiveres
    inOtherCase = true;
    if (millis() - lastCrossRoads < 50) // skrur av bilen i 50ms for å spare motor. vil ikke gå fra positiv til negativ fart
    {
      motors.setSpeeds(0, 0);
    }
    else // snur bilen 90 grader
    {
      motors.setLeftSpeed(-180);
      motors.setRightSpeed(180);
      if (millis() - lastCrossRoads > 500)
      { // går ut av case
        switchMode = 99;
      }
    }
    break;

  case 4: // gap i tape
    inOtherCase = true; // kjører rett frem til det treffer en strek
    motors.setSpeeds(150, 150);

    if (sensorZum > 1000) 
    {
      switchMode = 99;
    }
    break;

    // if sensorzum > 1000. {switchmode = 99}

  default:
    inOtherCase = false;
    lineFollowPID();
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
    oled.gotoXY(0, 5);
    oled.print("switchmode");
    oled.print(switchMode);

    lastPrintTime = millis();
  }
}

void resetMaxValue() // resetter max verdi ved b trykk
{
  if (buttonB.getSingleDebouncedPress())
  {
    maxValue = 0;
  }
}

// Taxi bestillt? ja/nei. til bruk i driveTaxi()
bool taxiOrder()
{
  int randomOrder = random(0, 10);   // Random tall 0-100
  int randomMatch = 1;               // Hva random tallet må være for å kicke inn
  return randomOrder == randomMatch; // Returner true for taxi bestillt, false for taxi ikke bestillt.
}

// Funksjon for taxi kjøring. skjer kun dersom taxi er bestillt.
void driveTaxi()
{

  int randomCrossPickup = random(0, 4);      // Generer random kryss pasasjeren skal hentes i
  int randomCrossDestination = random(0, 4); // Generer random kryss pasasjeren skal slippes av i
  bool pickupCompleted = false;              // Variabel for å sjekke om passasjeren har blitt hentet
  bool dropOffCompleted = false;             // Sjekker om passasjeren har blitt plukket opp

  oled.clear();
  oled.setLayout21x8();
  oled.gotoXY(0, 4);
  oled.print("Taxi is ordered");
  oled.gotoXY(0, 6);
  oled.print("Driving to ");
  oled.gotoXY(0, 7);
  oled.print("Bober nr. ");
  oled.print(randomCrossPickup);

  while (!pickupCompleted)
  {
    if (numCrossRoads == randomCrossPickup)
    {
      unsigned long pickupTime = millis();
      while ((millis() - pickupTime) < 5000)
      {
        motors.setSpeeds(0, 0);
        oled.clear();
        oled.setLayout21x8();
        oled.gotoXY(0, 4);
        oled.print("Picking up passenger");
      }
      pickupCompleted = true; // Passasjeren er nå plukket opp
    }
    else
    { // Dersom krysset ikke er nådd, fortsett linjefølging
      lineFollowPID();
    }
  }
  while (!dropOffCompleted)
  {
    if (numCrossRoads == randomCrossDestination)
    {
      unsigned long stopTime = millis();
      while ((millis() - stopTime) < 5000)
      {
        motors.setSpeeds(0, 0);
        oled.clear();
        oled.setLayout21x8();
        oled.gotoXY(0, 4);
        oled.print("We have arrived!");
      }
      dropOffCompleted = true; // Pasasjer er nå sluppet av
    }
    else
    { // Dersom krysset ikke er nådd enda, følg linja
      lineFollowPID();
    }
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

void findChargingStation() // Vurder å drit i :)
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

// irsocket: bilibliotek til ir sending