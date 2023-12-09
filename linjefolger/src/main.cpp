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
unsigned long lastCrossRoads; // tiden ved siste kryss

// printer variabler
unsigned long lastPrintTime; // tid ved sist print
int maxValue;                // høyeste verdi av valgfri ting

void calibrate()
{ // Kalibrere sensorene
  oled.clear();
  oled.gotoXY(0, 0);
  oled.print("calibrate");
  delay(500);
  for (int i = 0; i < 118; i++)
  {
    
    
      motors.setSpeeds(200, -200); // kjør i sirkel
    

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

// return true hvis den er rett på streken
bool isStraightOnLine(){ 
  if ((lineSensorArray[1] > 100 && lineSensorArray[2] > 800 && lineSensorArray[3] > 100) && (lineSensorArray[1]+lineSensorArray[3] < 1000)) {
    return true;
  }
  else {
    return false;
  }
}

// lagrer høyeste verdi av det du putter i 1. arg
int highestValue(int value) 
{
  if (value > maxValue)
  {
    maxValue = value;
  }
}

// gir ut summen av alle sensorene. lettere å jobb med enn 5 enkelt sensor
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
// Til bruk i kalibreringer og testing
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
    // printLineSensorReadingsToSerial();
  }
}



void lineFollowPID() // Gammel
{
  // leser sensor til linjefølger
  int16_t position = lineSensors.readLine(lineSensorArray);
  int16_t error = position - 2000;
  int16_t speedDifference = error / 4 + 8 * (error - lastError); // Proporsjonal term: error / 4 - Dette er en enkel proporsjonal komponent hvor feilen er delt på 4. Dette betyr at hastighetsforskjellen er proporsjonal med feilen, men skalert ned med 4.
  // Derivativ term: 6 * (error - lastError) - Dette er en derivativ komponent som er proporsjonal med endringen i feil over tid (derivasjon av feilen). Det multipliseres med 6 for å justere vektingen av denne termen.
  int16_t leftSpeed = followSpeed + speedDifference;
  int16_t rightSpeed = followSpeed - speedDifference;
  leftSpeed = constrain(leftSpeed, 0, followSpeed) + 50; // Constraining left and right speed to not be lower than 0 or higher than 400(followSpeed)
  rightSpeed = constrain(rightSpeed, 0, followSpeed) + 50;
  motors.setSpeeds(leftSpeed, rightSpeed); // Setting the speed for the motors
  
}
void PID()
{

  // leser sensor til linjefølger
  int16_t position = lineSensors.readLine(lineSensorArray);
  int16_t error = position - 2000;
  int16_t integral = 0.005 * error; // Integral term
  int16_t speedDifference = (error / 4) + (4 * (error - lastError)) + integral; // Proporsjonal term: error / 4 - Dette er en enkel proporsjonal komponent hvor feilen er delt på 4. Dette betyr at hastighetsforskjellen er proporsjonal med feilen, men skalert ned med 4.
  // Derivativ term: 6 * (error - lastError) - Dette er en derivativ komponent som er proporsjonal med endringen i feil over tid (derivasjon av feilen). Det multipliseres med 6 for å justere vektingen av denne termen.
  
  int16_t leftSpeed = followSpeed + speedDifference;
  int16_t rightSpeed = followSpeed - speedDifference;
  leftSpeed = constrain(leftSpeed, 0, followSpeed); // Constraining sørger for at hastigheten til julene ikke går under 0 eller overstiger followspeed
  rightSpeed = constrain(rightSpeed, 0, followSpeed);
  motors.setSpeeds(leftSpeed, rightSpeed); // Setting the speed for the motors
  lastError = error;
}

// følger linje med p regulering
void lineFollowP()
{ 
  lineMultiplier = (map(position, 0, 4000, 100.0, -100.0) / 100.0);
  speedLeft = normalSpeed * (1 - lineMultiplier);
  speedRight = normalSpeed * (1 + lineMultiplier);
  motors.setSpeeds(speedLeft, speedRight);
  lineMultiplier = (map(position, 0, 4000, 100.0, -100.0) / 100.0);
}

// Til bruk i countCrossRoads
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

// Se bort ifra midlertidig, vurder å slett
void gapControll()
{ // Sjekker om det har oppstått ett hull i løypa.
  if (gapIsDetected())
  { // Hvis hull, switch case 4 (kjør over)
  }
}
void countCrossRoads() // teller om man er nådd ett kryss. tidsdelay så den ikke registrerer ett kryss flere ganger
{
  if ((sensorZum > 2500 || lineSensorArray[0] > 900 && sensorZum > 2000 || lineSensorArray[4] > 900 && sensorZum > 2000) && millis() - lastCrossRoads > 2300 )
  {
    switchMode = numCrossRoads; // setter hvilken sving det er til switch casen
    numCrossRoads++;            // setter neste kryss
    Serial.print("Cross Count : ");
    Serial.println(numCrossRoads);

    if (numCrossRoads == 4)
    { // resetter etter siste kryss
      Serial.println("Last Cross, reached, resetting crosscount!");
      numCrossRoads = 0;
      Serial.print("Cross count after reset: ");
      Serial.println(numCrossRoads);
    }
    lastCrossRoads = millis(); // noterer når sist kryss var
  }

  if (gapIsDetected())
  {
    if (inOtherCase == false) //
    {                         // passer på at man ikke holder på med en anne case før man hopper inn i gap detect
      Serial.println(" GAP DETECTED ");
      switchMode = 4;         // vi kjører rett frem når det ikke oppdages noe strek
    }
  }
}

void switchting() {
  static bool casePrinted[5] = {false};  // Array for å holde styr på om hver case er printet

  switch (switchMode) {
    case 0: // første kryss
      if (!casePrinted[0]) {
        Serial.println("Case 0 ");
        casePrinted[0] = true;
      }
      inOtherCase = true;
      motors.setLeftSpeed(150);
      motors.setRightSpeed(230);
      if (millis() - lastCrossRoads > 500) {
        casePrinted[0] = false;  // Tilbakestiller variabelen når du går ut av casen
        switchMode = 99;
      }
      break;

    case 1: // andre kryss
      if (!casePrinted[1]) {
        Serial.println("Case 1");
        casePrinted[1] = true;
      }
      inOtherCase = true;
      motors.setLeftSpeed(200);
      motors.setRightSpeed(230);
      if (millis() - lastCrossRoads > 400) {
        casePrinted[1] = false;
        switchMode = 99;
      }
      break;

    case 2: // tredje kryss
      if (!casePrinted[2]) {
        Serial.println("Case 2");
        casePrinted[2] = true;
      }
      inOtherCase = true;
      motors.setLeftSpeed(200);
      motors.setRightSpeed(220);
      if (millis() - lastCrossRoads > 400) {
        casePrinted[2] = false;
        switchMode = 99;
      }
      break;

    case 3: // Blindvei
      if (!casePrinted[3]) {
        Serial.println("Case 3");
        casePrinted[3] = true;
      }
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
      if (millis() - lastCrossRoads > 400)
      { // går ut av case
        switchMode = 99;
      }
    }
    break;

    case 4: // gap i tape
      if (!casePrinted[4]) {
        Serial.println("Case 4");
        casePrinted[4] = true;
      }
      inOtherCase = true;
      motors.setSpeeds(200, 200);
      if (sensorZum > 1000) {
        switchMode = 99;
      }
      break;

    default:
      for (int i = 0; i < 5; i++) {
        casePrinted[i] = false;  // Tilbakestiller variablene for alle caser når du går ut av andre caser
      }
      inOtherCase = false;
      lineFollowPID();
  }
}

void normaDriving(){
  updateSensors();
  countCrossRoads();
  switchting();
}
/*void CrossActions() // kjøremodus for sving basert kjøring. kjører pid til vanlig pg går inn i case når den møter kryss.
{

  switch (switchMode)
  {
  case 0: // første kryss
    Serial.println("Case 0 ");
    inOtherCase = true;
    motors.setLeftSpeed(150);
    motors.setRightSpeed(230);
    if (millis() - lastCrossRoads > 500)
    { // går ut av case
      switchMode = 99;
    }
    break;

  case 1: // andre kryss
    //Serial.println("Case 1");
    inOtherCase = true;
    motors.setLeftSpeed(200);
    motors.setRightSpeed(230);
    if (millis() - lastCrossRoads > 400)
    { // går ut av case
      switchMode = 99;
    }
    break;

  case 2: // tredje kryss
    Serial.println("Case 2");
    inOtherCase = true;
    motors.setLeftSpeed(200);
    motors.setRightSpeed(220);
    if (millis() - lastCrossRoads > 400) // gå tilbake til default(PID)
    {                                    // går ut av case
      switchMode = 99;                   // setter den til default
    }
    break;

  case 3: // Blindvei
    Serial.println("Case 3");
    buzzer.playFrequency(6000, 250, 12); // tester om case aktiveres
    inOtherCase = true;
    if (millis() - lastCrossRoads > 2600) // kjører rett frem, tilbake og snu. if loopen starter nederst og går oppover
    {
      switchMode = 99;
    }
    else if (millis() - lastCrossRoads > 2100)
    {
      motors.setSpeeds(-180, 180);
    }
    else if (millis() - lastCrossRoads > 1100)
    {
      motors.setSpeeds(-normalSpeed, -normalSpeed);
    }
    else if (millis() - lastCrossRoads > 1000)
    {
      motors.setSpeeds(0, 0);
    }
    else if (millis() - lastCrossRoads < 1000)
    {
      motors.setSpeeds(normalSpeed, normalSpeed);
    }
    break;
  case 4:               // gap i tape

    inOtherCase = true; // kjører rett frem til det treffer en strek
    motors.setSpeeds(200, 200);
    if (sensorZum > 1000)
    {
      switchMode = 99;
    }
    break;

  default:
    inOtherCase = false;
    lineFollowPID();
  }
}
*/
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

void CrossPrint()
{
  oled.clear();
  oled.gotoXY(0,0);
  oled.print("cross: ");
  oled.gotoXY(0,1);
  oled.print(numCrossRoads);
  
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
  int randomOrder = random(0, 1000); // Random tall 0-100
  int randomMatch = 1;               // Hva random tallet må være for å kicke inn
  return randomOrder == randomMatch; // Returner true for taxi bestillt, false for taxi ikke bestillt.
}

void driveTaxi() {
  int randomCrossPickup = random(1, 4);
  int randomCrossDestination = random(1, 4);

  while (randomCrossPickup == randomCrossDestination) {
    randomCrossDestination = random(1, 4);
  }

  Serial.println("Taxi is ordered");
  Serial.print("Random Cross pickup: ");
  Serial.println(randomCrossPickup);
  Serial.print("Random Destination: ");
  Serial.println(randomCrossDestination);

  bool pickupCompleted = false;
  bool pickupPrinted = false;
  unsigned long orderTime = millis();

  while ((millis() - orderTime) < 1000) {
    motors.setSpeeds(0, 0);
    oled.clear();
    oled.setLayout21x8();
    oled.gotoXY(0, 2);
    oled.print(numCrossRoads);
    oled.gotoXY(0, 4);
    oled.print("Taxi is ordered");
    oled.gotoXY(0, 6);
    oled.print("Driving to ");
    oled.gotoXY(0, 7);
    oled.print("Bober nr. ");
    oled.print(randomCrossPickup);
  }

  while (!pickupCompleted) {
    if (randomCrossPickup == numCrossRoads) {
      if (!pickupPrinted) {
        Serial.println("Picking up!");
        pickupPrinted = true;
      }
      unsigned long pickupTime = millis();
      while ((millis() - pickupTime) < 5000) {
        motors.setSpeeds(0, 0);
        oled.clear();
        oled.setLayout21x8();
        oled.gotoXY(0, 2);
        oled.print(numCrossRoads);
        oled.gotoXY(0, 4);
        oled.print("Picking up");
        oled.gotoXY(0, 6);
        oled.print("Driving to ");
        oled.gotoXY(0, 7);
        oled.print("Bober nr. ");
        oled.print(randomCrossDestination);
      }
      pickupCompleted = true;
    } else {
      if (!pickupPrinted) {
        Serial.println("On the way to pick up");
        pickupPrinted = true;
      }
      normaDriving();
    }
  }

  bool dropOffCompleted = false;
  bool dropOffPrinted = false;

  while (!dropOffCompleted) {
    if (numCrossRoads == randomCrossDestination) {
      if (!dropOffPrinted) {
        Serial.println("Arrived!");
        dropOffPrinted = true;
      }
      unsigned long stopTime = millis();
      while ((millis() - stopTime) < 5000) {
        motors.setSpeeds(0, 0);
        oled.clear();
        oled.setLayout21x8();
        oled.gotoXY(0, 4);
        oled.print("We have arrived!");
      }
      Serial.println("Dropoff complete!");
      dropOffCompleted = true;
    } else {
      if (!dropOffPrinted) {
        Serial.println("On the way to drop off");
        oled.clear();
        oled.setLayout21x8();
        oled.gotoXY(0, 2);
        oled.print(numCrossRoads);
        oled.gotoXY(0, 4);
        oled.print("Taximeter running");
        oled.gotoXY(0, 6);
        oled.print("Driving to ");
        oled.gotoXY(0, 7);
        oled.print("Bober nr. ");
        oled.print(randomCrossDestination);
        dropOffPrinted = true;
      }
      normaDriving();
    }
  }
}

// Funksjon for taxi kjøring. skjer kun dersom taxi er bestillt.
/* uten printing
void driveTaxi()
{
  int randomCrossPickup = random(1, 4);      // Generer random kryss pasasjeren skal hentes i
  int randomCrossDestination = random(1, 4); // Generer random kryss pasasjeren skal slippes av i


  while (randomCrossPickup == randomCrossDestination){ // Sjekker at det ikke er samme kryss som plukkes opp og slippes av i. 
    randomCrossDestination = random(1,4);
  }

  Serial.print("Random Cross: ");
  Serial.println(randomCrossPickup);
  Serial.print("Random Destination: ");
  Serial.println(randomCrossDestination);

  bool pickupCompleted = false;              // Variabel for å sjekke om passasjeren har blitt hentet  
  unsigned long orderTime = millis();
  // Stopper i 1 sek for å demonstrere at en taxi er bestillt 
  while((millis() - orderTime) < 10000){
    motors.setSpeeds(0,0);
    oled.clear();
    oled.setLayout21x8();
    oled.gotoXY(0, 2);
    oled.print(numCrossRoads);
    oled.gotoXY(0, 4);
    oled.print("Taxi is ordered");
    oled.gotoXY(0, 6);
    oled.print("Driving to ");
    oled.gotoXY(0, 7);
    oled.print("Bober nr. ");
    oled.print(randomCrossPickup);
  }

  while (!pickupCompleted)
  {
    if (numCrossRoads == randomCrossPickup)
    {
      Serial.println("Picking up!");
      unsigned long pickupTime = millis();
      while ((millis() - pickupTime) < 5000)
      {
        motors.setSpeeds(0, 0);
        oled.clear();
        oled.setLayout21x8();
        oled.gotoXY(0, 2);
        oled.print(numCrossRoads);
        oled.gotoXY(0, 4);
        oled.print("Picking up");
        oled.gotoXY(0, 6);
        oled.print("Driving to ");
        oled.gotoXY(0, 7);
        oled.print("Bober nr. ");
        oled.print(randomCrossDestination);
      }
      pickupCompleted = true; // Passasjeren er nå plukket opp
    }
    else
    { // Dersom krysset ikke er nådd, fortsett linjefølging
      Serial.println("On the way to pick up");
      switchting();
      countCrossRoads();
    }
  }

  bool dropOffCompleted = false; // Sjekker om passasjeren har blitt plukket opp

  while (!dropOffCompleted)
  {
    if (numCrossRoads == randomCrossDestination)
    {
      Serial.println("Arrived!");
      unsigned long stopTime = millis();
      while ((millis() - stopTime) < 5000)
      {
        motors.setSpeeds(0, 0);
        oled.clear();
        oled.setLayout21x8();
        oled.gotoXY(0, 4);
        oled.print("We have arrived!");
      }
      Serial.println("Dropoff complete!");
      dropOffCompleted = true; // Pasasjer er nå sluppet av
    }
    else
    { // Dersom krysset ikke er nådd enda, følg linja
      Serial.println("On the way to drop off ");
      oled.clear();
      oled.setLayout21x8();
      oled.gotoXY(0, 2);
      oled.print(numCrossRoads);
      oled.gotoXY(0, 4);
      oled.print("Taximeter running");
      oled.gotoXY(0, 6);
      oled.print("Driving to ");
      oled.gotoXY(0, 7);
      oled.print("Bober nr. ");
      oled.print(randomCrossDestination);
      switchting();
      countCrossRoads();
    }
  }
}
*/
void setup()
{
  randomSeed(analogRead(0));
  Serial.begin(9600);
  lineSensors.initFiveSensors(); // initier de 5 sensorene
  buttonA.waitForButton();       // Vent på knappen før kalibrering
  calibrate();
}

void loop()
{
  updateSensors();
  countCrossRoads();
  driveTaxi();
  /*
  if (taxiOrder()){
    driveTaxi();
  } else {
    switchting();
  }*/
}