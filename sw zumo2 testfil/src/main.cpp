// RR Linjefølgerbane
// Bane bestående av:
// 1. Slakke/Krappe svinger.
// 2. rettvinklede svinger.
// 3. vei med manglende teip.
// 4. blindvei så bilen må snu. der rikrig vei er 90* venstre. bilen skal her klare å snu etter å ha kjørt rett fram, tilbake dit den kom fra.

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
const uint16_t normalSpeed = 250;          // Normal hastighet til bilen
int16_t lastError = 0;                     // Variabel for feilmargin
int16_t position;                          // bilens posisjon i forhold til linja
int speedLeft = 0;                         // venstre hjulhastigehet
int speedRight = 0;                        // høyre hjulhastighet
float lineMultiplier = 0;                  // tallet hjulene skal ganges med for linjefølging

// Variabler for kryss
bool crossRoad = false;         // Er det kjørt forbi kryss? ja/nei
int16_t numCrossRoads = 0;      // Large antallet kryss kjørt fobi
int16_t threshold = 200;        // Hva som regnes som mørk linje
int crossDetectionDelay = 2500; // tid som må gå fra et kryss er registrert til neste kan registreres

// switch variabler
int switchMode = 99;          // casen til switchcase ved kryssbasert kjøring
bool inOtherCase = false;     // sjekker om man er under opperasjon av en annen case
int sensorZum;                // summen av alle sensorene
unsigned long lastCrossRoads; // tiden ved siste kryss

// printing
int maxValue; // høyeste verdi av valgfri ting

// Banken
float bankBalace = 0.0;

// return true hvis den er rett på streken
bool isStraightOnLine()
{
  if ((lineSensorArray[1] > 100 && lineSensorArray[2] > 800 && lineSensorArray[3] > 100) && (lineSensorArray[1] + lineSensorArray[3] < 1000))
  {
    return true;
  }
  else
  {
    return false;
  }
}

// Kalibrere sensorene
void calibrate()
{
  oled.clear();
  oled.gotoXY(0, 0);
  oled.print("calibrate");
  delay(500);
  for (int i = 0; i < 118; i++)
  {
    if (i < 118)
    {
      motors.setSpeeds(200, -200); // kjør i sirkel
    }

    lineSensors.calibrate();
  }
  while (true)
  {
    oled.clear();
    oled.print("done");
    motors.setSpeeds(200, -200);
  }
  motors.setSpeeds(0, 0);
}

void highestValue(int value) // lagrer høyeste verdi av det du putter i 1. arg.
{                            // Til bruk i lineSensorZum()
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

// leser posisjonen til linjefølger
void updateSensors()
{
  position = lineSensors.readLine(lineSensorArray);
  lineSensorZum();
}

// Uten I
void lineFollowPID()
{
  // leser sensor til linjefølger
  int16_t position = lineSensors.readLine(lineSensorArray);
  int16_t error = position - 2000;
  int16_t speedDifference = error / 4 + 8 * (error - lastError); // Proporsjonal term: error / 4 - Dette er en enkel proporsjonal komponent hvor feilen er delt på 4. Dette betyr at hastighetsforskjellen er proporsjonal med feilen, men skalert ned med 4.
  // Derivativ term: 6 * (error - lastError) - Dette er en derivativ komponent som er proporsjonal med endringen i feil over tid (derivasjon av feilen). Det multipliseres med 6 for å justere vektingen av denne termen.
  int16_t leftSpeed = normalSpeed + speedDifference;
  int16_t rightSpeed = normalSpeed - speedDifference;
  leftSpeed = constrain(leftSpeed, 0, normalSpeed) + 50; // Constraining sørger for at hastigheten til julene ikke går under 0 eller overstiger normalSpeed
  rightSpeed = constrain(rightSpeed, 0, normalSpeed) + 50;
  motors.setSpeeds(leftSpeed, rightSpeed); // Setter farta til motorene
}
// Med I
void PID()
{

  // leser sensor til linjefølger
  int16_t position = lineSensors.readLine(lineSensorArray);
  int16_t error = position - 2000;
  int16_t integral = 0.005 * error;                                            // Integral term
  int16_t speedDifference = error / 0.25 + 3 * (error - lastError) + integral; // Proporsjonal term: error / 4 - Dette er en enkel proporsjonal komponent hvor feilen er delt på 4. Dette betyr at hastighetsforskjellen er proporsjonal med feilen, men skalert ned med 4.
  // Derivativ term: 6 * (error - lastError) - Dette er en derivativ komponent som er proporsjonal med endringen i feil over tid (derivasjon av feilen). Det multipliseres med 6 for å justere vektingen av denne termen.
  int16_t leftSpeed = normalSpeed + speedDifference;
  int16_t rightSpeed = normalSpeed - speedDifference;
  leftSpeed = constrain(leftSpeed, 0, normalSpeed); // Constraining sørger for at hastigheten til julene ikke går under 0 eller overstiger normalSpeed
  rightSpeed = constrain(rightSpeed, 0, normalSpeed);
  motors.setSpeeds(leftSpeed, rightSpeed); // Setter farta til motorene
  lastError = error;
}

// følger linje med p regulering
void lineFollowP()
{
  lineMultiplier = (map(position, 0, 4000, 100.0, -100.0) / 100.0);
  speedLeft = normalSpeed * (1 - lineMultiplier);
  speedRight = normalSpeed * (1 + lineMultiplier);
  motors.setSpeeds(speedLeft, speedRight);
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
void CrossPrint()
{
  oled.clear();
  oled.gotoXY(0, 0);
  oled.print("cross: ");
  oled.print(numCrossRoads);
}
// teller om man er nådd ett kryss. tidsdelay så den ikke registrerer ett kryss flere ganger
void countCrossRoads()
{
  if ((sensorZum > 2500 || lineSensorArray[0] > 900 && sensorZum > 2000 || lineSensorArray[4] > 900 && sensorZum > 2000) && millis() - lastCrossRoads > crossDetectionDelay)
  {
    switchMode = numCrossRoads; // setter hvilken sving det er til switch casen
    numCrossRoads++;            // setter neste kryss
    // Seriell kommunikasjon brukt til testing
    Serial.print("Cross Count : ");
    Serial.println(numCrossRoads);
    CrossPrint();
    crossDetectionDelay = 2500; // setter CDD tilbake til normal

    if (numCrossRoads == 4)
    { // resetter etter siste kryss
      numCrossRoads = 0;
    }
    lastCrossRoads = millis(); // noterer når sist kryss var
  }
  // Dersom det ikke er kryss men bare hull i linja
  if (gapIsDetected())
  {
    if (inOtherCase == false)
    {                 // passer på at man ikke holder på med en anne case før man hopper inn i gap detect
      switchMode = 4; // vi kjører rett frem når det ikke oppdages noe strek
    }
  }
}

// Linjefølging,telling av kryss og handlinger i kryss

// kjøremodus for sving basert kjøring. Går inn i case når den møter kryss.
void CrossActions()
{
  switch (switchMode)
  {
  case 0: // første kryss
    inOtherCase = true;
    motors.setLeftSpeed(150);
    motors.setRightSpeed(230);
    if (millis() - lastCrossRoads > 500) // Sving litt i kun 0.5 sekund så den finner linja igjen
    {                                    // går ut av case
      switchMode = 99;                   // setter den til default
    }
    break;

  case 1: // andre kryss
    inOtherCase = true;
    motors.setLeftSpeed(200);
    motors.setRightSpeed(230);
    if (millis() - lastCrossRoads > 400) // Sving litt i kun 0.4 sekund så den finner linja igjen
    {                                    // går ut av case
      switchMode = 99;                   // setter den til default
    }
    break;

  case 2: // tredje kryss
    inOtherCase = true;
    motors.setLeftSpeed(200);
    motors.setRightSpeed(220);
    if (millis() - lastCrossRoads > 400) // Sving litt i kun 0.4 sekund så den finner linja igjen
    {                                    // går ut av case
      switchMode = 99;                   // setter den til default
    }
    break;

  case 3: // Blindvei
    inOtherCase = true;
    crossDetectionDelay += 2600; // legger til slik at den ikke registrer kryss på nytt under opplegget
    // kjører rett frem, tilbake og snu. if loopen starter nederst og går oppover
    if (millis() - lastCrossRoads > 2600)
    {
      switchMode = 99;
    }
    else if (millis() - lastCrossRoads > 2200)
    {
      motors.setSpeeds(-180, 180);
    }
    else if (millis() - lastCrossRoads > 1200)
    {
      motors.setSpeeds(-normalSpeed, -normalSpeed);
    }
    else if (millis() - lastCrossRoads > 1050)
    {
      motors.setSpeeds(0, 0);
    }
    else if (millis() - lastCrossRoads < 1000)
    {
      motors.setSpeeds(normalSpeed, normalSpeed);
    }
    break;
  case 4: // gap i tape

    inOtherCase = true; // kjører rett frem til det treffer en strek
    motors.setSpeeds(normalSpeed, normalSpeed);
    if (sensorZum > 1000)
    {
      switchMode = 99;
    }
    break;
    // Dersom bilen ikke er inni en case, altså ikke er på et kryss, skal den føge linja som normalt
  default:
    inOtherCase = false;
    lineFollowPID();
  }
}
// Printer antallet kryss vi nå har passert
void normalDriving()
{
  updateSensors();   // Oppdaterer linjesensorene til å lese posisjonen
  countCrossRoads(); // Tell kryss
  CrossActions();    // Kjør med gitte caser for kryssene
}
// Setter penger på konto
void deposit(float amount)
{
  bankBalace += amount;
}
// Trekker penger fra konto
void withdraw(float amount)
{
  if (amount <= bankBalace)
  {
    bankBalace -= amount; // trekker amount fra bankBalance
  }
  else
  { // Dersom man ikke har nok penger får man det pent forklart
    oled.clear();
    oled.setLayout21x8();
    oled.gotoXY(0, 2);
    oled.print("U too broke bitch!");
    oled.gotoXY(0, 4);
    oled.print("Current balance: ");
    oled.gotoXY(0, 6);
    oled.print(bankBalace);
  }
}
// Taxi bestillt? true/false. til bruk i driveTaxi()
bool taxiOrder()
{
  int randomOrder = random(0, 1001); // Random tall 0-1000
  int randomMatch = 1;               // Hva random tallet må være for å kicke inn
  return randomOrder == randomMatch; // Returner true dersom de to variablene er like, false ellers
}



// Henter og plukker opp på random kryss og beregner betaling utifra tiden kjørt
void driveTaxi()
{
  unsigned long orderTime = millis(); // Registrer tiden en bestilling har tikket inn

  // Genererer randome tall fra 1-3 for hvilket kryss passasjeren skal hentes i, og hvilket den skal slippes av i
  int randomCrossPickup = random(1, 4);
  int randomCrossDestination = random(1, 4);
  while (randomCrossPickup == randomCrossDestination)
  { // Sjekker at de to randome talla ikke er like
    randomCrossDestination = random(1, 4);
  }

  bool pickupCompleted = false;  // Sier at passasjeren ikke er plukket opp
  bool dropOffCompleted = false; // Sier at passasjeren ikke er sluppet av
  unsigned long taximeterStart;  // Tiden bilen starter å kjøre etter å ha plukket opp passasjer

  // Kontroll printing, kan slette før levering
  Serial.println("Taxi is ordered");
  Serial.print("Random Cross pickup: ");
  Serial.println(randomCrossPickup);
  Serial.print("Random Destination: ");
  Serial.println(randomCrossDestination);

  // Stopper bilen i 1 sek for å illustrere at en bestilling er kommet inn
  while ((millis() - orderTime) < 1000)
  {
    motors.setSpeeds(0, 0);
    // Printing til oled
    oled.clear();
    oled.setLayout21x8();
    oled.gotoXY(0, 2);
    oled.print("Taxi is ordered");
    oled.gotoXY(0, 4);
    oled.print("Driving to ");
    oled.gotoXY(0, 6);
    oled.print("Bober nr. ");
    oled.print(randomCrossPickup);
  }

  while (!pickupCompleted)
  {                                         // Så lenge passasjeren ikke er henta
    if (randomCrossPickup == numCrossRoads) // Sjekk om man har annkommet krysset for å hente passasjeren
    {
      unsigned long pickupTime = millis();   // Tiden passasjeren blir plukket opp
      while ((millis() - pickupTime) < 5000) // hvis ankommet krysset, Stop i 5 sek for å plukke opp passasjeren
      {
        crossDetectionDelay = 7300; // Øker delayet i countCrossRoads så den ikke teller krysset den stopper på som nytt kryss igjen når den kjører videre
        motors.setSpeeds(0, 0);
        // Printing til oled
        oled.clear();
        oled.setLayout21x8();
        oled.gotoXY(0, 2);
        oled.print("Picking up");
        oled.gotoXY(0, 4);
        oled.print("Driving to ");
        oled.gotoXY(0, 6);
        oled.print("Bober nr. ");
        oled.print(randomCrossDestination);
      }
      pickupCompleted = true;    // Registrer at passasjeren er hentet
      taximeterStart = millis(); // Start taximeteret etter passasjeren er pluket opp
    }
    else
    { // Fram til man annkommer krysset, kjør som normalt
      normalDriving();
    }
  }

  while (!dropOffCompleted)
  { // Så lenge passasjeren ikke er registrert som sluppet av

    if (numCrossRoads == randomCrossDestination) // Sjekk om man har nådd krysset man skal slippe av passasjeren i
    {
      unsigned long stopTime = millis();                               // Lagrer tiden bilen stoppet opp for å slippe av passasjer
      unsigned long totalDrivingTime = stopTime - taximeterStart;      // Beregner tiden det tok fra passasjeren var i bilen til den ble sluppet av
      float costPerSecond = 1.0;                                       // Hvor mye det koster å sitte på taxi
      float totalCostTaxi = totalDrivingTime / 1000.0 * costPerSecond; // Beregner det passasjeren må betale baser på sin kjøretid
      while ((millis() - stopTime) < 5000)                             // Stop i 5 sekund for å slippe av passasjer
      {
        crossDetectionDelay = 7300; // Øker delayet i countCrossRoads så den ikke teller krysset den stopper på som nytt kryss igjen når den kjører videre
        motors.setSpeeds(0, 0);
        oled.clear();
        oled.setLayout21x8();
        oled.gotoXY(0, 2);
        oled.print("We have arrived!");
        oled.gotoXY(0, 4);
        oled.print("Your total is: ");
        oled.print(totalCostTaxi);
      }

      dropOffCompleted = true; // Registrer passasjeren som sluppet av
      deposit(totalCostTaxi);  // Legg til beløpet vi fikk for turen i bankkontoen
    }
    else
    { // Fram til passasjeren blir sluppet av, kjør normalt
      oled.clear();
      oled.setLayout21x8();
      oled.gotoXY(0, 4);
      oled.print("Taximeter running");
      oled.gotoXY(0, 6);
      oled.print("Driving to ");
      oled.gotoXY(0, 7);
      oled.print("Bober nr. ");
      oled.print(randomCrossDestination);
      normalDriving();
    }
  }
}

void setup()
{
  randomSeed(analogRead(0));     // Sørger for at det genereres nye randome taxibestillinger hver kjøring
  Serial.begin(9600);            // Initierer seriell kommunikasjon
  lineSensors.initFiveSensors(); // initier de 5 sensorene
  buttonA.waitForButton();       // Vent på knappen før kalibrering
  calibrate();                   // Kalibrerer de 5 linjesensorene for å lese underlaget
}

void loop()
{
  if (taxiOrder()) // Sjekker om det er kommet inn en taxi-bestilling
  {
    driveTaxi(); // Hvis bestillt, kjør taxi
  }
  else // Hvis ikke bestillt, kjør normalt
  {
    oled.clear();
    oled.setLayout11x4();

    oled.gotoXY(0, 0);
    oled.print(numCrossRoads);
    oled.gotoXY(0, 1);
    oled.print(millis() - lastCrossRoads);
    oled.gotoXY(0, 2);
    oled.print(crossDetectionDelay);

    /*
    oled.gotoXY(0, 3);
    oled.print("Crosses passed: ");
    oled.print(numCrossRoads);
    */
    normalDriving();
  }
}
