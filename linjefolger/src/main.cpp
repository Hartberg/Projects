// Linjefølgerbane
// Bane bestående av: 
// 1. Slakke/Krappe svinger. 
// 2. rettvinklede svinger. 
// 3. vei med manglende teip. 
// 4. blindvei så bilen må snu. der skal blindveien illustreres med en T form, der rikrig vei er 90* venstre. bilsen skal her klare å snu etter å ha kjørt rett fram, tilbake dit den kom fra.
// (bare rygge?, så ta venstre. registrere at vi kjørte forbi kryss ut på )
// Break-beam(lyskryss) på start/finish linje
// Finne ladestasjonen
#include <Wire.h>
#include <Zumo32U4.h>
#include <Arduino.h>
Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4OLED oled;
#define NUM_SENSORS 5                       // 5 Linjefølger-sensorer

// linjefølger
unsigned int lineSensorArray[NUM_SENSORS]; // Array for linjesensorverdiene
const uint16_t followSpeed = 250;  // Hastighet for linjefølging PID
int16_t lastError = 0;             // Variabel for feilmargin
int16_t position;                // bilens posisjon i forhold til linja
int speedLeft = 0;               // venstre hjulhastigehet
int speedRight = 0;              // høyre hjulhastighet
int normalSpeed = 225;           // basisfart for linjefølging P
float lineMultiplier = 0;        // tallet hjulene skal ganges med for linjefølging

// Variabler for kryss
bool crossRoad = false;     // Er det kjørt forbi kryss? ja/nei
bool tCrossRoad = false;    // 
int16_t numCrossRoads;   // Large antallet kryss kjørt fobi
int16_t numTCrossRoads;  // Lagre antallet T-kryss kjørt forbi
int16_t threshold = 200;     // Hva som regnes som mørk linje

void setup(){
  lineSensors.initFiveSensors(); //initier de 5 sensorene
  buttonA.waitForButton(); // Wait for button before running the program
  calibrate();

}

void loop(){
    updateSensors();
  lineFollowP();
  if(detectCrossRoad()){
    if (!crossRoad){
      crossRoad = true;
      numCrossRoads++;
      oled.clear();
      oled.setLayout21x8();
      oled.gotoXY(0, 2);
      oled.print("Crossroad detected");
      oled.gotoXY(0,4);
      oled.print("Number Crossroads: ");
      oled.print(numCrossRoads);
    }
  }
}

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

}

void updateSensors()
{ // leser posisjonen til linjefølger
  position = lineSensors.readLine(lineSensorArray);
}

void lineFollowPID(){
  // leser sensor til linjefølger
  int16_t position = lineSensors.readLine(lineSensorArray);
  int16_t error = position - 2000;
  int16_t speedDifference = error / 4 + 6 * (error-lastError); // Proporsjonal term: error / 4 - Dette er en enkel proporsjonal komponent hvor feilen er delt på 4. Dette betyr at hastighetsforskjellen er proporsjonal med feilen, men skalert ned med 4.
  // Derivativ term: 6 * (error - lastError) - Dette er en derivativ komponent som er proporsjonal med endringen i feil over tid (derivasjon av feilen). Det multipliseres med 6 for å justere vektingen av denne termen.
  int16_t leftSpeed = followSpeed + speedDifference;
  int16_t rightSpeed = followSpeed - speedDifference;
  leftSpeed = constrain(leftSpeed, 0 , followSpeed) + 50;               // Constraining left and right speed to not be lower than 0 or higher than 400(followSpeed)
  rightSpeed = constrain(rightSpeed, 0 , followSpeed) + 50;
  motors.setSpeeds(leftSpeed, rightSpeed);                      // Setting the speed for the motors
}
void lineFollowP()
{ // følger linje med p regulering
  /*lineMultiplier = (map(position, 0, 4000, 100.0, -100.0) / 100.0);
  speedLeft = normalSpeed * (1 - lineMultiplier);
  speedRight = normalSpeed * (1 + lineMultiplier);
  motors.setSpeeds(speedLeft, speedRight);*/
  lineMultiplier = (map(position, 0, 4000, 100.0, -100.0) / 100.0);

  if (lineMultiplier > -0.1 && lineMultiplier < 0.1) {
    // Hvis linjemultiplikatoren er nær null, anta at det ikke er noen linje
    motors.setSpeeds(normalSpeed, normalSpeed); // Fortsett rett fram
  } else {
    // Juster hastighetene basert på linjemultiplikatoren
    speedLeft = normalSpeed * (1 - lineMultiplier);
    speedRight = normalSpeed * (1 + lineMultiplier);
    motors.setSpeeds(speedLeft, speedRight);
  }
}
bool detectCrossRoad(){
  for (int i = 0; i < NUM_SENSORS; ++i){
    if ( lineSensorArray[i] < threshold){
      return false;
    }
  }
  return true;
}
  /*sjekk om man har kjørt forbi et veikryss. 
  det skjer dersom enten/både h og v sensorer leser svart.
    hvis veikryss = ja
        veikryss count = +1
    if både h og v = kryss(sort), 
    tKryss = ja
    tKryssCount = +1
    kjør blindvei()*/


void blindvei(){
    rygg tilbake til der krysset er 
    snu 90* venstre
    følg linjen igjen
}

void breakBeam(){
    Sjekk om rødt lys 
    if rødt lys 
        stop
    
}

void findChargingStation(){
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

void dontCrash(){ ? ikke nødvendig 
    sjekk om man er nært noe 
    hvis litt nært, sakk ned 
    hvis veeeeldig nært, stop, rygg, alternativ rute?
    tut om noen kjører forran :)

}

void Taxi(){
    random kick inn.
    hent passasjer etter random antall kryss
        tut når utenfor
    kjør passasjer til random kryss
    slipp av passasjer
        få penger 
        caching lyd :)
}