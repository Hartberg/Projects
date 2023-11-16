#include <Arduino.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4Encoders encoders;
Zumo32U4OLED oled;
Zumo32U4LineSensors lineSensors;


int tikkFrem = 0; // register sist  
int startverdi = 0;
int lineSensorV = 0;
unsigned int lineSensorArray[5]; // array til sensorveridiene
int position = 0; // linejposijon
int speedLeft = 0; // venstre hjulhastigehet
int speedRight = 0; // høyre hjulhastighet
int normalSpeed = 225; // basisfart
int dLine = 0;
float lineMultiplier = 0; // tallet hjulene skal ganges med


void setup() {
  lineSensors.initFiveSensors();
  calibrate();
}


void forwards() 
{

    encoders.getCountsAndResetLeft();
    while (true)
    {
        tikkFrem = encoders.getCountsLeft();
        motors.setSpeeds(200, 200);
        if (tikkFrem > 3672)
        { // beveger seg halv meter
            motors.setSpeeds(0, 0);
            delay(500);
            break;
        }
    }
}

void calibrate() {
  delay(500);
  for (int i = 0; i < 300; i ++) {
    if (i < 90) {
      motors.setSpeeds(200, -200); // kjør i sirkel
    }
    else if (i > 130 && i < 150) { // slapp av litt for ikke å denge motor
      motors.setSpeeds(0,0);
    }
    else if (i > 150) {
      motors.setSpeeds(-200, 200); // kjør sirkel andre vei
    }
    lineSensors.calibrate();
  }
  motors.setSpeeds(0,0);
}

void printSensor(){
  oled.clear();
  oled.gotoXY(0,0);
  oled.print(position); // printer hvor den ser linja
  oled.gotoXY(0,1);
  oled.print(lineMultiplier); // printer gangeverdi

}

void followLine(){
  motors.setSpeeds(200,200);
  if (position < 1500) {
    motors.setSpeeds(10, 100);
  }
  else if ( position < 2500) {
     motors.setSpeeds(100, 100);
  }
  else if (position > 2500) {
    motors.setSpeeds(100, 10);
  }
}

void followLineP() { 
  dLine = position-2000;
  lineMultiplier = (map(position,0, 4000, 100.0, -100.0)/100.0);
  speedLeft = normalSpeed*(1-lineMultiplier);
  speedRight = normalSpeed*(1+lineMultiplier);
  motors.setSpeeds(speedLeft, speedRight);
}

void updateSensors(){
  position = lineSensors.readLine(lineSensorArray);
}

void loop() {
  if (startverdi == 0){ // kjører calibrate en gang
    calibrate();
    startverdi++;
  }
  updateSensors();
  printSensor();
  //followLine();
  followLineP();
}

