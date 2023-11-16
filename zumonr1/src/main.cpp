#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4Encoders encoders;
Zumo32U4OLED oled;
Zumo32U4IMU imu;

long tikkFrem = 0;
float gyroX, gyroY, gyroZ;
int direction;       // hvilken modus er det rett frem/sving osv. bruker case
int svingTikk = 750; // hvor mange tikk for sinvg

void setup()
{
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
/*
void updateGyro()
{
    imu.readGyro(gyroX, gyroY, gyroZ);
    oled.print(gyroZ);
}
*/
void turnR()
{
    encoders.getCountsAndResetLeft();
    while (true)
    {
        tikkFrem = encoders.getCountsLeft();
        motors.setSpeeds(200, -200);
        if (tikkFrem > svingTikk)
        { // beveger seg halv meter
            motors.setSpeeds(0, 0);
            delay(500);
            break;
        }
    }
}

void turnL()
{
    encoders.getCountsAndResetRight();
    while (true)
    {
        tikkFrem = encoders.getCountsRight();
        motors.setSpeeds(-200, 200);
        if (tikkFrem > svingTikk)
        { // beveger seg halv meter
            motors.setSpeeds(0, 0);
            delay(500);
            break;
        }
    }
}

void turn180()
{
    {
        encoders.getCountsAndResetRight();
        while (true)
        {
            tikkFrem = encoders.getCountsRight();
            motors.setSpeeds(-100, 100);
            if (tikkFrem > (svingTikk*2))
            { // beveger seg halv meter
                motors.setSpeeds(0, 0);
                delay(500);
                break;
            }
        }
    }
}
void drive()
{
    switch (direction)
    {
    case (0):
        forwards();
        break;

    case (1):
        turnR();
        break;

    case (2):
        turnL();
        break;
    }
}

void loop()
{

    delay(3000);
    // forwards();
    forwards();
    turnL();
    forwards();
    turn180();
    forwards();
    turnR();
    forwards();

}
