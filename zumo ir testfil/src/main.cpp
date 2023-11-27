#include <arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <IRremote.h>

Zumo32U4Motors motors;


#define code1 3108437760
#define code2 3927310080
#define code3 2907897600

const long RECV_PIN = A4;
IRrecv irrecv(RECV_PIN);
unsigned long nowTime;
unsigned long considerTime;
unsigned long irNum;

void setup(){
	Serial.begin(9600);
	IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);

}


 
void loop(){
	if(IrReceiver.decode()){
	unsigned long irNum = IrReceiver.decodedIRData.decodedRawData;  
	Serial.println(irNum);
	switch (irNum)
	{
	case code1:
		motors.setSpeeds(200,200);
		break;

	case code2:
		motors.setSpeeds(0,0);
	break;

	case code3:
	Serial.println("3");
	break;
	
	default:
		break;
	}
	
	}
IrReceiver.resume();
}