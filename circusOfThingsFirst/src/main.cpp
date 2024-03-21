#include <Arduino.h>
#include <CircusESP32Lib.h>

int ledBluePin = 16;
int ledRedPin = 17;

#include <CircusESP32Lib.h>  // Biblioteket til Circus of Things.
char ssid[] = "NTNU-IOT";    // Skriv inn navnet til ruteren din her.
char password[] = "";        // Skriv inn passordet til ruteren din her.
char server[] = "www.circusofthings.com";  // Her ligger serveren.
char keyRed[] =
    "1590";  // Nøkkel-informasjon om konsollet for styring av LED hos CoT.
char keyBlue[] =
    "15218";  // Nøkkel-informasjon om konsollet for styring av LED hos CoT.
char token[] =
    "eyJhbGciOiJIUzI1NiJ9.eyJqdGkiOiIyOTExNCJ9.x3WFdG654q6OjhVFLE-"
    "UY4y8jcnwuNbpZ7J0jGPMkKE";
CircusESP32Lib circusESP32(
    server, ssid,
    password);  // Her leses nettadressen til,→ CoT, ssid, og //... passord inn.
                // Ikke gjør noen endringer her.

// Bruk LED1_state til ˚a styre LED av eller p˚a (digitalWrite())

void setup() {
  circusESP32.begin();  // Initialiserer oppkobling mot CoT
  pinMode(ledBluePin, OUTPUT);
  pinMode(ledRedPin, OUTPUT);
}

void loop() {
  // les av dashboard i CoT
  int LEDRed_state = circusESP32.read(keyRed, token);
  int LEDBlue_state = circusESP32.read(keyBlue, token);

  // utfør
  if (LEDRed_state == 1) {
    digitalWrite(ledRedPin, HIGH);
  } else if (LEDRed_state == 0) {
    digitalWrite(ledRedPin, LOW);
  }

  if (LEDBlue_state == 1) {
    digitalWrite(ledBluePin, HIGH);
  } else if (LEDBlue_state == 0) {
    digitalWrite(ledBluePin, LOW);
  }
}