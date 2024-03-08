//  Hva gjør denne koden?
//  1. Analyser  2. Kommenter  3. Skriv funksjonalitetskrav
#include <Arduino.h>
int redLED = 10;
int greenLED = 9;
int buzzerPin = 11;
int GameMode = 6;
int LED1 = 6;
int LED2 = 7;
int SW1 = 3;
int SW2 = 4;

int winner = 0;
int winnerBeep = 750;
int fault = 0;
int faultBeep = 200;

unsigned long wait = 0;
unsigned long now = 0;

void setup()
{
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
}





void waitForButton()
{
  digitalWrite(greenLED, HIGH);
  while (digitalRead(SW1) == HIGH && digitalRead(SW2) == HIGH)
  { // venter til en av knappene blir trykket
  }
  if (digitalRead(SW1) == LOW)
  { // spiller 1 vinner
    winner = LED1;
    GameMode = 2; // 2
  }
  else
  { // hvis ikke vinner spiller 2
    winner = LED2;
    GameMode = 3;
  }
  digitalWrite(greenLED, LOW); // skrur av grønt lys
                               // avslutt case
}


void vent()
{
  digitalWrite(redLED, HIGH);

  fault = 0;
  now = millis();
  wait = now + random(3000, 7000);
  while (millis() < wait && digitalRead(SW1) == HIGH && digitalRead(SW2) == HIGH)
  {
  }
  GameMode =1; // set case to next phase
}


void winBuzzer1()
{
  for (int k = 0; k < 5; k++)
  {                                    // If someone pressed button when light green, play "winner" sound
    tone(buzzerPin, (750 + (k * 20))); // increases frequency with each iteration
    digitalWrite(greenLED, HIGH);      // Green light On
    digitalWrite(LED1, HIGH);          // Make buzzer play winner freq
    delay(50);                         // Wait 50ms
    digitalWrite(LED1, LOW);           // Turn off buzzer
    digitalWrite(greenLED, LOW);       // Turn off green in LED
    delay(50);                         // Wait a bit before itterating further
  }
  noTone(buzzerPin);
  GameMode = 6; // goes to start
}


void winBuzzer2()
{
  for (int k = 0; k < 5; k++)
  {                                    // If someone pressed button when light green, play "winner" sound
    tone(buzzerPin, (750 + (k * 20))); // increases frequency with each iteration
    digitalWrite(greenLED, HIGH);      // Green light On
    digitalWrite(LED2, HIGH);          // Make buzzer play winner freq
    delay(50);                         // Wait 50ms
    digitalWrite(LED2, LOW);           // Turn off buzzer
    digitalWrite(greenLED, LOW);       // Turn off green in LED
    delay(50);                         // Wait a bit before itterating further
  }
  noTone(buzzerPin);
  GameMode = 6; // goes to start
}


void loseBuzzer1()
{

  tone(buzzerPin, 200, 500);
  for (int k = 0; k < 10; k++)
  {
    digitalWrite(redLED, HIGH); // Turn red in RGB LED on
    digitalWrite(LED1, HIGH);   // Makes the buzzer emit fualt freq
    delay(50);                  // emitting for this duration
    digitalWrite(redLED, LOW);  // Turn red in RGB LED off
    digitalWrite(LED1, LOW);    // Turn off the buzzer
    delay(50);                  // Wait a bit before ittrting further
    /* for loop to create a sound that is "moving" and
    make the red light blink inducing fear into the
    player who played it wrong and humiliating him*/
  }
  GameMode = 6; // goes to start
}


void loseBuzzer2()
{

  tone(buzzerPin, 200, 500);
  for (int k = 0; k < 10; k++)
  {
    digitalWrite(redLED, HIGH); // Turn red in RGB LED on
    digitalWrite(LED2, HIGH);   // Makes the buzzer emit fualt freq
    delay(50);                  // emitting for this duration
    digitalWrite(redLED, LOW);  // Turn red in RGB LED off
    digitalWrite(LED2, LOW);    // Turn off the buzzer
    delay(50);                  // Wait a bit before ittrting further
    /* for loop to create a sound that is "moving" and
    make the red light blink inducing fear into the
    player who played it wrong and humiliating him*/
  }
  GameMode = 6; // goes to start
}

void loop()
{

  switch (GameMode)
  {
  case 1: // vente på knappetryk i game modus
    waitForButton();
    break;
  case 2: // Win player 1
    winBuzzer1();
    break;
  case 3: // win player 2
    winBuzzer2();
    break;
  case 4: // fault p1
    loseBuzzer1();
    break;
  case 5: // fault p2
    loseBuzzer2();
    break;
  case 6: // wait
    vent();

    if (digitalRead(SW1) == LOW)
    {
      fault = LED1;
      GameMode = 4;              // endre slik at gamemode er lik tap case
      digitalWrite(redLED, LOW); // skrur av rødlyset
      break;
    }
    else if (digitalRead(SW2) == LOW)
    {
      fault = LED2;
      GameMode = 5; // case tap spiller 2
      break;
    }
    break;
  }
  digitalWrite(redLED, LOW);
}