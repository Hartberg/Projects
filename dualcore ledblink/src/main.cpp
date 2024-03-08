#include <Arduino.h>

int const buttonPin = 16; 
gpio_num_t buttonpin = GPIO_NUM_16; // noen funksjoner trenger egen gpio_num_t datatype på button.

//ved bruk i lightsleep. bli vekket av knapp deretter gå til søvn
void readButtonLS()  
{
  Serial.println(digitalRead(buttonPin));
  esp_light_sleep_start();
}

//setup til LS
/*
void setup()
{
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_ON); // pass på at knapp er aktiv under søvn
  esp_sleep_enable_gpio_wakeup(); //skru på at gpio kan vekke
  gpio_wakeup_enable(buttonpin, GPIO_INTR_HIGH_LEVEL); // velg hvilken pin som skal vekke
}
*/

//setup til DS
void setup(){
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  esp_sleep_enable_timer_wakeup(7*pow(10,6)); // ved deepsleep vekk etter 7 sek
}

//denne loopen leser av knappen hvert 7 sek og sover.
void loop()
{
    Serial.println(digitalRead(buttonPin));
    esp_deep_sleep_start();
}