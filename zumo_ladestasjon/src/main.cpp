#include <Arduino.h>
int batteryLevel = 100


void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}





void DriveToChargingStation(){
    Når bilen kjører mot ladestasjonen, sjekk batterinivå
    dersom batterinivå er >=5%, stopp innom ladestasjon. 
    velg mellom 6 alternativer
        1. Lad fullt opp. koster mest(200), påvirker battery_health
        2. Lad til 80%, koster mindre(100), påvirker ikke battery_health
        3. Lad for x mengde kroner, vil koste maks samme som full lading(200) ,kan påvire batteriet dersom det blir ladet >80%
        4. Lad hurtig, 10x normal hastighet, kun opp til 50%, koster 2x mer som å lade fullt(400), påvirker battery_health
        5. Battery service, increase battery health with 60, koster 1000
        6. Battery change, sets battery healt to 100, koster 3000
}

void SendBankAndBatteryInfoToChargingStation(){
    Må sende banksaldoen og batterinivået til ladestasjonen. 
}