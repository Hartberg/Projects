// Kommunikajson mellom 2 esp
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

const int sensorPin = 32;

// unsigned int(for å få positive og negative verdier) av typen 8 bit/ 1 byte som array
uint8_t broadcastAddress[]= {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcast høyeste verdien (255,255,255,255,255,255)

float outgoingSensorData; //Lagrer data som vi logger fra sensorPin

//data vi ønsker å få inn fra de andre ESP-ene
float incomingSensorData;
char* incomingMsg; //Char array/string
float incomingNum;

String success; // Om meldingen har sent eller ikke sendt

// Si til esp-biblioteket hvilken data den kan forvente å motta
typedef struct struct_message{// lager en struct- flere forskjellige variabeltyper.
  float sensorData;
  char* msg1;
  float num1;
} struct_message;

// Deklarerer ny struct, samme som å lage ny int nå som vi har laget struct
struct_message outgoingData;
struct_message incomingData;

esp_now_peer_info_t peerInfo; // Holder infor om de forskjellige esp-ene en kan snakke med


//Lage en funksjon som kjører hver gang data blir forsøkt sendt, om det ble sendt riktig eller ikke
void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status){ //* gjør det til et array, må ha den med
  Serial.println("Last packet send statud:");
  //if(status == ESP_NOW_SEND_SUCCESS) //Innebygd variabel til esp-biblioteket

  if (status == 0){
    success = "Delivery success :)";
  } else {
    success = "Delivery fail :(";
  }
}

// det som skjer etter vi har mottat/ forsøkt å motta data. mac adressen tilhører de vi mottar data fra
void onDataRecv(const uint8_t* mac, const uint8_t* callbackData, int len) { // ledn = lengden v data mottatt. callbackData er innkommende data i rådata/array av bytes
  // Vil konvertere callbackData til en struct
  memcpy(&incomingData, callbackData, sizeof(incomingData)); // & refererer til minneadressen, der data skal lagres
  Serial.print("Bytes recieved: ");
  Serial.println(len);

  // Vil sette de globale variablene
  incomingSensorData = incomingData.sensorData;
  incomingMsg = incomingData.msg1;
  incomingNum = incomingData.num1;

  Serial.println(incomingSensorData);
  Serial.println(incomingMsg);
  Serial.println(incomingNum);
}

void setup() {

  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
  WiFi.mode(WIFI_STA);

  if(esp_now_init() != ESP_OK){
    Serial.println("ERROR init ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSent);


  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if(esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
}

void loop() {

  // Sette struct for outgoing data
  outgoingData.sensorData = analogRead(sensorPin);
  outgoingData.msg1 = "Hartberg";
  outgoingData.num1 = 420;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));
  

  if (result == ESP_OK) {
    Serial.println("Sent successfully!");
  } else {
    Serial.println("Error sending data");
  }

  delay(5000);
}