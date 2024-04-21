#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>

// Replace the next variables with your SSID/Password combination
const char* ssid = "NTNU-IOT";
const char* password = "";
//ko

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "10.25.18.202"; // rpi ip adresse

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BMP280 bmp;  // I2C
long lastMsg = 0;
char msg[50];
int value = 0;

//variablelr til å sende
float temperature = 0;
float pressure = 0;
char carID[8] = "0001";  // idNummer bil nummer. må bruke char[] format

// LED Pin
const int ledPin = 4;

void setup_wifi() {
  delay(10);
  // kolbe på wifi
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) { // viser prikker til kobling er god
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the
  // message is either "on" or "off". Changes the output state according to the
  // message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    } else if (messageTemp == "off") {
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(9600);
  // default settings
  //sjekker at sensor er riktig koblet. trengs ikke dersom en ikke bruker bmp sensor
  unsigned status;
  status = bmp.begin();
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1)
      ;
  }
  //setter opp mqtt og wifi
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

//sender melding hvert 5. sek
  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    // Temperature in Celsius
    temperature = bmp.readTemperature()/100;
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    client.publish("esp32/temperature", tempString);
    
     pressure = bmp.readPressure()/100;
     // Convert the value to a char array
     char pressString[8]; 
     dtostrf(pressure, 1, 2, pressString);
     Serial.print("Pressure: ");
     Serial.println(pressString);
     client.publish("esp32/pressure", pressString);
 

     //send carID
     Serial.print("carID: ");
     Serial.println(carID);
     client.publish("esp32/ID-number", carID);
 
  }
}