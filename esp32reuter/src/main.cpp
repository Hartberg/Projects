#include <WiFi.h>
#include <WebServer.h>

// SSID & Password
const char *ssid = "Hartbergesp32";      // Enter your SSID here
const char *password = "IkkeBrukMeg119"; // Enter your Password here

// IP Address details
IPAddress local_ip(192, 168, 1, 2);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80); // Object of WebServer(HTTP port, 80 is defult)
// HTML & CSS contents which display on web server
String HTML = "<!DOCTYPE html>\
<html>\
<body>\
<h1>My First Web Server with ESP32 - AP Mode &#128522;</h1>\
</body>\
</html>";

// Handle root url (/)
void handle_root()
{
  server.send(200, "text/html", HTML);
}

void setup()
{
  Serial.begin(9600);

  // Create SoftAP
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  Serial.print("Connect to My access point: ");
  Serial.println(ssid);
  Serial.println("IP address: ");
  Serial.println(local_ip);

  server.on("/", handle_root);

  server.begin();
  Serial.println("HTTP server started");
  delay(100);
}

void loop()
{
  server.handleClient();
}


