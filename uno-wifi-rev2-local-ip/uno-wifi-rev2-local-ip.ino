#include <WiFiNINA.h>

char ssid[] = "NETGEAR12";      //SSID of your network
char pass[] = "smilinghat005";      //SSID of your network

void setup(){
  // initialize serial:
  Serial.begin(9600);
  
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
  }
  // Once connected, print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop () {}
