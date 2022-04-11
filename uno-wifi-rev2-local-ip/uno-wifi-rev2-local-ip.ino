/*
 * DESCRIPTION:
 * This program will connnect to the provided network and display 
 * the local IP address of the Arduino microcontroller to the Serial Monitor.
 * 
 * HARDWARE:
 * Arduino Uno WiFi Rev 2 
 * https://store.arduino.cc/usa/arduino-uno-wifi-rev2
 * 
 * REFERENCE:
 * https://www.arduino.cc/en/Guide/ArduinoUnoWiFiRev2#toc13
 */

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
