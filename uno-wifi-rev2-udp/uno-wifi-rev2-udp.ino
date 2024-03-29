/*
 * This sketch provides basic WiFi communication using UDP packets.
 * a) receives a UDP packet on a specified port.
 * b) sends a return UDP packet on the same port.
 * IMPORTANT: Both the sending and receiving hosts must use the same port number!!
 * 
 * FUNCTIONALITY:
 * This program can communicate via UDP with the following Unity project:
 * https://github.com/fudro/unity-robot-interface/tree/master/udp-communication
 * 
 * HARDWARE:
 * Arduino Uno WiFi Rev2
 * 
 * REFERENCE:
 * https://www.arduino.cc/en/Tutorial/WiFiNINAWiFiUdpSendReceiveString
 */


#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h"  //please enter your sensitive data in the Secret tab/arduino_secrets.h

WiFiUDP Udp;

int status = WL_IDLE_STATUS;
unsigned int localPort = 5555;      // local port to listen to
char packetBuffer[255];   //buffer to hold incoming packet
char ReplyBuffer[] = "acknowledged";       // a string to send back as a response
int ledPin = 25;    //set pin for built-in LED of Uno Wifi Rev2 board

void setup() {
  pinMode(ledPin, OUTPUT);
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  //check firmware version
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network. NOTE: This can take about 30 seconds.
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  Udp.begin(localPort);
  Serial.println("\nWaiting for UDP packets...");

  digitalWrite (ledPin, HIGH);    //turn on LED to indicate that the board is ready
}

void loop() {

  //Receive packet. Display packet information.
  int packetSize = Udp.parsePacket();   //Get the size of the packet in bytes (1 byte = 1 octet)
  if (packetSize) {
    Serial.println("");
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From: ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.println(remoteIp);
    Serial.print("On port: ");
    Serial.println(Udp.remotePort());

    // Read packet. Store contents into packetBufffer. Store the number of bytes read as the variable 'len'
    int len = Udp.read(packetBuffer, 255);  //Read and store into packetBuffer up to 255 bytes (1 byte = 1 octet)
    if (len > 0) {
      packetBuffer[len] = 0;  //Set last character of the packetBuffer to zero to create a null terminated string (C string) - Not sure why/if this is necessary
    }
    Serial.print("Contents: ");
    Serial.println(packetBuffer);

    //Compare packet. Check if the packetBuffer is equal to a desired string. Perform different commands depending on the string that was received.
    String myString = (char*)packetBuffer;
    Serial.println("PacketBuffer as String: " + myString);

    /*  MESSAGE OPTIONS:
     * ****************************
     */
    if (myString.startsWith("hello")) {
      digitalWrite (ledPin, HIGH);
    }
    else if (myString.startsWith("bye")) {
      digitalWrite (ledPin, LOW);
    }
    /* 
     * ****************************
     */

    //Send a reply
    //Use the IP address retrieved from the sent packet.
    //Use the previously defined port for both sender and receiver. DO NOT use the port retrieved from the packet!!
    Udp.beginPacket(Udp.remoteIP(), localPort);
    Udp.write(ReplyBuffer);
    Udp.endPacket();

    Serial.println("");
    Serial.print("Reply Message: '");
    Serial.print(ReplyBuffer);
    Serial.println("'");
    Serial.print("Sent to: ");
    Serial.println(Udp.remoteIP());
    Serial.print("On port: ");
    Serial.println(localPort);
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
