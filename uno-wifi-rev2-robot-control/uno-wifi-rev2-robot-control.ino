/*
 * DESCRIPTION:
 * This sketch provides basic WiFi control of a robot using UDP packets.
 * Separate commands can be sent to the robot as individual strings.
 * IMPORTANT: Both the sending and receiving hosts must use the same port number!!
 * 
 * FUNCTIONALITY:
 * This program can communicate via UDP with the following Unity project:
 * https://github.com/fudro/unity-robot-interface/tree/master/udp-communication
 * 
 * HARDWARE:
 * Arduino Uno WiFi Rev2
 * SeeedStudio Motorshield V2.0
 * 
 * REFERENCE:
 * https://www.arduino.cc/en/Tutorial/WiFiNINAWiFiUdpSendReceiveString
 */

#include <Wire.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
//#include "MotorDriver.h"
#include "Files\MotorDriver.h"
#include "arduino_secrets.h"

MotorDriver motor;

/***************************
 * UDP OBJECT
 **************************/
WiFiUDP Udp;    //Create the UDP object

/**************************
 * GLOBAL VARIABLES
 *************************/
int status = WL_IDLE_STATUS;
unsigned int localPort = 5555;      // local port to listen to
char packetBuffer[255];   //buffer to hold incoming packet
char ReplyBuffer[] = "acknowledged";       // a string to send back as a response
int ledPin = 25;    //set pin for built-in LED of Uno Wifi Rev2 board


void setup() {  
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //Check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  //Check WiFi module firmware version
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  //Attempt to connect to Wifi network. NOTE: This can take about 30 seconds!
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
  

  //MOTOR CONTROLLER SETUP
  motor.begin();
  //Format: motor.speed([motor], [speed]) 
  motor.speed(0, 0);    //motor speed ranges from 0 to 100
  motor.speed(1, 0);
  motor.brake(0);   //brake both motors.
  motor.brake(1);
  motor.stop(0);    //stop both motors
  motor.stop(1);
  Serial.println("SeeedStudio Motorshield v2 - Ready!");

  //LED FEEDBACK SETUP
  pinMode(ledPin, OUTPUT);    //Set pinMode for visual feedback LED
  digitalWrite (ledPin, HIGH);    //turn on LED to indicate that the board is ready
}

void loop() {
  //Receive packet. Display packet information.
  int packetSize = Udp.parsePacket();   //Get the size of the packet in bytes (1 byte = 1 octet)
  if (packetSize) {
    Serial.print("Received packet of size: ");
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
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    //Compare packet. Check if the packetBuffer is equal to a desired string. Perform different commands depending on the string that was received.
    String myString = (char*)packetBuffer;
    Serial.println("PacketBuffer as String: " + myString);

    /*  MESSAGE OPTIONS:
     * ****************************
     */
    //COMM TEST FUNCTIONS
    //Control built-in LED for basic communication tests.
    if (myString.startsWith("hello")) {
      digitalWrite (ledPin, HIGH);
    }
    else if (myString.startsWith("bye")) {
      digitalWrite (ledPin, LOW);
    }

    //MOTOR CONTROL FUNCTIONS
    //Control all drive functions of the robot chassis.
    else if (myString.startsWith("MOVE_F")) {
      digitalWrite (ledPin, LOW);
      motorTest();
      digitalWrite (ledPin, HIGH);
    }
    else if (myString.startsWith("MOVE_B")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("TURN_L")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("TURN_R")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("SPIN_L")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("SPIN_R")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("STOP")) {
      digitalWrite (ledPin, LOW);
    }

    //CLAW FUNCTIONS
    //Control the rear claw on the chassis base.
    else if (myString.startsWith("CLAW_OPEN")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("CLAW_CLOSE")) {
      digitalWrite (ledPin, LOW);
    }
    //ARM FUNCTIONS
    //Control all arm functions.
    else if (myString.startsWith("POSE_ARM")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("ROTATE_L")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("ROTATE_R")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("LIFT_ARM")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("LOWER_ARM")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("EXTEND_ELBOW")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("RETRACT_ELBOW")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("WRIST_CW")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("WRIST_CCW")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("WRIST_HORIZONTAL")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("WRIST_VERTICAL")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("GRIP_OPEN")) {
      digitalWrite (ledPin, LOW);
    }
    else if (myString.startsWith("GRIP_CLOSE")) {
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

void motorTest() {
  uint8_t i;
  
  Serial.print("Moving Forward!");
  motor.speed(0, 100);    //set motor speed to POSITIVE value to move FORWARD
  motor.speed(1, 100);
  delay(3000);
  motor.brake(0);                 // brake
  motor.brake(1);
  delay(2000);
  
  Serial.print("Moving Backward!");
  motor.speed(0, -100);   // set motor speed to NEGATIVE value to move BACKWARD
  motor.speed(1, -100);
  delay(3000);
  motor.stop(0);    //stop both motors
  motor.stop(1);
  Serial.print("Stopped!");
  delay(2000);
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
