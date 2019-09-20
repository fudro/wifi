/*********************************************
 * This program demonstrates receiving serial data using AltSoftSerial. https://github.com/PaulStoffregen/AltSoftSerial
 * AltSoftSerial is supposed to be more reliable at higher baud rates.
 * AltSoftSerial ALWAYS uses these pins:

 Board              Transmit  Receive   PWM Unusable
 -----              --------  -------   ------------
 Arduino Uno/Nano        9         8         10
 
 **********************************************/

#include <AltSoftSerial.h>


AltSoftSerial mySerial(8, 9); //RX, TX

void setup() {
  Serial.begin(74880);    //IMPORTANT: In this example program, AltSoftSerial is reliable up to max baud rate of 74880.
  mySerial.begin(74880);
  delay(5000);

  //Create ouput pins to demonstrate wifi controlled output.
  pinMode (6, OUTPUT);
  pinMode (7, OUTPUT);
  digitalWrite (6, HIGH);   //Turn outputs ON
  digitalWrite (7, HIGH);
}

void loop() {
  String IncomingString = "";
  boolean StringReady = false;
  while (mySerial.available()) {
    IncomingString = mySerial.readString();
    StringReady = true;
  }
  if (StringReady) {    //Whenever serial data has been received, turn outputs OFF for 5 seconds.
      digitalWrite (6, LOW);
      digitalWrite (7, LOW);
      delay (5000);
      digitalWrite (6, HIGH);
      digitalWrite (7, HIGH);
    Serial.println("Received String: ");
    Serial.print(IncomingString);
  }
}
