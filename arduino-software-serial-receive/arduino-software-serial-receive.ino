#include <SoftwareSerial.h>
SoftwareSerial mySerial(5, 10); //RX, TX

void setup() {
  Serial.begin(19200);
  mySerial.begin(19200);
  delay(5000);

//  pinMode (6, OUTPUT);
//  pinMode (7, OUTPUT);
//  pinMode (8, OUTPUT);
//  pinMode (9, OUTPUT);
//  digitalWrite (6, HIGH);
//  digitalWrite (7, HIGH);
//  digitalWrite (8, HIGH);
//  digitalWrite (9, HIGH);
}

void loop() {
  String IncomingString = "";
  boolean StringReady = false;
  while (mySerial.available()) {
    IncomingString = mySerial.readString();
    StringReady = true;
  }
  if (StringReady) {
//      digitalWrite (6, HIGH);
//      digitalWrite (7, HIGH);
//      digitalWrite (8, HIGH);
//      digitalWrite (9, HIGH);
//
//      switch (IncomingString.toInt()) {
//      case 1:
//        digitalWrite (6, LOW);
//      break;
//      case 2:
//        digitalWrite (7, LOW);
//      break;
//      case 3:
//        digitalWrite (8, LOW);
//      break;
//      case 4:
//        digitalWrite (9, LOW);
//      break;
//      default:
//
//      break;
//    }
    Serial.println("Received String: ");
    Serial.print(IncomingString);
  }
}
