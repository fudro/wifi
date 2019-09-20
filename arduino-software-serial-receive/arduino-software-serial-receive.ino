#include <SoftwareSerial.h>
SoftwareSerial mySerial(5, 10); //RX, TX

void setup() {
  Serial.begin(19200);
  mySerial.begin(19200);
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
