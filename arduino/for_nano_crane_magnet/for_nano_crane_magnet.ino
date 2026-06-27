#include <SoftwareSerial.h>

SoftwareSerial masterLine(10, 11);

const int magPin1 = 2;
const int magPin2 = 3;

void setup() {
  pinMode(magPin1, OUTPUT);
  pinMode(magPin2, OUTPUT);

  digitalWrite(magPin1, LOW);
  digitalWrite(magPin2, LOW);

  Serial.begin(9600);
  masterLine.begin(9600);
}

void loop() {
  if (masterLine.available()) {
    char cmd = masterLine.read();

    if (cmd == '1') {
      digitalWrite(magPin1, HIGH);
      digitalWrite(magPin2, LOW);
    }
    else if (cmd == '0') {
      digitalWrite(magPin1, LOW);
      digitalWrite(magPin2, LOW);
    }
  }
}
