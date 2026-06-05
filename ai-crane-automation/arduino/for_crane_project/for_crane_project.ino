#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial nanoLine(A4, A5);

const int LIMIT_TROLLEY = 200;
const int LIMIT_BODY = 130;
const int LIMIT_HOOK = 255;

const int bodyPin1 = 11;
const int bodyPin2 = 12;
const int bodyPWM = 3;

const int hookPin1 = 8;
const int hookPin2 = 9;
const int hookPWM = 6;

const int trolleyPin1 = 5;
const int trolleyPin2 = 4;
const int trolleyPWM = 10;

const int joyTrolleyPin = A0;
const int joyHookPin = A1;
const int joyBodyPin = A2;
const int deadZone = 80;

bool aiActive = false;
unsigned long aiTimer = 0;
unsigned long aiDuration = 0;

const byte numChars = 64;
char receivedChars[numChars];
boolean newData = false;

void setup() {
  Serial.begin(115200);
  nanoLine.begin(9600);

  pinMode(bodyPin1, OUTPUT); pinMode(bodyPin2, OUTPUT); pinMode(bodyPWM, OUTPUT);
  pinMode(hookPin1, OUTPUT); pinMode(hookPin2, OUTPUT); pinMode(hookPWM, OUTPUT);
  pinMode(trolleyPin1, OUTPUT); pinMode(trolleyPin2, OUTPUT); pinMode(trolleyPWM, OUTPUT);

  Serial.println("<Ready>");
}

void loop() {
  recvWithStartEndMarkers();

  if (newData) {
    parseData();
    newData = false;
  }

  if (aiActive) {
    if (millis() - aiTimer >= aiDuration) {
      stopAll();
      aiActive = false;
      Serial.println("<DONE>");
    }
  } else {
    readJoystick();
  }
}

void readJoystick() {
  int joyBody = analogRead(joyBodyPin) - 512;
  int joyHook = analogRead(joyHookPin) - 512;
  int joyTrolley = analogRead(joyTrolleyPin) - 512;

  int speedBody = abs(joyBody) > deadZone ? map(abs(joyBody), 0, 512, 0, LIMIT_BODY) : 0;
  if (joyBody < 0) speedBody = -speedBody;

  int speedHook = abs(joyHook) > deadZone ? map(abs(joyHook), 0, 512, 0, LIMIT_HOOK) : 0;
  if (joyHook > 0) speedHook = -speedHook;

  int speedTrolley = abs(joyTrolley) > deadZone ? map(abs(joyTrolley), 0, 512, 0, LIMIT_TROLLEY) : 0;
  if (joyTrolley > 0) speedTrolley = -speedTrolley;

  driveMotor(bodyPin1, bodyPin2, bodyPWM, speedBody);
  driveMotor(hookPin1, hookPin2, hookPWM, speedHook);
  driveMotor(trolleyPin1, trolleyPin2, trolleyPWM, speedTrolley);
}

void parseData() {
  char *strtokIndx;
  strtokIndx = strtok(receivedChars, ",");

  if (strcmp(strtokIndx, "J") == 0) {
    int rot = atoi(strtok(NULL, ","));
    int hook = atoi(strtok(NULL, ","));
    int trol = atoi(strtok(NULL, ","));
    long duration = atol(strtok(NULL, ","));

    driveMotor(bodyPin1, bodyPin2, bodyPWM, constrain(rot, -LIMIT_BODY, LIMIT_BODY));
    driveMotor(hookPin1, hookPin2, hookPWM, constrain(hook, -LIMIT_HOOK, LIMIT_HOOK));
    driveMotor(trolleyPin1, trolleyPin2, trolleyPWM, constrain(trol, -LIMIT_TROLLEY, LIMIT_TROLLEY));

    aiActive = true;
    aiTimer = millis();
    aiDuration = duration;
    Serial.println("<ACK>");
  }

  else if (strcmp(strtokIndx, "M") == 0) {
    int magState = atoi(strtok(NULL, ","));

    if (magState == 1) {
      nanoLine.print('1');
      Serial.println("<MAGNET_ON>");
    } else {
      nanoLine.print('0');
      Serial.println("<MAGNET_OFF>");
    }
  }

  else if (strcmp(strtokIndx, "s") == 0 || strcmp(strtokIndx, "S") == 0) {
    stopAll();
    aiActive = false;
    Serial.println("<STOPPED>");
  }
}

void driveMotor(int p1, int p2, int pwmPin, int speed) {
  if (speed == 0) {
    digitalWrite(p1, LOW);
    digitalWrite(p2, LOW);
    analogWrite(pwmPin, 0);
  } else if (speed > 0) {
    digitalWrite(p1, HIGH);
    digitalWrite(p2, LOW);
    analogWrite(pwmPin, speed);
  } else {
    digitalWrite(p1, LOW);
    digitalWrite(p2, HIGH);
    analogWrite(pwmPin, abs(speed));
  }
}

void stopAll() {
  driveMotor(bodyPin1, bodyPin2, bodyPWM, 0);
  driveMotor(hookPin1, hookPin2, hookPWM, 0);
  driveMotor(trolleyPin1, trolleyPin2, trolleyPWM, 0);

  nanoLine.print('0');
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) ndx = numChars - 1;
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}
