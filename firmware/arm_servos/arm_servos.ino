// Robotic arm servo firmware (Arduino Uno / Nano)
//
// Receives lines from the ROS 2 serial_bridge node over USB serial:
//
//   <a1,a2,a3,a4,ms>
//
//   a1..a4  target angles in degrees (0-180) for servo 1..4 (joint1..joint4)
//   ms      time in milliseconds to reach the targets
//
// The servos move smoothly from their current angle to the target over ms.
// After each valid command the board replies "OK a1,a2,a3,a4".

#include <Servo.h>

const int NUM_SERVOS = 4;
const int SERVO_PINS[NUM_SERVOS] = {3, 5, 6, 9};  // joint1, joint2, joint3, joint4
const long BAUD = 115200;

Servo servos[NUM_SERVOS];

float startDeg[NUM_SERVOS];
float currentDeg[NUM_SERVOS];
int targetDeg[NUM_SERVOS];
unsigned long moveStart = 0;
unsigned long moveTime = 0;

char buf[48];
int bufLen = 0;
bool inMessage = false;

void setup() {
  Serial.begin(BAUD);
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    currentDeg[i] = startDeg[i] = targetDeg[i] = 90;  // arm straight up
    servos[i].write(90);
  }
  Serial.println("READY");
}

// Parses "a1,a2,a3,a4,ms" (without the < >). Returns true on success.
bool parseMessage(char *msg) {
  long values[NUM_SERVOS + 1];
  int count = 0;
  char *token = strtok(msg, ",");
  while (token != NULL && count < NUM_SERVOS + 1) {
    values[count++] = atol(token);
    token = strtok(NULL, ",");
  }
  if (count != NUM_SERVOS + 1) {
    return false;
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    startDeg[i] = currentDeg[i];
    targetDeg[i] = constrain(values[i], 0, 180);
  }
  moveTime = max(values[NUM_SERVOS], 0L);
  moveStart = millis();

  Serial.print("OK ");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print(targetDeg[i]);
    Serial.print(i < NUM_SERVOS - 1 ? "," : "\n");
  }
  return true;
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '<') {
      inMessage = true;
      bufLen = 0;
    } else if (c == '>' && inMessage) {
      buf[bufLen] = '\0';
      inMessage = false;
      if (!parseMessage(buf)) {
        Serial.println("ERR bad message");
      }
    } else if (inMessage) {
      if (bufLen < (int)sizeof(buf) - 1) {
        buf[bufLen++] = c;
      } else {
        inMessage = false;  // too long, drop it
        Serial.println("ERR too long");
      }
    }
  }
}

void updateServos() {
  float a = 1.0;
  if (moveTime > 0) {
    a = (float)(millis() - moveStart) / moveTime;
    if (a > 1.0) a = 1.0;
  }
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentDeg[i] = startDeg[i] + a * (targetDeg[i] - startDeg[i]);
    servos[i].write((int)(currentDeg[i] + 0.5));
  }
}

void loop() {
  readSerial();
  updateServos();
  delay(10);
}
