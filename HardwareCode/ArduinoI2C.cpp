#include <Wire.h>

#define CMD_STOP       0x00
#define CMD_FORWARD    0x01
#define CMD_BACKWARD   0x02
#define CMD_TURN_LEFT  0x03
#define CMD_TURN_RIGHT 0x04
#define CMD_SET_SERVO  0x05
#define CMD_PING       0xFF

void setup() {
  Wire.begin(0x08); // I2C address
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.begin(9600);
}

uint8_t lastCommand = 0;

void loop() {
  delay(10);
}

void receiveEvent(int numBytes) {
  if (numBytes < 1) return;
  uint8_t cmd = Wire.read();
  lastCommand = cmd;

  switch (cmd) {
    case CMD_STOP:
      stopMotors();
      break;
    case CMD_FORWARD:
      if (Wire.available()) {
        uint8_t speed = Wire.read();
        moveForward(speed);
      }
      break;
    case CMD_BACKWARD:
      if (Wire.available()) {
        uint8_t speed = Wire.read();
        moveBackward(speed);
      }
      break;
    case CMD_TURN_LEFT:
    case CMD_TURN_RIGHT:
      if (Wire.available()) {
        uint8_t angle = Wire.read();
        if (cmd == CMD_TURN_LEFT) turnLeft(angle);
        else turnRight(angle);
      }
      break;
    case CMD_SET_SERVO:
      if (Wire.available() >= 2) {
        uint8_t id = Wire.read();
        uint8_t angle = Wire.read();
        setServo(id, angle);
      }
      break;
  }
}

void requestEvent() {
  if (lastCommand == CMD_PING) {
    Wire.write(0xAC);  // Arbitrary ACK byte
  }
}

// Dummy function placeholders
void stopMotors() { Serial.println("Stop"); }
void moveForward(uint8_t speed) { Serial.print("Forward "); Serial.println(speed); }
void moveBackward(uint8_t speed) { Serial.print("Backward "); Serial.println(speed); }
void turnLeft(uint8_t angle) { Serial.print("Left "); Serial.println(angle); }
void turnRight(uint8_t angle) { Serial.print("Right "); Serial.println(angle); }
void setServo(uint8_t id, uint8_t angle) { Serial.print("Servo "); Serial.print(id); Serial.print(" to "); Serial.println(angle); }
