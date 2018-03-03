#include "AeroBridge.h"

const byte startByte = 251;
const byte endByte = 252;

// for the received message
const byte messageLen = 14;
byte message[messageLen];

bool messageWaiting = false;
bool messageInProgress = false;
byte messageIndex = 0;


QUBEAero qube; 

void setup() {
  // code is run once on setup
  Serial.begin(115200); // for debugging port, be sure to set the Serial Monitor to the correct baud rate
  qube.reset();

  float pitch = qube.get_pitch();
  float yaw = qube.get_yaw();
  sendMessage(pitch, yaw);
}

void recvMessage() {
  byte part;
  while (Serial.available() > 0 && !messageWaiting) {
    part = Serial.read();
    if (messageInProgress) {
      if (messageIndex < messageLen) {
        message[messageIndex] = part;
        messageIndex++;
      } else if (part == endByte) {
        messageInProgress = false;
        messageWaiting = true;
        messageIndex = 0;
      }
    } else if (part == startByte) {
      messageInProgress = true;
    }
  }
}

void sendMessage(float pitch, float yaw) {
    union {
        float value;
        byte bytes[4];
    } unpackFloat;

    const byte responseLen = 10;
    byte response[responseLen];

    // start
    response[0] = startByte;
    // pitch
    unpackFloat.value = pitch;
    response[1] = unpackFloat.bytes[0];
    response[2] = unpackFloat.bytes[1];
    response[3] = unpackFloat.bytes[2];
    response[4] = unpackFloat.bytes[3];
    // yaw
    unpackFloat.value = yaw;
    response[5] = unpackFloat.bytes[0];
    response[6] = unpackFloat.bytes[1];
    response[7] = unpackFloat.bytes[2];
    response[8] = unpackFloat.bytes[3];
    // end
    response[9] = endByte;

    Serial.write(response, responseLen);
}

void loop() {
    if (messageWaiting) { //unpack message
        float motor0Voltage;
        float motor1Voltage;
        int LEDRed;
        int LEDGreen;
        int LEDBlue;

        union {
            float value;
            byte bytes[4];
        } unpackFloat;

        union {
            int value;
            byte bytes[2];
        } unpackInt;

        // motor0Voltage
        unpackFloat.bytes[0] = message[0];
        unpackFloat.bytes[1] = message[1];
        unpackFloat.bytes[2] = message[2];
        unpackFloat.bytes[3] = message[3];
        motor0Voltage = unpackFloat.value;

        // motor1Voltage
        unpackFloat.bytes[0] = message[4];
        unpackFloat.bytes[1] = message[5];
        unpackFloat.bytes[2] = message[6];
        unpackFloat.bytes[3] = message[7];
        motor1Voltage = unpackFloat.value;

        // LEDRed
        unpackInt.bytes[0] = message[8];
        unpackInt.bytes[1] = message[9];
        LEDRed = unpackInt.value;

        // LEDGreen
        unpackInt.bytes[0] = message[10];
        unpackInt.bytes[1] = message[11];
        LEDGreen = unpackInt.value;

        // LEDBlue
        unpackInt.bytes[0] = message[12];
        unpackInt.bytes[1] = message[13];
        LEDBlue = unpackInt.value;

        // update qube
        qube.update(motor0Voltage, motor1Voltage, LEDRed, LEDGreen, LEDBlue);

        // send message
        float pitch = qube.get_pitch();
        float yaw = qube.get_yaw();
        sendMessage(pitch, yaw);
        messageWaiting = false;
    } else {
        recvMessage();
    }
}

