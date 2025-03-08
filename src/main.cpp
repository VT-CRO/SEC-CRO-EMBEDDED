#include "comm.hpp"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <RotaryEncoder.h>
#include <stdlib.h>
// put function declarations here:
// int myFunction(int, int);

JsonDocument doc;
const char *test =
    "{\"Command\":\"Drive\", \"data\":[2,127,127,127], \"run\": true }";
// DeserializationError error = deserializeJson(doc, test);

const int ledPin = 13;

// Motor Macros
#define FR_PIN_1 1 // 1
#define FR_PIN_2 0 // 2

#define FL_PIN_1 3
#define FL_PIN_2 2

#define BR_PIN_1 23
#define BR_PIN_2 22

#define BL_PIN_1 19
#define BL_PIN_2 18

#define EL_PIN_1 28
#define EL_PIN_2 29

#define ER_PIN_1 6
#define ER_PIN_2 7

#define EC_PIN_1 17
#define EC_PIN_2 16

#define COUNTS_PER_REV 64
#define GEAR_RATIO 100

MotorControl FR = MotorControl(FR_PIN_1, FR_PIN_1);
MotorControl FL = MotorControl(FL_PIN_1, FL_PIN_2);
MotorControl BR = MotorControl(BR_PIN_1, BR_PIN_2);
MotorControl BL = MotorControl(BL_PIN_1, BL_PIN_2);

RotaryEncoder Encoder_Left =
    RotaryEncoder(EL_PIN_1, EL_PIN_2, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder Encoder_Right =
    RotaryEncoder(ER_PIN_1, ER_PIN_2, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder Encoder_Center =
    RotaryEncoder(EC_PIN_1, EC_PIN_2, RotaryEncoder::LatchMode::TWO03);

devices activeDevices = {&FL, &FR, &BL, &BR};

jetsonComms jet(activeDevices);

bool run = true;

bool bin_intake = false;

JsonDocument docTest;

void checkPositionL() {
  Encoder_Left.tick(); // just call tick() to check the state.
}

void checkPositionR() {
  Encoder_Right.tick(); // just call tick() to check the state.
}

void checkPositionC() {
  Encoder_Center.tick(); // just call tick() to check the state.
}

void setup() {
  Serial3.begin(115200);

  FR.Motor_enablePIDMode(false);
  FL.Motor_enablePIDMode(false);
  BR.Motor_enablePIDMode(false);
  BL.Motor_enablePIDMode(false);

  attachInterrupt(digitalPinToInterrupt(EL_PIN_1), checkPositionL,
                  arduino::CHANGE);
  attachInterrupt(digitalPinToInterrupt(EL_PIN_2), checkPositionL,
                  arduino::CHANGE);

  attachInterrupt(digitalPinToInterrupt(ER_PIN_1), checkPositionR,
                  arduino::CHANGE);
  attachInterrupt(digitalPinToInterrupt(ER_PIN_2), checkPositionR,
                  arduino::CHANGE);

  attachInterrupt(digitalPinToInterrupt(EC_PIN_1), checkPositionC,
                  arduino::CHANGE);
  attachInterrupt(digitalPinToInterrupt(EC_PIN_2), checkPositionC,
                  arduino::CHANGE);

  // pinMode(ledPin, arduino::OUTPUT);
}

void loop() {

  if (Serial3.available()) {
    // digitalWrite(ledPin, arduino::HIGH);

    deserializeJson(doc, Serial3);

    int msg_type = doc["header"]["message_type"];

    Serial3.flush();

    switch (msg_type) {
    case 0:
      doc.clear();
      doc["start_led"] = 0;
      doc["deadwheel_stats"]["x"] = Encoder_Left.getPosition();
      doc["deadwheel_stats"]["y"] = Encoder_Right.getPosition();
      doc["deadwheel_stats"]["heading"] = Encoder_Center.getPosition();
      serializeJson(doc, Serial3);
      break;
    case 1:
      run = doc["run"];

      if (run) {
        float fl_speed = doc["motor_speeds"][0];
        float fr_speed = doc["motor_speeds"][1];
        float bl_speed = doc["motor_speeds"][2];
        float br_speed = doc["motor_speeds"][3];
        FL.Motor_setGoalSpeed(fl_speed);
        FR.Motor_setGoalSpeed(fr_speed);
        BL.Motor_setGoalSpeed(bl_speed);
        BR.Motor_setGoalSpeed(br_speed);

        bin_intake = doc["bin_intake"];
      } else {
        FL.Motor_setGoalSpeed(0);
        FR.Motor_setGoalSpeed(0);
        BL.Motor_setGoalSpeed(0);
        BR.Motor_setGoalSpeed(0);
        bin_intake = false;
      }
      break;
    }

    doc.clear();
  }

  // UPDATE MOTOR POSITIONS WITH ENCODERS HERE:
  FL.Motor_update();
  FR.Motor_update();
  BL.Motor_update();
  BR.Motor_update();
}