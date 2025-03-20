#include "comm.hpp"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <stdlib.h>
#include "odometry.hpp"
#include "Macros.h"
// put function declarations here:
// int myFunction(int, int);

JsonDocument doc;
const char *test =
    "{\"Command\":\"Drive\", \"data\":[2,127,127,127], \"run\": true }";
// DeserializationError error = deserializeJson(doc, test);

const int ledPin = 13;

// Drivetrain Macros
#define FL_PIN_1 MD3_M2_PIN_2
#define FL_PIN_2 MD3_M2_PIN_1

#define BL_PIN_1 MD4_M2_PIN_1
#define BL_PIN_2 MD4_M2_PIN_2

#define BR_PIN_1 MD4_M1_PIN_2
#define BR_PIN_2 MD4_M1_PIN_1

#define FR_PIN_1 MD3_M1_PIN_1
#define FR_PIN_2 MD3_M1_PIN_2

// Encoder Macros
#define EL_PIN_1 Encoder_2_PIN_1
#define EL_PIN_2 Encoder_2_PIN_2

#define ER_PIN_1 Encoder_1_PIN_1
#define ER_PIN_2 Encoder_1_PIN_2

#define EC_PIN_1 Encoder_3_PIN_1
#define EC_PIN_2 Encoder_3_PIN_2


MotorControl FR = MotorControl(FR_PIN_1, FR_PIN_1);
MotorControl FL = MotorControl(FL_PIN_1, FL_PIN_2);
MotorControl BR = MotorControl(BR_PIN_1, BR_PIN_2);
MotorControl BL = MotorControl(BL_PIN_1, BL_PIN_2);

RotaryEncoder ENCODER1(EL_PIN_1, EL_PIN_2, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder ENCODER2(ER_PIN_1, ER_PIN_2, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder ENCODER3(EC_PIN_1, EC_PIN_2, RotaryEncoder::LatchMode::TWO03);

Odometry odom = {&ENCODER1, &ENCODER2, &ENCODER3};

devices activeDevices = {&FL, &FR, &BL, &BR};

jetsonComms jet(activeDevices);

bool run = true;

bool bin_intake = false;

JsonDocument docTest;
void updateLeftEncoder() {
  ENCODER1.tick();
}

void updateRightEncoder(){
  ENCODER2.tick();
}

void updateCenterEncoder() {
  ENCODER3.tick();
}

void setup() {
  Serial8.begin(115200);

  FR.Motor_enablePIDMode(false);
  FL.Motor_enablePIDMode(false);
  BR.Motor_enablePIDMode(false);
  BL.Motor_enablePIDMode(false);

  attachInterrupt(digitalPinToInterrupt(EL_PIN_1), updateLeftEncoder, arduino::CHANGE);
  attachInterrupt(digitalPinToInterrupt(EL_PIN_2), updateLeftEncoder, arduino::CHANGE);

  attachInterrupt(digitalPinToInterrupt(ER_PIN_1), updateRightEncoder, arduino::CHANGE);
  attachInterrupt(digitalPinToInterrupt(ER_PIN_2), updateRightEncoder, arduino::CHANGE);

  attachInterrupt(digitalPinToInterrupt(EC_PIN_1), updateCenterEncoder, arduino::CHANGE);
  attachInterrupt(digitalPinToInterrupt(EC_PIN_2), updateCenterEncoder, arduino::CHANGE);

  pinMode(ledPin, arduino::OUTPUT);
  for(int i = 0; i < 10; i++){
    digitalWrite(ledPin, arduino::LOW);   // turn the LED off by making the voltage LOW
    delay(200);                                // wait for a second
    digitalWrite(ledPin, arduino::HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(200);                                // wait for a second
  }
}

void loop() {

  if (Serial8.available())
  {
    digitalWrite(ledPin, arduino::HIGH);

    deserializeJson(doc, Serial8);

    int msg_type = doc["header"]["message_type"];

    Serial8.flush();

    switch (msg_type)
    {
      case 0:
        doc.clear();
        doc["start_led"] = 0;
        doc["deadwheel_stats"]["encoder_left"] = (odom.Encoder1)->getPosition();
        doc["deadwheel_stats"]["encoder_right"] = (odom.Encoder2)->getPosition();
        doc["deadwheel_stats"]["encoder_center"] = (odom.Encoder3)->getPosition();
        doc["deadwheel_stats"]["heading"] = 1.0;
        serializeJson(doc, Serial8);
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

    digitalWrite(ledPin, arduino::LOW);
  }

  // UPDATE MOTOR POSITIONS WITH ENCODERS HERE:
  FL.Motor_update();
  FR.Motor_update();
  BL.Motor_update();
  BR.Motor_update();
}
