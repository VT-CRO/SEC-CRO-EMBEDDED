#include "comm.hpp"
#include <Arduino.h>
#include <ArduinoJson.h>
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

MotorControl FR = MotorControl(FR_PIN_1, FR_PIN_1);
MotorControl FL = MotorControl(FL_PIN_1, FL_PIN_2);
MotorControl BR = MotorControl(BR_PIN_1, BR_PIN_2);
MotorControl BL = MotorControl(BL_PIN_1, BL_PIN_2);

devices activeDevices = {&FL, &FR, &BL, &BR};

jetsonComms jet(activeDevices);

bool run = true;

bool bin_intake = false;

JsonDocument docTest;

void setup() {
  Serial3.begin(115200);

  FR.Motor_enablePIDMode(false);
  FL.Motor_enablePIDMode(false);
  BR.Motor_enablePIDMode(false);
  BL.Motor_enablePIDMode(false);

  // pinMode(ledPin, arduino::OUTPUT);
  // digitalWrite(ledPin, arduino::HIGH); // set the LED on
  // doc["start_led"] = 0;
  // doc["deadwheel_stats"]["x"] = 6;
  // doc["deadwheel_stats"]["y"] = 100;
  // doc["deadwheel_stats"]["heading"] = 1.0;

  // serializeJson(doc, Serial3);
  // doc.clear();

  // delay(30);
}

int state = 0;

void loop() {

  switch (state) {
  case 0:
    if (Serial3.available() > 0) {
      state = 1;
    }
    Serial3.write("HII!\n");
    break;
  case 1:
    if (Serial3.available()) {
      deserializeJson(doc, Serial3);
      Serial3.flush();

      doc.clear();

      Serial3.write("HIII!\n");
    }
    break;
  }

  // jet.execute(test);

  // if (Serial3.available()) {

  //   // serializeJson(doc, Serial);

  //   run = doc["run"];

  //   // if (!run) //{

  //   FL.Motor_setGoalSpeed(atof(doc["motor_speeds"][0]));
  //   FR.Motor_setGoalSpeed(atof(doc["motor_speeds"][1]));
  //   BL.Motor_setGoalSpeed(atof(doc["motor_speeds"][2]));
  //   BR.Motor_setGoalSpeed(atof(doc["motor_speeds"][3]));

  //   bin_intake = doc["bin_intake"];
  //   // } else {
  //   //   FL.Motor_setGoalSpeed(0);
  //   //   FR.Motor_setGoalSpeed(0);
  //   //   BL.Motor_setGoalSpeed(0);
  //   //   BR.Motor_setGoalSpeed(0);
  //   //   bin_intake = false;
  //   // }
  //   delay(30);
  //   doc.clear();

  //   // TO-DO change this to turn on when the start LED is on and give actual
  //   // deadwheel odo info Sending status
  // }

  // // UPDATE MOTOR POSITIONS WITH ENCODERS HERE:
  // // FL.Moto_setPosition();

  // // FL.Motor_update();
  // // FR.Motor_update();
  // // BL.Motor_update();
  // // BR.Motor_update();

  // doc["start_led"] = 0;
  // doc["deadwheel_stats"]["x"] = 6;
  // doc["deadwheel_stats"]["y"] = 100;
  // doc["deadwheel_stats"]["heading"] = 1.0;

  // serializeJson(doc, Serial3);
  // // serial3.wrtie

  // doc.clear();
  // delay(30);

  // if (!docTest.isNull()) {
  //   int val = docTest["data"][0];
  //   Serial3.println(val);
  //   docTest.clear();
  //   docTest["Status"] = "Good";
  //   serializeJson(docTest, Serial3);
  //   docTest.clear();
  //   digitalWrite(ledPin, arduino::LOW); // set the LED off
  //   delay(1000);
  //   digitalWrite(ledPin, arduino::HIGH);
  //   delay(1000);
  // }

  // put your main code here, to run repeatedly:
}

// const int ledPin = 13;

// the setup() method runs once, when the sketch starts

// void setup() {
//   initialize the digital pin as an output.
//   Serial1.begin(9600);
//   Serial1.println("Hello");
//   pinMode(ledPin, arduino::OUTPUT);
// }

// the loop() methor runs over and over again,
// as long as the board has power

// void loop() {
//   digitalWrite(ledPin, arduino::HIGH); // set the LED on
//   delay(1000);
//   Serial1.println("Hello");           // wait for a second
//   digitalWrite(ledPin, arduino::LOW); // set the LED off
//   delay(1000);                        // wait for a second
// }
