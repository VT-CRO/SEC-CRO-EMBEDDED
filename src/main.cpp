#include "comm.hpp"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <stdlib.h>
#include "odometry.hpp"
#include "Macros.h"
#include "Servo.h"
#include "ServoPositions.h"
#include "LimitSwitch.h"
#include "Speed.h"
#include "SmoothServo.h"

// put function declarations here:
// int myFunction(int, int);

enum BIN_MOVE{ IN, OUT, STOP };

enum SORTING_FLAP_MOVE{ CENTER, GEODINIUM, NEBULITE };

enum BEACON{UP, DOWN};

enum VIBRATING{VIBRATE, STOP_VIBRATING};

bool START{false};

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
#define EL_PIN_1 Encoder_2_PIN_2
#define EL_PIN_2 Encoder_2_PIN_1

#define ER_PIN_1 Encoder_1_PIN_1
#define ER_PIN_2 Encoder_1_PIN_2

#define EC_PIN_1 Encoder_3_PIN_1
#define EC_PIN_2 Encoder_3_PIN_2

// Bin Motors Macros
#define BIN_PIN_1 MD2_M2_PIN_1
#define BIN_PIN_2 MD2_M2_PIN_2

// Intake Motor Macros
#define INTAKE_PIN_1 MD2_M1_PIN_1
#define INTAKE_PIN_2 MD2_M1_PIN_2

// Elevator Motor Macros
#define ELEVATOR_PIN_1 MD1_M1_PIN_1
#define ELEVATOR_PIN_2 MD1_M1_PIN_2

// Vibrator Motor Macros
#define VIBRATOR_PIN_1 MD1_M2_PIN_1
#define VIBRATOR_PIN_2 MD1_M2_PIN_2

// Servo pins
#define BIN_FLAP_PIN PWM_3
#define BEACON_EXTENDED_PIN PWM_1
#define BEACON_DROP_PIN PWM_2

// limit switch
#define LIMIT_SWITCH_PIN PWM_4

#define Bin_Release_Delay 300

MotorControl FR = MotorControl(FR_PIN_1, FR_PIN_2);
MotorControl FL = MotorControl(FL_PIN_1, FL_PIN_2);
MotorControl BR = MotorControl(BR_PIN_1, BR_PIN_2);
MotorControl BL = MotorControl(BL_PIN_1, BL_PIN_2);

MotorControl bin = MotorControl(BIN_PIN_1, BIN_PIN_2);
MotorControl material = MotorControl(INTAKE_PIN_1, INTAKE_PIN_2);
MotorControl elevator = MotorControl(ELEVATOR_PIN_1, ELEVATOR_PIN_2);
MotorControl vibrator = MotorControl(VIBRATOR_PIN_1, VIBRATOR_PIN_2);

RotaryEncoder ENCODER1(EL_PIN_1, EL_PIN_2, RotaryEncoder::LatchMode::TWO03); // left
RotaryEncoder ENCODER2(ER_PIN_1, ER_PIN_2, RotaryEncoder::LatchMode::TWO03); // right
RotaryEncoder ENCODER3(EC_PIN_1, EC_PIN_2, RotaryEncoder::LatchMode::TWO03); // center

Odometry odom = {&ENCODER1, &ENCODER2, &ENCODER3};

SmoothServo Bin_Flap(BIN_FLAP_PIN); 
SmoothServo Beacon_Exteded(BEACON_EXTENDED_PIN);
SmoothServo Beacon_Drop(BEACON_DROP_PIN);

LimitSwitch Limit_Switch = LimitSwitch(LIMIT_SWITCH_PIN);

devices activeDevices = {&FL, &FR, &BL, &BR};

jetsonComms jet(activeDevices);

bool run = true;

int bin_intake = STOP;

int sorting = CENTER;

bool beacon = UP;

bool vibrate = STOP_VIBRATING;

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

void BinIntake(int bin_state) {
  switch (bin_state)
  {
    case IN:
      // motors move the belts to take in the bin
      if(!Limit_Switch.readSwitch())
      {
        bin.Motor_start(Bin_In_Speed);
      }
      else
      {
        bin.Motor_start(Stop);
      }
    break;
    case OUT:
      // motors move to push out the bin
      bin.Motor_start(Bin_Out_Speed);
    break;
    case STOP:
      // motors stop once the limit switch is pressed by the bin
      bin.Motor_start(Stop);
    break;
  }
}

void SortingFlap(int sort, bool vibrate) {
  // Vibrator is a part of this
  switch(sort)
  {
    case CENTER:
      // Servo starts in this state but just keeps the flap over both sides
      vibrator.Motor_start(Stop);
      Bin_Flap.setGoalPosition(SortingFlap_Closed);
      // moveServo(Bin_Flap, SortingFlap_Closed, SortingFlap_Speed);
    break;
    case GEODINIUM:
      // Servo opens the left side
      vibrator.Motor_start(Vibrator_Speed);
      Bin_Flap.setGoalPosition(SortingFlap_OpenGeodinium);
      // moveServo(Bin_Flap, SortingFlap_OpenGeodinium, SortingFlap_Speed);
    break;
    case NEBULITE:
      // Servo opens the right side
      vibrator.Motor_start(Vibrator_Speed);
      Bin_Flap.setGoalPosition(SortingFlap_OpenNebulite);
      // moveServo(Bin_Flap, SortingFlap_OpenNebulite, SortingFlap_Speed);
    break;
  }
}

void BeaconDrop(bool beacon) {
  switch(beacon)
  {
    case UP:
      Beacon_Drop.setGoalPosition(Beacon_Drop_Up);
      Beacon_Exteded.setGoalPosition(Beacon_Exteded_Up);
      // moveServo(Beacon_Exteded,  Beacon_Exteded_Up , Beacon_Exteded_Speed);
      // moveServo(Beacon_Drop, Beacon_Drop_Up, Beacon_Drop_Speed);
    
    break;
    case DOWN:
      Beacon_Exteded.setGoalPosition(Beacon_Exteded_Down);

      if (Beacon_Exteded.atGoalPosition())
        Beacon_Drop.setGoalPosition(Beacon_Drop_Down);

      // moveServo(Beacon_Exteded, Beacon_Exteded_Down , Beacon_Exteded_Speed );
      // moveServo(Beacon_Drop, Beacon_Exteded_Down , Beacon_Drop_Speed); // add if()

    break;
  }
}

bool StartLedRead() {
  if(analogRead(Photoresistor_PIN) >= LED_threashold){ // todo
    return true;
  }
  return false;
}

// bool goToServoAngle(Servo motor, float desiredPosition, int speed, int dt) {
//   // interpolate currentPos and desiredPosition'
//   // 

//   int pos = motor.read();

//   int time = 50 - speed;

//   if(abs(pos - desiredPosition) > 2)
//   {
//     if (desiredPosition < pos) {
//       pos += speed*dt;
//     } else {
//       pos ++;
//     }
//     motor.write(pos);
//   }

//   while(abs(pos - desiredPosition) > 2) {

//       if (desiredPosition < pos) {
//         pos --;
//       } else {
//         pos ++;
//       }

      
//       delay(time);
//   }
//   motor.write(desiredPosition);
//   delay(5);

// }

void setup() {
  Serial8.begin(115200);
  
  Bin_Flap.setConstants(0.1, 0, 0);
  Beacon_Exteded.setConstants(0.1, 0, 0);
  Beacon_Drop.setConstants(0.1, 0, 0);

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

    switch (msg_type)
    {
      case 0:
        doc.clear();
        doc["start_led"] = START;
        doc["deadwheel_stats"]["encoder_left"] = (odom.Encoder1)->getPosition();
        doc["deadwheel_stats"]["encoder_right"] = (odom.Encoder2)->getPosition();
        doc["deadwheel_stats"]["encoder_center"] = (odom.Encoder3)->getPosition();
        doc["deadwheel_stats"]["heading"] = 1.0;
        serializeJson(doc, Serial8);
        Serial8.print("\n");
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

          sorting = doc["sorting"];

          beacon = doc["beacon"];

          vibrate = doc["vibrate"];

          BinIntake(bin_intake);
          SortingFlap(sorting, vibrate);
          BeaconDrop(beacon);
          /*
          bin intake motors - 3 states: IN, OUT, STOP
          sorting - 3 states: CENTER, LEFT, RIGHT
          beacon - 2 states(?): UP, DOWN
          vibrator - 2 states: ON, OFF
          reset case for the odo - probably as a basic function
          */
        } else {
          FL.Motor_setGoalSpeed(0);
          FR.Motor_setGoalSpeed(0);
          BL.Motor_setGoalSpeed(0);
          BR.Motor_setGoalSpeed(0);
          bin_intake = false;
        }
        break;
      case 2:
        // Set encoders to 0 or something to reset odo
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

  // Updating servo goal positions
  Bin_Flap.updatePosition();
  Beacon_Drop.updatePosition();
  Beacon_Exteded.updatePosition();

  Limit_Switch.updateSwitch();

  if(StartLedRead() || Limit_Switch.readSwitchHold()) {
    START = true;
    elevator.Motor_start(Elevator_Speed);
    material.Motor_start(Material_Speed);
    bin.Motor_start(Bin_Releasing_Speed);
    delay(Bin_Release_Delay);
    bin.Motor_start(0);
  }
}
