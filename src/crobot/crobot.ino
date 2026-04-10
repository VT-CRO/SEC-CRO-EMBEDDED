#include <ArduinoJson.h>
#include "Macros.h"
#include <Servo.h>
#include "MotorControl.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define BAUD_RATE 115200

#define MOTOR_FL_PIN_1 MD3_M1_PIN_2
#define MOTOR_FL_PIN_2 MD3_M1_PIN_1

#define MOTOR_BL_PIN_1 MD1_M2_PIN_2
#define MOTOR_BL_PIN_2 MD1_M2_PIN_1

#define MOTOR_BR_PIN_1 MD1_M1_PIN_1
#define MOTOR_BR_PIN_2 MD1_M1_PIN_2

#define MOTOR_FR_PIN_1 MD3_M2_PIN_1
#define MOTOR_FR_PIN_2 MD3_M2_PIN_2

#define WINCH_PIN_1    MD2_M2_PIN_1
#define WINCH_PIN_2    MD2_M2_PIN_2

#define ENCODER_FRONT_LEFT_A 33
#define ENCODER_FRONT_LEFT_B 13

#define ENCODER_FRONT_RIGHT_A 12
#define ENCODER_FRONT_RIGHT_B 11

// #define ENCODER_BACK_RIGHT_A 33
// #define ENCODER_BACK_RIGHT_B 13

volatile int32_t encoder_fl_ticks = 0;
volatile int32_t encoder_fr_ticks = 0;
// volatile int32_t encoder_br_ticks = 0;

MotorControl FrontLeftMotor  = MotorControl(MOTOR_FL_PIN_1, MOTOR_FL_PIN_2);
MotorControl FrontRightMotor = MotorControl(MOTOR_FR_PIN_1, MOTOR_FR_PIN_2);
MotorControl BackLeftMotor   = MotorControl(MOTOR_BL_PIN_1, MOTOR_BL_PIN_2);
MotorControl BackRightMotor  = MotorControl(MOTOR_BR_PIN_1, MOTOR_BR_PIN_2);

MotorControl Winch = MotorControl(WINCH_PIN_1, WINCH_PIN_2);

#define SERVO_FL_PIN PWM_9
#define SERVO_FR_PIN PWM_3
#define SERVO_BL_PIN PWM_8
#define SERVO_BR_PIN PWM_1

#define SERVO_FL_HOME 130
#define SERVO_FR_HOME 53
#define SERVO_BL_HOME 57
#define SERVO_BR_HOME 110
#define SWEEPER_HOME  150

#define SWEEPER_PIN PWM_6
#define FLAG_PIN PWM_5

// TODO: DEFINE PINS FOR ARM
#define SHOULDER_PIN  PWM_10
#define ELBOW_PIN     PWM_11
// #define GRIPPER_PIN   PWM_4 (erm, what pin is this now???)
#define CATAPULT_PIN PWM_4

// TODO: DEFINE POSITIONS FOR ARM
#define SHOULDER_OPEN 150
#define SHOULDER_CLOSE 0
#define ELBOW_OPEN 90
#define ELBOW_CLOSE 180
#define GRIPPER_OPEN 40 // needs testing
#define GRIPPER_CLOSE 0

#define WAIT_TIME 2000
#define WAIT_TIME_15 1500
#define CRATER_ORBIT_TIME 15000
#define CRATER_ORBIT_VELOCITY 140


Servo FrontLeftServo;
Servo FrontRightServo;
Servo BackLeftServo;
Servo BackRightServo;

Servo Sweeper;
Servo FlagDropper;

Servo Shoulder;
Servo Elbow;
Servo Gripper;
Servo Catapult;

// bool Start = false;

Adafruit_MPU6050 mpu;

enum State {
  JETSON,
  CRATER_ENTER_DELAY,
  CRATER_ENTER,
  CRATER_ENTERING,
  CRATER_ALIGN_DELAY,
  CRATER_ALIGN_ANKLE_1,
  CRATER_ALIGN_ANKLE_2,
  CRATER_ALIGN_ANKLE_3,
  CRATER_ALIGN_ANKLE_4,
  CRATER_ALIGNING,
  CRATER_ORBIT_DELAY,
  CRATER_ORBIT,
  CRATER_WAIT_AFTER,
} currentState;

// bool urMom;
bool isClosed;

elapsedMillis stateTime;
const unsigned int CRATER_ENTER_DELAY_MS = 2000; 
const unsigned int CRATER;

void resetServos(){
  FrontLeftServo. write(SERVO_FL_HOME);
  FrontRightServo.write(SERVO_FR_HOME);
  BackLeftServo.  write(SERVO_BL_HOME);
  BackRightServo. write(SERVO_BR_HOME);
  Sweeper.        write(SWEEPER_HOME);
  // Shoulder.       write(SHOULDER_DOWN);
  // Elbow.          write(ELBOW_DOWN);
  // Gripper.        write(GRIPPER_CLOSED);
}

// interrupts that trigger on channel A and 
// use channel B to determine direction
void isr_encoder_fl_a() {
  bool a = digitalReadFast(ENCODER_FRONT_LEFT_A);
  bool b = digitalReadFast(ENCODER_FRONT_LEFT_B);
  if (a == b) {
    encoder_fl_ticks += 1;
  } else {
    encoder_fl_ticks -= 1;
  }
}

void isr_encoder_fr_a() {
  bool a = digitalReadFast(ENCODER_FRONT_RIGHT_A);
  bool b = digitalReadFast(ENCODER_FRONT_RIGHT_B);
  if (a == b) {
    encoder_fr_ticks += 1;
  } else {
    encoder_fr_ticks -= 1;
  }
}

// void WaitforPhotoresistor(){
//   if (Start){
//     return;
//   }
//   int Ph_value = analogRead(Photoresistor_PIN);
//   Serial.println(Ph_value);
//   if(Ph_value < LED_threashold ){
//     // Serial.println(Ph_value);
//     Start = true;
//     Serial.println("start the bot");
//   }
// }
// void isr_encoder_br_a() {
//   bool a = digitalReadFast(ENCODER_BACK_RIGHT_A);
//   bool b = digitalReadFast(ENCODER_BACK_RIGHT_B);
//   if (a == b) {
//     encoder_br_ticks += 1;
//   } else {
//     encoder_br_ticks -= 1;
//   }
// }

void setup() {
  Serial4.begin(BAUD_RATE);

  FrontLeftServo. attach(SERVO_FL_PIN);
  FrontRightServo.attach(SERVO_FR_PIN);
  BackLeftServo.  attach(SERVO_BL_PIN);
  BackRightServo. attach(SERVO_BR_PIN);

  // encoders
  pinMode(ENCODER_FRONT_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_FRONT_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_FRONT_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_FRONT_RIGHT_B, INPUT_PULLUP);
  // pinMode(ENCODER_BACK_RIGHT_A, INPUT_PULLUP);
  // pinMode(ENCODER_BACK_RIGHT_B, INPUT_PULLUP);


  attachInterrupt(digitalPinToInterrupt(ENCODER_FRONT_LEFT_A), isr_encoder_fl_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_FRONT_RIGHT_A), isr_encoder_fr_a, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_FRONT_LEFT_B), isr_encoder_fl_a, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_FRONT_RIGHT_B), isr_encoder_fr_a, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_BACK_RIGHT_A), isr_encoder_br_a, CHANGE);

  Sweeper.attach(SWEEPER_PIN);
  FlagDropper.attach(FLAG_PIN);
  Catapult.attach(CATAPULT_PIN);

  Shoulder.attach(SHOULDER_PIN);
  Elbow.attach(ELBOW_PIN);
  // Gripper.attach(GRIPPER_PIN);

  Shoulder.write(0);
  Elbow.write(0);
  // Gripper.write(0);

  Wire2.begin();              // start I2C bus 2
  Wire2.setClock(400000);     // optional: fast I2C

  // initialize MPU6050 using Wire2
  if (!mpu.begin(0x68, &Wire2)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  delay(4000);
  // pinMode(LED_BUILTIN, OUTPUT);
  // for(int i = 0; i < 10; i++){
  //   digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  //   delay(200);                      // wait for a second
  //   digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  //   delay(200);                      // wait for a second
  // }

  resetServos();
  delay(2000);
  isClosed = true;
}

void loop() {
  // for (int i = 10; i < 160; i++) {
  //   FrontLeftServo. write(i);
  //   delay(100);
  // }

  // for (int i = 160; i > 10; i--) {
  //   FrontLeftServo. write(i);
  //   delay(100);
  // }

  String json_str = "";

  // loop to find the most recent command
  while (Serial4.available()) {
    String line = Serial4.readStringUntil('\n'); // should read until the very end, but if the loop keeps going, it wasn't the end
    if (line.length() > 0) { // if we were at the last command before, then reading more should give us nothing
        json_str = line;  // keep overwriting until buffer is empty
    }
  }

  if (json_str.length() > 0) {
    // Serial4.print(json_str);

    StaticJsonDocument<256> doc;
    deserializeJson(doc, json_str);

    String cmd = doc["cmd"];

    if (cmd == "read") {
      // Serial4.write("recieved read command");
    }
    else if (cmd == "write") {
      currentState = JETSON;
      
      int fl_angle = doc["ankles"]["front_left"];
      int fr_angle = doc["ankles"]["front_right"];
      int bl_angle = doc["ankles"]["back_left"];
      int br_angle = doc["ankles"]["back_right"];

      int fl_speed = doc["wheels"]["front_left"];
      int fr_speed = doc["wheels"]["front_right"];
      int bl_speed = doc["wheels"]["back_left"];
      int br_speed = doc["wheels"]["back_right"];

      int sweeper_angle = doc["sweeper"];
      int winch_speed = doc["winch"];

      int flag_angle = doc["flag"];

      int shoulder_angle = doc["shoulder"];
      int elbow_angle = doc["elbow"];
      int gripper_angle = doc["gripper"];

      fl_angle =      constrain(fl_angle,       10, 160);
      fr_angle =      constrain(fr_angle,       20, 160);
      bl_angle =      constrain(bl_angle,       30, 180);
      br_angle =      constrain(br_angle,       0,  150);
      sweeper_angle = constrain(sweeper_angle,  60, 165);

      //TODO: Add contraints to all new servos after testing
      flag_angle =      constrain(flag_angle,     80, 150); // Open at 150, closed at 80

      shoulder_angle =  constrain(shoulder_angle, 0, 150); // Optimal for use is 150
      elbow_angle =     constrain(elbow_angle,    80, 180); // Optimal for use is 90
      gripper_angle =   constrain(gripper_angle,  0, 120); // Remember tbe 1.5 multiplier for angle

      FrontLeftServo. write(fl_angle);
      FrontRightServo.write(fr_angle);
      BackLeftServo.  write(bl_angle);
      BackRightServo. write(br_angle);

      FrontLeftMotor.Motor_start(fl_speed);
      FrontRightMotor.Motor_start(fr_speed);
      BackLeftMotor.Motor_start(bl_speed);
      BackRightMotor.Motor_start(br_speed);

      Sweeper.write(sweeper_angle);
      Winch.Motor_start(winch_speed);

      // Set the assignment for all
      FlagDropper.write(flag_angle);
      Shoulder.write(shoulder_angle);
      Elbow.write(elbow_angle);
      Gripper.write(gripper_angle);

      // noInterrupts();
      // int32_t fl = encoder_fl_ticks;
      // int32_t fr = encoder_fr_ticks;
      // // int32_t br = encoder_br_ticks;
      // interrupts(); 

      // OUTPUT --------------------------------------------

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      StaticJsonDocument<256> out;
      out["cmd"] = "response";
      JsonObject enc = out.createNestedObject("encoders");
      enc["front_left"] = encoder_fl_ticks;
      enc["front_right"] = encoder_fr_ticks;
      // enc["back_right"] = br;

      out["yaw"] = g.gyro.z;

      out["photoresistor"] = analogRead(Photoresistor_PIN);
      Serial.print(analogRead(Photoresistor_PIN));
      serializeJson(out, Serial4);
      Serial4.print('\n');
    }

    else if (cmd == "launchDrone") {
      // launch drone
      stateTime = elapsedMillis();
      Catapult.write(150);
    }

    else if (cmd == "closeArm") {
      Elbow.write(ELBOW_CLOSE);
      if (!isClosed) {
        isClosed = true;
        stateTime = elapsedMillis();
      }

      if (stateTime >= WAIT_TIME_15) {
          Shoulder.write(SHOULDER_CLOSE);
      }
      // Gripper.write(gripper_angle);
    }

    else if (cmd == "openArm") {
      Shoulder.write(SHOULDER_OPEN);

      if (isClosed) {
        isClosed = false;
        stateTime = elapsedMillis();
      }

      if (stateTime >= WAIT_TIME_15) {
        Elbow.write(ELBOW_OPEN);
      }

      // Gripper.write(GRIPPER_OPEN);
    }
    
    else if (cmd == "craterRun") {
      /*
      enum State {
        JETSON,
        CRATER_ENTER_DELAY,
        CRATER_ALIGN_DELAY,
        CRATER_ALIGN_ANKLES,
        CRATER_ORBIT_DELAY,
        CRATER_ORBIT,
        CRATER_WAIT_AFTER
      } currentState;
      */
      if (currentState == JETSON) {
        stateTime = elapsedMillis();

        FrontLeftMotor.Motor_start(0);
        FrontRightMotor.Motor_start(0);
        BackLeftMotor.Motor_start(0);
        BackRightMotor.Motor_start(0);
        Winch.Motor_start(0);
        resetServos();

        currentState = CRATER_ENTER_DELAY;
      }
      else if (currentState == CRATER_ENTER_DELAY) {
        if (stateTime >= WAIT_TIME) {
          currentState = CRATER_ALIGN_ANKLE_1;
          stateTime = 0;
        }
      }
      
      // 90 degrees
      else if (currentState == CRATER_ALIGN_ANKLE_1) {
        if (stateTime >= WAIT_TIME) {
          FrontLeftServo.write(SERVO_FL_HOME - 92 / 0.9);
          stateTime = 0;
          currentState = CRATER_ALIGN_ANKLE_2;
        }
      }

      // 90 degrees
      else if (currentState == CRATER_ALIGN_ANKLE_2) {
        if (stateTime >= WAIT_TIME) {
          FrontRightServo.write(SERVO_FR_HOME -92 / 0.9);
          stateTime = 0;
          currentState = CRATER_ALIGN_ANKLE_3;
        }
      }

      // 67.5 degrees
      else if (currentState == CRATER_ALIGN_ANKLE_3) {
        if (stateTime >= WAIT_TIME) {
          BackLeftServo.write(SERVO_BL_HOME + 67.5 / 0.9);
          stateTime = 0;
          currentState = CRATER_ALIGN_ANKLE_4;
        }
      }

      // 67.5 degrees
      else if (currentState == CRATER_ALIGN_ANKLE_4) {
        if (stateTime >= WAIT_TIME) {
          BackRightServo.write(SERVO_BR_HOME - 67.5 / 0.9);
          stateTime = 0;
          currentState = CRATER_ALIGN_DELAY;
        }
      }

      else if (currentState == CRATER_ALIGN_DELAY) {
        if (stateTime >= WAIT_TIME) {
          stateTime = 0;
          currentState = CRATER_ORBIT;
        }

      }
      else if (currentState == CRATER_ORBIT) {
        FrontLeftMotor.Motor_start(CRATER_ORBIT_VELOCITY);
        FrontRightMotor.Motor_start(-CRATER_ORBIT_VELOCITY);
        BackLeftMotor.Motor_start(-CRATER_ORBIT_VELOCITY);
        BackRightMotor.Motor_start(CRATER_ORBIT_VELOCITY);
        currentState = CRATER_ORBIT_DELAY;
      }
      else if (currentState == CRATER_ORBIT_DELAY) {
        if (stateTime >= CRATER_ORBIT_TIME) {
          currentState = CRATER_WAIT_AFTER;
          stateTime = 0;
        }
      }
      else if (currentState == CRATER_WAIT_AFTER) {
        FrontLeftMotor.Motor_start(0);
        FrontRightMotor.Motor_start(0);
        BackLeftMotor.Motor_start(0);
        BackRightMotor.Motor_start(0);
      }
      

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      StaticJsonDocument<256> out;
      out["cmd"] = "response";
      JsonObject enc = out.createNestedObject("encoders");
      enc["front_left"] = encoder_fl_ticks;
      enc["front_right"] = encoder_fr_ticks;
      // enc["back_right"] = br;

      out["yaw"] = g.gyro.z;

      out["photoresistor"] = analogRead(Photoresistor_PIN);
      serializeJson(out, Serial4);
      Serial4.print('\n');
    }
  }
}
