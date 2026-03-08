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

#define ENCODER_FRONT_RIGHT_A 24
#define ENCODER_FRONT_RIGHT_B 25

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

#define SWEEPER_PIN PWM_6
#define FLAG_PIN PWM_5

Servo FrontLeftServo;
Servo FrontRightServo;
Servo BackLeftServo;
Servo BackRightServo;

Servo Sweeper;
Servo FlagDropper;

Adafruit_MPU6050 mpu;

void resetServos(){
  FrontLeftServo. write(SERVO_FL_HOME);
  FrontRightServo.write(SERVO_FR_HOME);
  BackLeftServo.  write(SERVO_BL_HOME);
  BackRightServo. write(SERVO_BR_HOME);
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

      fl_angle =      constrain(fl_angle,       10, 160);
      fr_angle =      constrain(fr_angle,       20, 160);
      bl_angle =      constrain(bl_angle,       30, 180);
      br_angle =      constrain(br_angle,       0,  150);
      sweeper_angle = constrain(sweeper_angle,  65, 150);

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

      // noInterrupts();
      // int32_t fl = encoder_fl_ticks;
      // int32_t fr = encoder_fr_ticks;
      // // int32_t br = encoder_br_ticks;
      // interrupts(); 

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      StaticJsonDocument<256> out;
      out["cmd"] = "response";
      JsonObject enc = out.createNestedObject("encoders");
      enc["front_left"] = encoder_fl_ticks;
      enc["front_right"] = encoder_fr_ticks;
      // enc["back_right"] = br;

      out["yaw"] = g.gyro.z;

      serializeJson(out, Serial4);
      Serial4.print('\n');
    }
  }
}