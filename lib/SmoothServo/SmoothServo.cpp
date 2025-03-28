// /////////////////////////////////////////////////////////////
// // Author: Jayson De La Vega   R&D Team VT CRO
// // filename: SmoothServo.cpp
// // Last Modified: 3/24/2024
// // Description:  This file contains class definitions for
// //               a servo class with smoother movement
// /////////////////////////////////////////////////////////////
#include "arduino_freertos.h"
#include "SmoothServo.h"

SmoothServo::SmoothServo(int pin)
{
    Servo s;
    servo = s;
    
    servo.attach(pin);
    lastUpdate = millis();

    ydd = 0;
    yd = 0;
    y = 0;
    xd = 0;
}

SmoothServo::~SmoothServo()
{
    servo.detach();
}

void SmoothServo::setConstants(double damping, double resonant, double r)
{
    k1 = damping / (2 * PI * resonant);
    k2 = 1 / ((2 * PI * resonant) * (2 * PI * resonant));
    k3 = r * damping / (2 * PI * resonant);
}

void SmoothServo::updatePosition()
{
    unsigned long dt = millis() - lastUpdate;
    if (dt >= 100) {
        lastUpdate = millis();

        y = dt * (k3 * xd + goalAngle - k1 * ydd - k2 * yd);
        
        xd = goalAngle;
        ydd = yd;
        yd = y;

        servo.write(y);
    }
}

bool SmoothServo::atGoalPosition()
{
    return (goalAngle == y);
}

void SmoothServo::setGoalPosition(int angle)
{
    goalAngle = angle;
    if (goalAngle > 180)
        goalAngle = 180;
    if (goalAngle < 0)
        goalAngle = 0;
}

int SmoothServo::getAngle() {
  return y;
}