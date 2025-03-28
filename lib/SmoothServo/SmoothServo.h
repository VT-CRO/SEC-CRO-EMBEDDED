// /////////////////////////////////////////////////////////////
// // Author: Jayson De La Vega   R&D Team VT CRO
// // filename: SmoothServo.h
// // Last Modified: 3/24/2024
// // Description:  This file contains class declarations for
// //               a servo class with smoother movement
// /////////////////////////////////////////////////////////////
#include <Servo.h>

class SmoothServo {
    public:
        SmoothServo(int pin);
        ~SmoothServo();

        void updatePosition();
        void setConstants(double damping, double resonant, double r);
        void setGoalPosition(int angle);
        int getAngle();
        bool atGoalPosition();

    private:
        Servo servo;
        double k1;
        double k2;
        double k3;
        int goalAngle;
        int y;
        int yd;
        int ydd;
        int xd;
        long lastUpdate;
};