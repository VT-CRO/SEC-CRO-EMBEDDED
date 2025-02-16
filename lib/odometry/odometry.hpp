/**
 * @file 
 * @brief This file definies the serial communication protocol for a through bore encoder
 * 
 */
#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <stdint.h>
#include <RotaryEncoder.h>

struct Odometry {
  RotaryEncoder*  Encoder1;
  RotaryEncoder*  Encoder2;
  RotaryEncoder*  Encoder3;
  // double          imuHeading;
};

#endif