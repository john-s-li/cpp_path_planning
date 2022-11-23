#ifndef CONFIG_H_
#define CONFIG_H_

#include <math.h>

inline double deg2rad(double d) {
  return d * M_PI / 180.0;
}

struct C { // Param Config
  const double PI = M_PI;
  
  const double XY_RESO = 2.0; // [m]
  const double YAW_RESO = deg2rad(15.0); // [rad]
  const double MOVE_STEP = 0.4; // [m] path interpolate resolution
  const double N_STEER = 20.0; // steer command number
  const int COLLISION_CHECK_STEP = 5; // skip number for collision check
  const int EXTEND_BOUND = 1; // collision check range extended
  
  const double GEAR_COST = 100.0; // switch back penalty
  const double BACKWARD_COST = 5.0; // backward penalty
  const double STEER_CHANGE_COST = 5.0; // steer angle change penalty
  const double STEER_ANGLE_COST = 1.0; // steer angle penalty
  
  // vehicle related params 
  const double RF = 4.5; // [m] distance from rear to vehicle front end
  const double RB = 1.0; // [m] distance from rear to vehicl back end
  const double W = 3.0; // [m] vehicle width
  const double WD = 0.7 * W; // [m] distance between left and right wheels
  const double WB = 3.5; // [m] wheel base
  const double TR = 0.5; // [m] tire radius
  const double TW = 1; // [m] tire width
  const double MAX_STEER = 0.6; // [rad] maximum steering angle
};

#endif