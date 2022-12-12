#ifndef CONFIG_H_
#define CONFIG_H_

#include <math.h>
#include <limits>

using namespace std;

inline float deg2rad(float d) {
  return d * M_PI / 180.0;
}

template <typename T>
inline float hypot(T x, T y) {
  return sqrt(pow(x, 2) + pow(y, 2));
}

struct C { // Param Config
  const float PI = M_PI;
  
  const float XY_RESO = 2.0; // [m]
  const float YAW_RESO = deg2rad(15.0); // [rad]
  const float MOVE_STEP = 0.4; // [m] path interpolate resolution
  const float N_STEER = 20.0; // steer command number
  const int COLLISION_CHECK_STEP = 5; // skip number for collision check
  const int EXTEND_BOUND = 1; // collision check range extended
  
  const float GEAR_COST = 100.0; // switch back penalty
  const float BACKWARD_COST = 5.0; // backward penalty
  const float STEER_CHANGE_COST = 5.0; // steer angle change penalty
  const float STEER_ANGLE_COST = 1.0; // steer angle penalty
  const float H_COST = 15.0; // heuristic cost penalty

  // vehicle related params 
  const float RF = 4.5; // [m] distance from rear to vehicle front end
  const float RB = 1.0; // [m] distance from rear to vehicle back end
  const float W = 3.0; // [m] vehicle width
  const float WD = 0.7 * W; // [m] distance between left and right wheels
  const float WB = 3.5; // [m] wheel base
  const float TR = 0.5; // [m] tire radius
  const float TW = 1; // [m] tire width
  const float MAX_STEER = 0.6; // [rad] maximum steering angle

  static C& instance() {
    static C singleton;
    return singleton;
  }
};

#endif