#ifndef DRAW_H_
#define DRAW_H_

#include <math.h>
#include <string> 
#include <map>

#include <Eigen/Dense>
#include "matplotlibcpp.h"
#include "config.h"

const float PI = C::instance().PI;

class Draw {
  public:
    static void draw_arrow(float x, float y, float theta, 
                           float length, string color, float line_width = 2.0);

    static void draw_car(float x, float y, float yaw, float steer, 
                          string color = "black");
};

#endif