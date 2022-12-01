#ifndef DRAW_H_
#define DRAW_H_

#include <math.h>
#include <string> 
#include <map>

#include <Eigen/Dense>
#include "matplotlibcpp.h"
#include "config.h"

using namespace std;
namespace plt = matplotlibcpp;

const double PI = C::instance().PI;

class Arrow {
  public:
    Arrow(double x, double y, double theta, double length, string color) {
      
      double angle = deg2rad(30.0); // angle of the arrow head
      double d = 0.3 * length;
      double line_width = 2.0;

      map<string, string> settings;
      settings["linewidth"] = to_string(line_width);
      settings["color"] = color;

      // need to ensure that theta is in radians (simple check)
      if (theta > 2*PI) {
        theta = deg2rad(theta);
      }

      double x_start = x;
      double y_start = y;
      double x_end = x + length * cos(theta);
      double y_end = y + length * sin(theta);  

      double theta_hat_L = theta + PI - angle;
      double theta_hat_R = theta + PI + angle;

      double x_hat_start = x_end;
      double x_hat_end_L = x_hat_start + d * cos(theta_hat_L);
      double x_hat_end_R = x_hat_start + d * cos(theta_hat_R);

      double y_hat_start = y_end;
      double y_hat_end_L = y_hat_start + d * sin(theta_hat_L);
      double y_hat_end_R = y_hat_start + d * sin(theta_hat_R);

      plt::plot(vector<double>{x_start, x_end},
                vector<double>{y_start, y_end}, settings);

      plt::plot(vector<double>{x_hat_start, x_hat_end_L},
                vector<double>{y_hat_start, y_hat_end_L}, settings);

      plt::plot(vector<double>{x_hat_start, x_hat_end_R},
                vector<double>{y_hat_start, y_hat_end_R}, settings);

    }
};

class Car {
  public:
   Car() = default;
   static void draw_car(double x, double y, double yaw,
                        string color = "black");
};

void Car::draw_car(double x, double y, double yaw, string color) {

};

#endif