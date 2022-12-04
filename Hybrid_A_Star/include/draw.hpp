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

const float PI = C::instance().PI;
class Arrow {
  public:
    Arrow(float x, float y, float theta, 
          float length, string color, float line_width = 2.0) {
      
      float angle = deg2rad(30.0); // angle of the arrow head
      float d = 0.3 * length;

      map<string, string> settings;
      settings["linewidth"] = to_string(line_width);
      settings["color"] = color;

      // need to ensure that theta is in radians (simple check)
      if (theta > 2*PI) {
        theta = deg2rad(theta);
      } 

      float x_start = x;
      float y_start = y;
      float x_end = x + length * cos(theta);
      float y_end = y + length * sin(theta);  

      float theta_hat_L = theta + PI - angle;
      float theta_hat_R = theta + PI + angle;

      float x_hat_start = x_end;
      float x_hat_end_L = x_hat_start + d * cos(theta_hat_L);
      float x_hat_end_R = x_hat_start + d * cos(theta_hat_R);

      float y_hat_start = y_end;
      float y_hat_end_L = y_hat_start + d * sin(theta_hat_L);
      float y_hat_end_R = y_hat_start + d * sin(theta_hat_R);

      plt::plot(vector<float>{x_start, x_end},
                vector<float>{y_start, y_end}, settings);

      plt::plot(vector<float>{x_hat_start, x_hat_end_L},
                vector<float>{y_hat_start, y_hat_end_L}, settings);

      plt::plot(vector<float>{x_hat_start, x_hat_end_R},
                vector<float>{y_hat_start, y_hat_end_R}, settings);
    }
};

class Car {
  public:
   Car() = default;
   static void draw_car(float x, float y, float yaw, float steer, 
                        string color = "black");
};

void Car::draw_car(float x, float y, float yaw, float steer, string color) {
  Eigen::Matrix<float, 2, 5> car; // x-y positions of car polygon
  car << -C::instance().RB, -C::instance().RB, 
          C::instance().RF,  C::instance().RF, 
         -C::instance().RB, // first row
          C::instance().W / 2.0, -C::instance().W / 2.0,
         -C::instance().W / 2.0,  C::instance().W / 2.0,
          C::instance().W / 2.0;

  Eigen::Matrix<float, 2, 5> wheel;
  wheel << -C::instance().TR, -C::instance().TR, 
            C::instance().TR,  C::instance().TR, 
           -C::instance().TR,
            C::instance().TW / 4.0, -C::instance().TW / 4.0, 
           -C::instance().TW / 4.0, C::instance().TW / 4.0, 
            C::instance().TW / 4.0;

  Eigen::Matrix<float, 2, 5> rl_wheel = wheel;
  Eigen::Matrix<float, 2, 5> rr_wheel = wheel;
  Eigen::Matrix<float, 2, 5> fl_wheel = wheel;
  Eigen::Matrix<float, 2, 5> fr_wheel = wheel;

  // rotation matrices
  Eigen::Matrix2f R1;
  Eigen::Matrix2f R2;

  R1 << cos(yaw), -sin(yaw),
        sin(yaw),  cos(yaw);

  R2 <<  cos(steer), sin(steer),
        -sin(steer), cos(steer);

  fl_wheel = R2 * fl_wheel;
  fr_wheel = R2 * fr_wheel;

  Eigen::Vector2f add_to_fl; 
  Eigen::Vector2f add_to_fr; 

  add_to_fl << C::instance().WB,  C::instance().WD / 2.0;
  add_to_fr << C::instance().WB, -C::instance().WD / 2.0;

  fl_wheel.colwise() += add_to_fl;
  fr_wheel.colwise() += add_to_fr;
  
};

#endif