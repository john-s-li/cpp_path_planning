#include "draw.h"

using namespace std;
namespace plt = matplotlibcpp;

void Draw::draw_arrow(float x, float y, float theta, 
                float length, string color, float line_width) {
      
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

void Draw::draw_car(float x, float y, float yaw, float steer, string color) {
  // need 5 points to close the polygon in plotter
  Eigen::Matrix<float, 2, 5, Eigen::RowMajor> car; // x-y positions of car polygon
  car << -C::instance().RB, -C::instance().RB, 
          C::instance().RF,  C::instance().RF, 
         -C::instance().RB, // first row
          C::instance().W / 2.0, -C::instance().W / 2.0,
         -C::instance().W / 2.0,  C::instance().W / 2.0,
          C::instance().W / 2.0;

  Eigen::Matrix<float, 2, 5, Eigen::RowMajor> wheel;
  wheel << -C::instance().TR, -C::instance().TR, 
            C::instance().TR,  C::instance().TR, 
           -C::instance().TR,
            C::instance().TW / 4.0, -C::instance().TW / 4.0, 
           -C::instance().TW / 4.0, C::instance().TW / 4.0, 
            C::instance().TW / 4.0;

  Eigen::Matrix<float, 2, 5, Eigen::RowMajor> rl_wheel = wheel;
  Eigen::Matrix<float, 2, 5, Eigen::RowMajor> rr_wheel = wheel;
  Eigen::Matrix<float, 2, 5, Eigen::RowMajor> fl_wheel = wheel;
  Eigen::Matrix<float, 2, 5, Eigen::RowMajor> fr_wheel = wheel;

  // rotation matrices
  Eigen::Matrix2f R1;
  Eigen::Matrix2f R2;

  // Yaw
  R1 << cos(yaw), -sin(yaw),
        sin(yaw),  cos(yaw);

  // Steering
  // transpose of R1 due to industry convention
  R2 <<  cos(steer), sin(steer),
        -sin(steer), cos(steer);

  // Rotate the front wheels based on steering
  fl_wheel = R2 * fl_wheel;
  fr_wheel = R2 * fr_wheel;

  Eigen::Vector2f add_to_fl; 
  Eigen::Vector2f add_to_fr; 

  add_to_fl << C::instance().WB,  C::instance().WD / 2.0;
  add_to_fr << C::instance().WB, -C::instance().WD / 2.0;

  fl_wheel.colwise() += add_to_fl;
  fr_wheel.colwise() += add_to_fr;

  // Do yaw rotation for all of car
  fl_wheel = R1 * fl_wheel;
  fr_wheel = R1 * fr_wheel;

  rl_wheel.row(1).array() += C::instance().WD / 2.0;
  rr_wheel.row(1).array() -= C::instance().WD / 2.0;

  rl_wheel = R1 * rl_wheel;
  rr_wheel = R1 * rr_wheel;

  car = R1 * car;

  Eigen::Vector2f start; start << x, y;

  fl_wheel.colwise() += start;
  fr_wheel.colwise() += start;
  rl_wheel.colwise() += start;
  rr_wheel.colwise() += start;

  car.colwise() += start;

  vector<float> car_x(car.row(0).data(), car.row(0).data() + car.cols());
  vector<float> car_y(car.row(1).data(), car.row(1).data() + car.cols());

  vector<float> fl_x(fl_wheel.row(0).data(), fl_wheel.row(0).data() + fl_wheel.cols());
  vector<float> fl_y(fl_wheel.row(1).data(), fl_wheel.row(1).data() + fl_wheel.cols());

  vector<float> fr_x(fr_wheel.row(0).data(), fr_wheel.row(0).data() + fr_wheel.cols());
  vector<float> fr_y(fr_wheel.row(1).data(), fr_wheel.row(1).data() + fr_wheel.cols());

  vector<float> rl_x(rl_wheel.row(0).data(), rl_wheel.row(0).data() + rl_wheel.cols());
  vector<float> rl_y(rl_wheel.row(1).data(), rl_wheel.row(1).data() + rl_wheel.cols());

  vector<float> rr_x(rr_wheel.row(0).data(), rr_wheel.row(0).data() + rr_wheel.cols());
  vector<float> rr_y(rr_wheel.row(1).data(), rr_wheel.row(1).data() + rr_wheel.cols());

  plt::plot(car_x, car_y, color);
  plt::plot(fl_x, fl_y, color);
  plt::plot(fr_x, fr_y, color);
  plt::plot(rl_x, rl_y, color);
  plt::plot(rr_x, rr_y, color);
  draw_arrow(x, y, yaw, C::instance().WB * 0.8, color);
  plt::show();
};
