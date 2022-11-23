#ifndef HYBRID_A_STAR_H_
#define HYBRID_A_STAR_H_

#include <string>
#include <vector>
#include <memory>

#include "kdtree.hpp"
#include "matplotlibcpp.h"
#include "reeds_shepp.h"
#include "config.h"

struct Node {
  Node(double x_ind, double y_ind, double yaw_ind, int direction,
       vector<double> xs, vector<double> ys, vector<double> yaws,
       vector<int> &directions, double steer, double cost, int path_ind)
  : x_ind(x_ind), y_ind(y_ind), yaw_ind(yaw_ind), 
    direction(direction),
    xs(xs), ys(ys), yaws(yaws),
    directions(directions),
    steer(steer), cost(cost), path_ind(path_ind)
  {}

  double x_ind;
  double y_ind;
  double yaw_ind;
  int direction; // {-1, 1}
  vector<double> xs;
  vector<double> ys;
  vector<double> yaws;
  vector<int> directions;
  double steer;
  double cost;
  int path_ind;
};

struct Path {
  Path(vector<double> xs, vector<double> ys, vector<double> yaws,
       vector<int> directions, double cost) 
  : xs(xs), ys(ys), yaws(yaws),
    directions(directions), cost(cost) 
  {}

  vector<double> xs;
  vector<double> ys;
  vector<double> yaws;
  vector<int> directions;
  double cost;
};

// extended params from config based on obstacles list
struct Params {
  Params(double min_x, double min_y, double min_yaw,
         double max_x, double max_y, double max_yaw,
         double x_width, double y_width,
         vector<double> ox, vector<double> oy,
         Kdtree::KdTree tree)
  : min_x(min_x), min_y(min_y), min_yaw(min_yaw),
    max_x(max_x), max_y(max_y), max_yaw(max_yaw),
    x_width(x_width), y_width(y_width), ox(ox), oy(oy), tree(tree)
  {}

  double min_x;
  double min_y;
  double min_yaw;
  double max_x;
  double max_y;
  double max_yaw;
  double x_width;
  double y_width;
  vector<double> ox; // obstacle x-coords
  vector<double> oy; // obstacle y-coords
  Kdtree::KdTree tree;
};

class HybridAStar {

};

#endif