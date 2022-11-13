#ifndef RRT_SOLVER_
#define RRT_SOLVER_

#include <math.h>
#include <iostream>
#include <vector>
#include <memory>
#include <limits.h>
#include <string>
#include <random>
#include <tuple>

#include <Eigen/Dense>
// #include <matplot/matplot.h>
#include "matplotlibcpp.h"

using namespace std;

struct Node; // prevent circular dependency

const bool SHOW_ANIMATION = true;
double INF = numeric_limits<double>::infinity();

typedef pair<double, double> position;
typedef tuple<double, double, double> obstacle;
typedef vector<obstacle> obstacle_list;
typedef tuple<double, double> dist_and_angle;
typedef shared_ptr<Node> node_ptr;

struct Node {
  double x;
  double y;
  vector<double> path_x;
  vector<double> path_y;
  shared_ptr<Node> parent;

  Node(double x, double y) : 
    x(x), y(y) {}
};

ostream& operator<< (ostream& out, const Node* node) {
  return out << "(" << node->x << ", " << node->y << ")";
}

struct AreaBounds {
  double x_min = 0.0; 
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;

  AreaBounds(const double area[4]) : 
    x_min(area[0]), x_max(area[1]),
    y_min(area[2]), y_max(area[3]) {};
};

inline double deg2rad(double d) {
  return M_PI * d / 180.0;
}

class RRT {
  public:
    RRT(position start, 
        position end,
        obstacle_list obs,
        double rand_area[2],
        double play_area[4] = nullptr,
        double expand_dist = 3.0,
        double path_res = 0.5,
        int goal_sample_rate = 5,
        int max_iters = 500,
        double robot_radius = 0.0
    ) : _start(start), 
        _end(end),
        _obs(obs),
        _expand_dist(expand_dist),
        _path_res(path_res),
        _goal_sample_rate(goal_sample_rate),
        _max_iters(max_iters),
        _robot_radius(robot_radius) {

      _start_node = make_shared<Node>(_start.first, _start.second);
      _end_node = make_shared<Node>(_end.first, _end.second);
      _min_rand = rand_area[0];
      _max_rand = rand_area[1];

      if (play_area) {
        _play_area = make_shared<AreaBounds>(play_area);
      }
      else {
        _play_area = nullptr;;
      }
    }

    // member functions
    void plan_rrt(bool animation = SHOW_ANIMATION);
    node_ptr steer(node_ptr from_node, node_ptr to_node,
                   double extend_length = INF);
    vector<position> generate_final_course(const position goal_idx);
    double calc_dist_to_goal(double x, double y);
    node_ptr get_random_node() const;
    void draw_graph(node_ptr rnd = nullptr) const;

    static void draw_circle(double x, double y, double size, 
                            string color = "-b");
    static int get_nearest_node_index(
                  const vector<node_ptr> node_list,
                  const node_ptr rnd_node);
    static bool check_if_outside_play_area(
                node_ptr node, shared_ptr<AreaBounds> play_area);
    static bool check_collision(node_ptr node,
                                obstacle_list obs_list,
                                double robot_radius);
    static dist_and_angle calc_dist_and_angle(node_ptr from_node,
                                              node_ptr to_node);

  private:
    position _start; // start position [x, y]
    position _end;   // goal position [x, y]
    obstacle_list _obs; // list of obstacles [[x, y, size], ...]
    shared_ptr<AreaBounds> _play_area; // solver stays inside this area 
    double _expand_dist;
    double _path_res; // path resolution
    int _goal_sample_rate;
    int _max_iters;
    double _robot_radius; // robot body modeled as circle with given radius

    node_ptr _start_node;
    node_ptr _end_node;
    double _min_rand; // random sampling area [min, max]
    double _max_rand;
    vector<node_ptr> _node_list;
};

#endif