#ifndef HYBRID_A_STAR_H_
#define HYBRID_A_STAR_H_

#include <string>
#include <vector>
#include <memory>
#include <tuple>
#include <unordered_map>

#include "kdtree.hpp"
#include "matplotlibcpp.h"
#include "reeds_shepp.h"
#include "config.h"

using Kdtree::KdTree;

struct Node;
struct Path;
struct Params;

typedef tuple<double, Path> path_with_cost;
typedef shared_ptr<Node> node_ptr;
typedef unordered_map<double, node_ptr> node_set;
typedef shared_ptr<KdTree> kdtree_ptr;
typedef shared_ptr<Params> params_ptr;
typedef vector<double> steer_set;
typedef vector<double> motion_set;
typedef vector<vector<double>> heuristic_map;

struct Node {
  Node(double x_ind, double y_ind, double yaw_ind, int direction,
       vector<double> xs, vector<double> ys, vector<double> yaws,
       vector<int> &directions, double steer, double cost, double hash_val)
  : x_ind(x_ind), y_ind(y_ind), yaw_ind(yaw_ind), 
    direction(direction),
    xs(xs), ys(ys), yaws(yaws),
    directions(directions),
    steer(steer), cost(cost), hash_val(hash_val)
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
  double hash_val; // need to confirm this as we go
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
         kdtree_ptr tree)
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
  kdtree_ptr tree;
};


class HybridAStar {
  public:
    void run_hybrid_a_star(double start_x, double start_y, double start_yaw,
                           double goal_x, double goal_y, double goal_yaw,
                           vector<double> obs_x, vector<double> obs_y,
                           double xy_reso, double yaw_reso);
    
    Path extract_path() const;
    
    node_ptr calc_next_node(node_ptr curr_node, double hash_val, 
                            double steer_val, double direction_val);
    
    double make_hash_val(const node_ptr node) const;
    
    bool is_index_okay(double x_ind, double y_ind,
                       vector<double> x_list, vector<double> y_list,
                       vector<double> yaw_list) const;

    bool is_collision(vector<double> xs, vector<double> ys, 
                      vector<double> yaws) const;

    tuple<bool, node_ptr> update_node_with_analytic_expansion(
      node_ptr curr_node) const;

    double calc_rs_path_cost(ReedsSheppStateSpace::sample_paths rs_path) const;

    double calc_hybrid_cost(node_ptr node) const;

    ReedsSheppStateSpace::sample_paths analytic_expansion(
      node_ptr curr_node) const;

    static tuple<steer_set, motion_set> calc_motion_set();

    static heuristic_map calc_holonomic_heuristic_with_obs(
      node_ptr curr_node, vector<double> obs_x, vector<double> obs_y,
      double resolution , double robot_radius);

    static bool is_same_grid(const node_ptr node1, const node_ptr node2);
    
    static params_ptr update_parameters(vector<double> obs_x, 
                                        vector<double> obs_y,
                                        double yaw_reso, kdtree_ptr kdtree);
  
  private:
    node_ptr start_node_;
    node_ptr goal_node_;

    priority_queue<path_with_cost,
                   vector<path_with_cost>,
                   greater<path_with_cost>> path_pq_;

    node_set closed_set_; // visited nodes
    node_set open_set_; // nodes to be visited

    shared_ptr<Path> best_path_;
    params_ptr params_;

    heuristic_map hmap_;
};

#endif