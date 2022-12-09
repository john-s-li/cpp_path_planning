#ifndef HYBRID_A_STAR_H_
#define HYBRID_A_STAR_H_

#include <string>
#include <vector>
#include <memory>
#include <tuple>
#include <queue>
#include <unordered_map>

#include "kdtree.hpp"
#include "matplotlibcpp.h"
#include "reeds_shepp.h"
#include "config.h"
#include "a_star_helper.h"
#include "draw.h"

using Kdtree::KdTree;
typedef shared_ptr<Kdtree::KdTree> kdtree_ptr;

using namespace std;

class HybridAStar {
  public:
    struct Node : public AStarHelper::Node { // inheritance for polymorphic function pass-in
      Node(float x_ind, float y_ind, float yaw_ind, int direction,
           vector<float> xs, vector<float> ys, vector<float> yaws,
           vector<int> directions, float steer, float cost, float parent_idx)
      : x_ind(x_ind), y_ind(y_ind), yaw_ind(yaw_ind), direction(direction),
        xs(xs), ys(ys), yaws(yaws),
        directions(directions), 
        steer(steer), cost(cost), parent_idx(parent_idx)
      {}

      float x_ind;
      float y_ind;
      float yaw_ind;
      int direction; // {-1, 1}
      vector<float> xs;
      vector<float> ys;
      vector<float> yaws;
      vector<int> directions;
      float steer;
      float cost;
      float parent_idx; // hash value of the node's parent
    };

    struct Path {
      Path(vector<float> xs, vector<float> ys, vector<float> yaws,
          vector<int> directions, float cost) 
      : xs(xs), ys(ys), yaws(yaws),
        directions(directions), cost(cost) 
      {}

      vector<float> xs;
      vector<float> ys;
      vector<float> yaws;
      vector<int> directions;
      float cost;
    };

    // extended params from AStarHelper
    struct Params : public AStarHelper::Params {
      Params(float min_x, float min_y, float min_yaw,
            float max_x, float max_y, float max_yaw,
            float x_width, float y_width, float yaw_width,
            float xy_reso, float yaw_reso,
            vector<int> ox, vector<int> oy,
            kdtree_ptr tree)
      : min_x(min_x), min_y(min_y), min_yaw(min_yaw),
        max_x(max_x), max_y(max_y), max_yaw(max_yaw),
        x_width(x_width), y_width(y_width), yaw_width(yaw_width),
        xy_reso(xy_reso), yaw_reso(yaw_reso), obs_x(ox), obs_y(oy), tree(tree)
      {}

      float min_x;
      float min_y;
      float min_yaw;
      float max_x;
      float max_y;
      float max_yaw;
      float x_width;
      float y_width;
      float yaw_width;
      float xy_reso;
      float yaw_reso;
      vector<int> obs_x; // obstacle x-coords
      vector<int> obs_y; // obstacle y-coords
      kdtree_ptr tree;
    };

    typedef shared_ptr<Node> node_ptr;
    typedef shared_ptr<Params> params_ptr;
    typedef tuple<float, float> idx_with_cost; // [cost, node idx]
    typedef unordered_map<float, node_ptr> node_set; // [node idx, node]
    typedef vector<float> steer_set;
    typedef vector<float> motion_set;

  public:
    HybridAStar() = default;
    ~HybridAStar() = default;

    void run_hybrid_a_star(float start_x, float start_y, float start_yaw,
                           float goal_x, float goal_y, float goal_yaw,
                           vector<int> obs_x, vector<int> obs_y,
                           float xy_reso, float yaw_reso);
    
    Path extract_path() const;
    
    node_ptr calc_next_node(node_ptr curr_node, float hash_val, 
                            float steer_val, float direction_val);
    
    float make_hash_val(const node_ptr node) const;
    
    bool is_index_okay(float x_ind, float y_ind,
                       vector<float> x_list, vector<float> y_list,
                       vector<float> yaw_list) const;

    bool is_collision(vector<float> xs, vector<float> ys, 
                      vector<float> yaws) const;

    tuple<bool, node_ptr> update_node_with_analytic_expansion(
      node_ptr curr_node) const;

    float calc_rs_path_cost(ReedsSheppStateSpace::sample_paths rs_path) const;

    float calc_hybrid_cost(node_ptr node) const;

    ReedsSheppStateSpace::sample_paths analytic_expansion(
      node_ptr curr_node) const;

    static tuple<steer_set, motion_set> calc_motion_set();

    static bool is_same_grid(const node_ptr node1, const node_ptr node2);
    
    static params_ptr update_parameters(vector<int> obs_x, 
                                        vector<int> obs_y,
                                        float xy_reso, float yaw_reso, 
                                        kdtree_ptr kdtree);

    static tuple<vector<int>, vector<int>> design_obstacles(int x, int y);

    template <typename T> // for double or for float
    static T pi_2_pi(T theta);
  
  private:
    node_ptr start_node_;
    node_ptr goal_node_;

    priority_queue<idx_with_cost,
                   vector<idx_with_cost>,
                   greater<idx_with_cost>> path_pq_;

    node_set closed_set_; // visited nodes
    node_set open_set_; // nodes to be visited

    shared_ptr<Path> best_path_;
    params_ptr params_;

    heuristic_map hmap_;
};

#endif