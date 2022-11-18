#ifndef RRT_STAR_H_
#define RRT_STAR_H_

#include "RRT.h"

using namespace std;

typedef vector<int> list_idx; 

class RRT_Star : public RRT {
  public:
    RRT_Star(position start, 
             position end,
             obstacle_list obs,
             double rand_area[2],
             double expand_dist = 3.0,
             double path_res = 1.0,
             int goal_sample_rate = 20,
             int max_iters = 300,
             double connect_circle_dist = 50.0,
             bool search_until_max_iter = false,
             double robot_radius = 0.8
    ) : RRT(start, end, obs, rand_area, nullptr, 
            expand_dist, path_res, goal_sample_rate,
            max_iters, robot_radius)
    {
      _connect_circle_dist = connect_circle_dist;
      _search_until_max_iter = search_until_max_iter;
    }

    void plan_rrt(bool animation = false);
    node_ptr choose_parent(node_ptr new_node, list_idx idxs) const;
    int search_best_goal_node() const;
    list_idx find_near_nodes(node_ptr new_node) const;
    void rewire(node_ptr new_node, list_idx idxs);
    void propogate_cost_to_leaves(node_ptr parent_node);
    static double calc_new_cost(node_ptr from_node, node_ptr to_node);
    
  private:
    double _connect_circle_dist;
    bool _search_until_max_iter;
}; 

#endif