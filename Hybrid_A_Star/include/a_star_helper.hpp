#ifndef A_STAR_HELPER_H_
#define A_STAR_HELPER_H_

#include <vector>
#include <math.h>
#include <memory>

typedef vector<vector<float>> heuristic_map;

class AStarHelper {
  public:
    struct Node {
      float x;
      float y;
      float cost;
      float parent_idx;

      Node(float x, float y, float cost, float parent_idx)
      : x(x), y(x), cost(cost), parent_idx(parent_idx) {}

    }; // end Node

    typedef shared_ptr<Node> node_ptr;

    struct Params {
      float min_x;
      float min_y;
      float max_x;
      float max_y;
      float x_width;
      float y_width;
      float reso; // resolution of the grid
      static vector<pair<int, int>> motion_set;

      Params(float min_x, float min_y, float max_x, float max_y, 
             float xw, float yw, float reso)
      : min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y),
        x_width(xw), y_width(yw), reso(reso) {}

    }; // end Params

    typedef shared_ptr<Params> params_ptr;

    static heuristic_map calc_holonomic_heuristic_with_obs(
      node_ptr curr_node, params_ptr P, float robot_radius);

    private:
      params_ptr params_;


}; // end AStarHelper

vector<pair<int, int>> AStarHelper::Params::motion_set = {
  make_pair(-1,  0),
  make_pair(-1,  1),
  make_pair( 0,  1),
  make_pair( 1,  1),
  make_pair( 1,  0),
  make_pair( 1, -1),
  make_pair( 0, -1),
  make_pair(-1, -1),
};


#endif