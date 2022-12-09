#include <vector>
#include <iostream>

#include "a_star_helper.h"

using namespace std;

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

heuristic_map AStarHelper::calc_holonomic_heuristic_with_obs(
  float node_x, float node_y, const vector<int> obs_x, const vector<int> obs_y, 
  float reso, float robot_radius) {

  node_ptr n_goal = make_shared<Node>(round(node_x / reso),
                                      round(node_y / reso), 
                                      0.0, -1.0);

  vector<float> ox, oy;
  for (int i = 0; i < obs_x.size(); i++) {
    ox.push_back(obs_x[i] / reso);
    oy.push_back(obs_y[i] / reso);
  }
  
  params_ptr params = calc_params(ox, oy, robot_radius, reso);
  obs_map o_map = calc_obs_map(ox, oy, robot_radius, params);
    
}

AStarHelper::params_ptr AStarHelper::calc_params(
  const vector<float> ox, const vector<float> oy,
  float robot_radius, float reso) {

  float min_x = round(*min_element(ox.begin(), ox.end())); 
  float min_y = round(*min_element(oy.begin(), oy.end()));
  float max_x = round(*max_element(ox.begin(), ox.end())); 
  float max_y = round(*max_element(oy.begin(), oy.end()));

  float xw = max_x - min_x, yw = max_y - min_y;

  return make_shared<Params>(min_x, min_y, max_x, max_y,
                            xw, yw, reso);
}

obs_map AStarHelper::calc_obs_map(const vector<float> obs_x,
                                  const vector<float> obs_y,
                                  float robot_radius,
                                  params_ptr P) {

  cout << P->x_width << " , " << P->y_width << endl;

}