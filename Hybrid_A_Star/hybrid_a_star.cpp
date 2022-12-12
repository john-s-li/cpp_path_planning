#include "hybrid_a_star.h"

using namespace std;

ostream& operator<< (ostream& out, HybridAStar::node_ptr node) {
  return out << "(" << node->x << ", " << node->y << ")";
}

template<typename T>
T HybridAStar::pi_2_pi(T theta) {
  while (theta > C::instance().PI) {
    theta -= 2.0 * C::instance().PI;
  }

  while (theta < -C::instance().PI) {
    theta += 2.0 * C::instance().PI;
  }
 
  return theta;
}

void HybridAStar::run_hybrid_a_star(float start_x, float start_y, float start_yaw,
                                    float goal_x, float goal_y, float goal_yaw,
                                    vector<int> obs_x, vector<int> obs_y,
                                    float xy_reso, float yaw_reso) {

  // sxr = start x with resolution r
  float sxr = round(start_x / xy_reso), syr = round(start_y / xy_reso);
  float gxr = round(goal_x / xy_reso),  gyr = round(goal_y / xy_reso);

  float syawr = round(pi_2_pi<float>(start_yaw) / yaw_reso);
  float gyawr = round(pi_2_pi<float>(goal_yaw) / yaw_reso);

  start_node_ = make_shared<Node>(sxr, syr, syawr, 1, 
                                  vector<float>{start_x}, 
                                  vector<float>{start_y}, 
                                  vector<float>{start_yaw},
                                  vector<int>{1}, 
                                  0.0, 0.0, -1.0);

  goal_node_  = make_shared<Node>(gxr, gyr, gyawr, 1, 
                                  vector<float>{goal_x}, 
                                  vector<float>{goal_y}, 
                                  vector<float>{goal_yaw},
                                  vector<int>{1}, 
                                  0.0, 0.0, -1.0);

  cout << "Hybrid A* Start: " << start_node_ << endl;
  cout << "Hybrid A* Goal: " << goal_node_ << endl;

  // make the obstacle kd-tree
  Kdtree::KdNodeVector nodes;

  for (int i = 0; i < obs_x.size(); i++) {
    vector<float> point(2);
    // cout << "Obs: (" << obs_x[i] << " , " << obs_y[i] << ")\n";
    point[0] = float(obs_x[i]);
    point[1] = float(obs_y[i]);
    nodes.push_back(Kdtree::KdNode(point));
  }

  kdtree_ptr kd_tree = make_shared<KdTree>(&nodes);

  params_ = update_parameters(obs_x, obs_y, xy_reso, yaw_reso, kd_tree);

  heuristic_map hmap = AStarHelper::calc_holonomic_heuristic_with_obs(
                        goal_node_->xs[0], goal_node_->ys[0], 
                        params_->obs_x, params_->obs_y,
                        params_->xy_reso, 1.0);

  // cout << "Heuristic nodes map by A* helper done.\n";

  auto [steer_set, direct_set] = calc_motion_set();

}

HybridAStar::params_ptr HybridAStar::update_parameters(
                    vector<int> obs_x, vector<int> obs_y,
                    float xy_reso, float yaw_reso, kdtree_ptr kd_tree) {

  float min_x = round(*min_element(obs_x.begin(), obs_x.end()) / xy_reso);
  float min_y = round(*min_element(obs_y.begin(), obs_y.end()) / xy_reso);
  float max_x = round(*max_element(obs_x.begin(), obs_x.end()) / xy_reso);
  float max_y = round(*max_element(obs_y.begin(), obs_y.end()) / xy_reso);

  float x_width = max_x - min_x, y_width = max_y - min_y;

  float min_yaw = round(-C::instance().PI / yaw_reso) - 1;
  float max_yaw = round( C::instance().PI / yaw_reso);
  float yaw_width = max_yaw - min_yaw;

  return make_shared<Params>(min_x, min_y, min_yaw,
                             max_x, max_y, max_yaw,
                             x_width, y_width, yaw_width,
                             xy_reso, yaw_reso,
                             obs_x, obs_y, kd_tree);

}

tuple<HybridAStar::steer_set, HybridAStar::direct_set> HybridAStar::calc_motion_set() {
  float ds = C::instance().MAX_STEER / C::instance().N_STEER;
  float c = ds;
  
  steer_set s = steer_set(C::instance().N_STEER - 1);
  s[0] = ds;
  generate(s.begin() + 1, s.end(), [&c, ds]() -> float 
                                    { c += ds; return c; } );

  s.push_back(0.0);

  c = -ds;
  steer_set neg_s = steer_set(C::instance().N_STEER - 1);
  neg_s[0] = -ds;
  generate(neg_s.begin() + 1, neg_s.end(), [&c, ds]() -> float 
                                            { c -= ds; return c; } );

  s.insert(s.end(), neg_s.begin(), neg_s.end());

  direct_set d = direct_set(s.size(), 1.0);
  direct_set neg_d = direct_set(s.size(), -1.0);

  d.insert(d.end(), neg_d.begin(), neg_d.end());

  steer_set dup_s = s;
  s.insert(s.end(), dup_s.begin(), dup_s.end());

  // cout << "Size of steer set = " << s.size() << endl; 
  // cout << "Size of direct set = " << d.size() << endl; 

  return make_tuple(s, d);
}

tuple<vector<int>, vector<int>> HybridAStar::design_obstacles(int x, int y) {
  /**
   * Make (x, y) points that compose walls of parking scenario
   */

  vector<int> obs_x, obs_y;

  for (int i = 0; i < x; i++) {
    obs_x.push_back(i);
    obs_y.push_back(0);
  }

  for (int i = 0; i < x; i++) {
    obs_x.push_back(i);
    obs_y.push_back(y - 1);
  }

  for (int i = 0; i < y; i++) {
    obs_x.push_back(0);
    obs_y.push_back(i);
  }

  for (int i = 0; i < y; i++) {
    obs_x.push_back(x - 1);
    obs_y.push_back(i);
  }  

  for (int i = 10; i < 21; i++) {
    obs_x.push_back(i);
    obs_y.push_back(15);
  }

  for (int i = 0; i < 15; i++) {
    obs_x.push_back(20);
    obs_y.push_back(i);
  }

  for (int i = 15; i < 30; i++) {
    obs_x.push_back(30);
    obs_y.push_back(i);
  }

  for (int i = 0; i < 16; i++) {
    obs_x.push_back(40);
    obs_y.push_back(i);
  }

  return make_tuple(obs_x, obs_y);
}
