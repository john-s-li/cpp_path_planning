#include "hybrid_a_star.h"

using namespace std;

void HybridAStar::run_hybrid_a_star(float start_x, float start_y, float start_yaw,
                                    float goal_x, float goal_y, float goal_yaw,
                                    vector<float> obs_x, vector<float> obs_y,
                                    float xy_reso, float yaw_reso) {
  // sxr = start x with resolution r
  float sxr = round(start_x / xy_reso), syr = round(start_y / xy_reso);
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
    obs_x.append(x - 1);
    obs_y.append(i);
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
