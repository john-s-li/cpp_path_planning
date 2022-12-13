#include "hybrid_a_star.h"

void test() {
  printf("Reeds Shepp Test.\n");
  ReedsSheppStateSpace rs(0.3);
  float q_init[3] = {0.0, 0.0, 0.0};
  float q_goal[3] = {1.0, 1.0, 1.0};
  float step_size = 0.2;

  auto samples = rs.sample(q_init, q_goal, step_size);
  auto types = rs.type(q_init, q_goal);
  printf("Reeds Shepp Test done.\n");


  Draw::draw_car(0.0, 0.0, deg2rad(30.0), deg2rad(20.0));
}


int main() {
  test();

  cout << "Running Path Planning for Hybrid A*\n";

  // starting pose
  float sx = 10.0, sy = 7.0, syaw = deg2rad(120.0);
  // end pose
  float gx = 45.0, gy = 20.0, gyaw = deg2rad(90.0);

  auto [obs_x, obs_y] = HybridAStar::design_obstacles(51, 31);

  // cout << "Size of obs_x == obs_y : " << 
  //         boolalpha << (obs_x.size() == obs_y.size()) << endl;

  HybridAStar hybrid_a_star;
  hybrid_a_star.run_hybrid_a_star(sx, sy, syaw,
                                  gx, gy, gyaw,
                                  obs_x, obs_y,
                                  C::instance().XY_RESO,
                                  C::instance().YAW_RESO);

  return 0;
}