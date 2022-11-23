#include "hybrid_a_star.h"

using namespace std;

int main() {
  ReedsSheppStateSpace* rs = new ReedsSheppStateSpace(0.3);
  double q_init[3] = {0.0, 0.0, 0.0};
  double q_goal[3] = {1.0, 1.0, 1.0};
  double step_size = 0.2;

  auto samples = rs->sample(q_init, q_goal, step_size);
  auto types = rs->type(q_init, q_goal);

  return 0;
}