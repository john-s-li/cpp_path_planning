#include "RRT.h"

void RRT::plan_rrt(bool animation) {
  _node_list.push_back(_start_node);
  for(int i; i < _max_iters; i++) {
    node_ptr random_node = get_random_node();
    int nearest_idx = get_nearest_node_index(_node_list, random_node);
  }
}

node_ptr RRT::get_random_node() const {
  random_device rd; // random number from hardware
  mt19937 gen(rd()); // see the generator
  uniform_int_distribution<> uniform_int(0, 100); 
  uniform_real_distribution<double> x_distr(_min_rand, _max_rand);
  uniform_real_distribution<double> y_distr(_min_rand, _max_rand);

  node_ptr rand_node;
  if (uniform_int(gen) > _goal_sample_rate) {
    // generate random node
    rand_node = make_shared<Node>(x_distr(gen), y_distr(gen));
  }
  else {
    // goal node sampling
    rand_node = _end_node;
  }

  return rand_node;
}

int RRT::get_nearest_node_index(const vector<node_ptr> node_list,
                                const node_ptr rnd_node) {
  vector<double> distances(node_list.size());
  for (auto node: node_list) {
    distances.push_back(pow(node->x - rnd_node->x, 2) +
                        pow(node->y - rnd_node->y, 2));
  }        

  return distance(distances.begin(), 
                  min_element(distances.begin(), distances.end()));                                   
}

int main() {

  return 0;
}