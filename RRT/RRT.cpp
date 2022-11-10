#include "RRT.h"

void RRT::plan_rrt(bool animation) {
  _node_list.push_back(_start_node);
  for(int i; i < _max_iters; i++) {
    node_ptr random_node = get_random_node();
    int nearest_idx = get_nearest_node_index(_node_list, random_node);
    node_ptr nearest_node = _node_list[nearest_idx];
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

bool check_if_outside_play_area(node_ptr node, shared_ptr<AreaBounds> play_area) {
  if(!play_area) return true; // no play area defined
  if (node->x < play_area->x_min || node->x > play_area->x_max ||
      node->y < play_area->y_min || node->y > play_area->y_max) {
        return false; // outside = bad
      }
  return true; 
}

bool check_collision(node_ptr node, obstacle_list obs_list, double robot_radius) {
  if (!node) return false;

  vector<double> d_list;
  double min_dist;
  for(auto &obs: obs_list) {
    auto [ox, oy, size] = obs;
    // calculate distance of node to all obstacles
    // if distance of node to an obstacle is < (robot_radius + size of obs)^2 --> collision
    for(int i = 0; i < node->path_x.size(); i++) {
      d_list.push_back(pow(ox - node->path_x[i], 2) + 
                       pow(oy - node->path_y[i], 2));

      // check smallest distance 
      min_dist = *min_element(d_list.begin(), d_list.end());
      if (min_dist < pow(size + robot_radius, 2)) {
        return false; // collision
      }
    }
  }s

  return true;
}

int main() {

  return 0;
}