#include "RRT.h"

void RRT::plan_rrt(bool animation) {
  _node_list.push_back(_start_node);
  for(int i; i < _max_iters; i++) {
    node_ptr random_node = get_random_node();
    int nearest_idx = get_nearest_node_index(_node_list, random_node);
    node_ptr nearest_node = _node_list[nearest_idx];

    node_ptr new_node = steer(nearest_node, random_node, _expand_dist);

    if (!check_if_outside_play_area(new_node, _play_area) &&
        !check_collision(new_node, _obs, _robot_radius)) {
      _node_list.push_back(new_node);
    }

    if (animation && i % 5 == 0) {
      draw_graph(random_node);
    }
  }
}

node_ptr RRT::steer(node_ptr from_node, node_ptr to_node, double extend_length) {
  // make this as to not alter from_node
  node_ptr new_node = make_shared<Node>(from_node->x, from_node->y);

  auto [d, theta] = calc_dist_and_angle(new_node, to_node);

  new_node->path_x.push_back(new_node->x);
  new_node->path_y.push_back(new_node->y);

  if (extend_length > d) {
    extend_length = d;
  }

  int num_expansions = floor(extend_length / _path_res);

  for (int i = 0; i < num_expansions; i++) {
    new_node->x += _path_res * cos(theta);
    new_node->y += _path_res * sin(theta);
    new_node->path_x.push_back(new_node->x);
    new_node->path_y.push_back(new_node->y);
  }

  // might not be exactly at the to_node so try extending just a wee bit
  // NOTE: need to check how to re-assign values using this syntax
  // without making new variables
  auto [last_d, last_theta] = calc_dist_and_angle(new_node, to_node);

  if (last_d <= _path_res) {
    new_node->path_x.push_back(new_node->x);
    new_node->path_y.push_back(new_node->y);
    new_node->x = to_node->x;
    new_node->y = to_node->y;
  }

  new_node->parent = from_node;

  return new_node;
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

void RRT::draw_graph(node_ptr rnd_node) const {

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

bool RRT::check_if_outside_play_area(node_ptr node, shared_ptr<AreaBounds> play_area) {
  if(!play_area) return false; // no play area defined
  if (node->x < play_area->x_min || node->x > play_area->x_max ||
      node->y < play_area->y_min || node->y > play_area->y_max) {
        return true; // outside = bad
      }
  return false; 
}

bool RRT::check_collision(node_ptr node, obstacle_list obs_list, double robot_radius) {
  if (!node) return true; // don't add to node list

  vector<double> d_list;
  double min_dist;
  for(auto &obs: obs_list) {
    auto [ox, oy, size] = obs;
    // calculate distance of node to all obstacles
    // if distance of node to an obstacle is < (robot_radius + size of obs)^2 --> collision
    for(int i = 0; i < node->path_x.size(); i++) {
      d_list.push_back(sqrt(pow(ox - node->path_x[i], 2) + 
                            pow(oy - node->path_y[i], 2)));

      // check smallest distance 
      min_dist = *min_element(d_list.begin(), d_list.end());
      // do circle collision checks
      if (min_dist <= size + robot_radius) {  // robot intersects with obs
        return true; // collision
      }
    }
  }

  return false;
}

dist_and_angle RRT::calc_dist_and_angle(node_ptr from_node, node_ptr to_node) {
  double dx = to_node->x - from_node->x;
  double dy = to_node->y - from_node->y;
  double d = sqrt(pow(dx, 2) + pow(dy, 2));
  double theta = atan2(dy, dx);

  return make_tuple(d, theta);
}

int main() {

  return 0;
}