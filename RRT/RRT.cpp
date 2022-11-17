#include "RRT.h"

namespace plt = matplotlibcpp;

// helper function
int find_min_elem_idx(vector<double> dists) {
  int min_idx;
  double min = INF;
  for(int i = 0; i < dists.size(); i++) {
    if (dists[i] < min) {
      min = dists[i];
      min_idx = i;
    }
  }

  return min_idx;
}

void RRT::plan_rrt(bool animation) {
  cout << "Start node: " << _start_node << endl;
  cout << "End node: " << _end_node << endl;
  _node_list.push_back(_start_node);
  for(int i = 0; i < _max_iters; i++) {
    node_ptr random_node = get_random_node();
    cout << "\nTrying random node: " << random_node << endl;
    int nearest_idx = get_nearest_node_index(_node_list, random_node);
    node_ptr nearest_node = _node_list[nearest_idx];
    cout << "Nearest node : " << nearest_node << endl;
    node_ptr new_node = steer(nearest_node, random_node, _expand_dist);

    if (!check_if_outside_play_area(new_node, _play_area) &&
        !check_collision(new_node, _obs, _robot_radius)) {
      cout << "Adding new node " << new_node << endl;
      _node_list.push_back(new_node); 
    }

    if (animation && i % 5 == 0) {
      cout << "Drawing animation" << endl;
      draw_graph(random_node);
    }

    if (calc_dist_to_goal(_node_list.back()->x, _node_list.back()->y)
        <= _expand_dist) {
      node_ptr final_node = steer(_node_list.back(), _end_node, _expand_dist);

      if (!check_collision(final_node, _obs, _robot_radius)) {
        cout << "Final node found!" << endl;
        generate_final_course(_node_list.size() - 1);
        break;
      }
    }
  }
}

void RRT::generate_final_course(int goal_idx) {
  _final_path.push_back(make_pair(_end_node->x, _end_node->y));
  node_ptr node = _node_list[goal_idx];
  while (node->parent) {
    _final_path.push_back(make_pair(node->x, node->y));
    node = node->parent;
  }
  _final_path.push_back(make_pair(node->x, node->y));
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
    new_node->path_x.push_back(to_node->x);
    new_node->path_y.push_back(to_node->y);
    new_node->x = to_node->x;
    new_node->y = to_node->y;
  }

  new_node->parent = from_node;

  return new_node;
}

double RRT::calc_dist_to_goal(double x, double y) const {
  return sqrt(pow(x - _end_node->x, 2) + pow(y - _end_node->y, 2));
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
    cout << "Sampling goal node" << endl;
    rand_node = _end_node;
  }

  return rand_node;
}

void RRT::draw_graph(node_ptr rnd_node) const {
  plt::clf();
  if (rnd_node) {
    plt::plot(vector<double> {rnd_node->x}, vector<double> {rnd_node->y}, "^k");
    if (_robot_radius > 0.0) {
      draw_circle(rnd_node->x, rnd_node->y, _robot_radius, "-r");
    }
  }

  for (const auto &node: _node_list) {
    if (node->parent) {
      plt::plot(node->path_x, node->path_y, "-g");
    }
  }

  for (const auto &ob: _obs) {
    auto [ox, oy, size] = ob;
    draw_circle(ox, oy, size);
  }

  plt::plot(vector<double>{_start_node->x}, 
            vector<double>{_start_node->y}, "xr");
  
  plt::plot(vector<double>{_end_node->x}, 
            vector<double>{_end_node->y}, "xr");
  
  plt::axis("equal");
  plt::xlim(-2, 15);
  plt::ylim(-2, 15);
  plt::grid(true);
  plt::title("RRT Path Finding in C++");

  plt::pause(0.02);
}

int RRT::get_nearest_node_index(const vector<node_ptr> node_list,
                                const node_ptr rnd_node) {
  vector<double> distances(node_list.size());
  for (int i = 0; i < node_list.size(); i++) {
    distances[i] = (pow(node_list[i]->x - rnd_node->x, 2) + 
                    pow(node_list[i]->y - rnd_node->y, 2));
  }        

  int min_idx = find_min_elem_idx(distances);
  return min_idx;                                  
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
        cout << "Collision of node " << node << endl;
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

void RRT::draw_circle(double x, double y, double size, string color) {
  static vector<double> deg(360/5);
  static bool deg_init = false;
  if (!deg_init) {
    generate(deg.begin(), deg.end(),
            [n = 0] () mutable { return n += 5; });
    deg.push_back(0);
    deg_init = true;
  }

  vector<double> x_circle;
  vector<double> y_circle;

  for(auto d: deg) {
    x_circle.push_back(x + size * cos(deg2rad(d)));
    y_circle.push_back(y + size * sin(deg2rad(d)));
  }
  plt::plot(x_circle, y_circle, color);
}

int main() {

  obstacle_list obs = {
    make_tuple(5.0, 5.0, 1.0),
    make_tuple(3.0, 6.0, 2.0),
    make_tuple(3.0, 8.0, 2.0),
    make_tuple(3.0, 10.0, 2.0),
    make_tuple(7.0, 5.0, 2.0),
    make_tuple(9.0, 5.0, 2.0),
    make_tuple(8.0, 10.0, 1.0)
  };

  position start = make_pair(0.0, 0.0);
  position end = make_pair(6.0, 10.0);
  double rand_area[2] = {-2, 15};

  RRT rrt(start, end, obs, rand_area);

  rrt.plan_rrt(true);

  if (rrt.get_final_path().empty()) {
    cout << "Cannot find path..." << endl;
  }
  else {
    cout << "Path found!" << endl;
  }

  cout << "Press enter to close ..." << endl;
  cin.ignore();

  return 0;
}