#include "RRT_Star.h"

const bool SHOW_ANIMATION = true;

namespace plt = matplotlibcpp;

void RRT_Star::plan_rrt(bool animation) {
  cout << "Running RRT* with start = " << _start_node 
       << " and end = " << _end_node << endl;
  _node_list.push_back(_start_node);

  for (int i = 0; i < _max_iters; i++) {
    cout << "Iteration " << i << ", Num nodes = " << size(_node_list) << endl;
    node_ptr rnd_node = get_random_node();
    int nearest_idx = get_nearest_node_index(_node_list, rnd_node);
    node_ptr nearest_node = _node_list[nearest_idx];
    node_ptr new_node = steer(nearest_node, rnd_node, _expand_dist);

    // init cost for candidate node
    new_node->cost = calc_new_cost(nearest_node, new_node);

    if (!check_collision(new_node, _obs, _robot_radius)) {
      list_idx near_idxs = find_near_nodes(new_node);
    }
  }
}

node_ptr RRT_Star::choose_parent(node_ptr new_node, list_idx idxs) const {

}

int RRT_Star::search_best_goal_node() const {

}

list_idx RRT_Star::find_near_nodes(node_ptr new_node) const {
  /**
   * 1) Define a ball centered on new_node
   * 2) returns all nodes of the tree that are inside this ball
   * 
   * Inputs
   *  new_node: randomly generated node without collision to obstacles
   * Return
   *  list of indexes of nodes from _node_list that are inside this ball
   */

  int num_nodes = size(_node_list) + 1;
  // see Sertac and Frazzoli Theorem 7: connectivity of random r-disc graphs
  double r = _connect_circle_dist * sqrt(log(num_nodes)/ num_nodes);
  r = min(r, _expand_dist);

  auto dist = [](const auto node1, const auto node2) -> double {
    return pow(node1->x - node2->x, 2) + pow(node1->y - node2->y, 2);
  };
  
  list_idx near_idxs;
  for(int i = 0; i < _node_list.size(); i++) {
    double distance = dist(new_node, _node_list[i]);
    if (distance <= pow(r, 2)) {
      near_idxs.push_back(i);
    }
  }

  return near_idxs;
}

void RRT_Star::rewire(node_ptr new_node, list_idx idxs) {

}

void RRT_Star::propogate_cost_to_leaves(node_ptr parent_node) {

}

double RRT_Star::calc_new_cost(node_ptr from_node, node_ptr to_node) {
  auto [d, th] = calc_dist_and_angle(from_node, to_node);
  return d + from_node->cost;
}

int main() {
    obstacle_list obs = {
    make_tuple(5.0, 5.0, 1.0),
    make_tuple(3.0, 6.0, 2.0),
    make_tuple(3.0, 8.0, 2.0),
    make_tuple(3.0, 10.0, 2.0),
    make_tuple(7.0, 5.0, 2.0),
    make_tuple(9.0, 5.0, 2.0),
    make_tuple(8.0, 10.0, 1.0),
    make_tuple(6.0, 12.0, 1.0)
  }; // x, y, radius

  position start = make_pair(0.0, 0.0);
  position end = make_pair(6.0, 10.0);
  double rand_area[2] = {-2, 15};
  double expand_dist = 1.0;

  RRT_Star rrt_star = RRT_Star(start,
                               end,
                               obs,
                               rand_area,
                               expand_dist);

  if (rrt_star.get_final_path().empty()) {
    cout << "Cannot find path..." << endl;
  }
  else {
    cout << "Path found!" << endl;

    if (SHOW_ANIMATION) {
      rrt_star.draw_graph(); // draw all nodes in node list
      vector<double> x_pos;
      vector<double> y_pos;
      for (auto pos: rrt_star.get_final_path()) {
        x_pos.push_back(pos.first);
        y_pos.push_back(pos.second);
      }
      plt::plot(x_pos, y_pos, "-r");
      plt::grid(true);
      plt::pause(0.1); 
    }
  }

  cout << "Press enter to close ..." << endl;
  cin.ignore();

  return 0;
}