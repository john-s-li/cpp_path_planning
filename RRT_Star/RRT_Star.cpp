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
    if (!new_node) continue;
    // init cost for candidate node
    new_node->cost = calc_new_cost(nearest_node, new_node);

    if (!check_collision(new_node, _obs, _robot_radius)) {
      list_idx near_idxs = find_near_nodes(new_node);
      node_ptr node_with_updated_parent = choose_parent(new_node, near_idxs);
      if (node_with_updated_parent) {
        rewire(node_with_updated_parent, near_idxs);
        _node_list.push_back(node_with_updated_parent);
      }
      else {
        _node_list.push_back(new_node);
      }
    }

    if (animation) draw_graph(rnd_node, "RRT*");

    if (!_search_until_max_iter) {
      int last_idx = search_best_goal_node();
      if (last_idx) {
        cout << "Goal node reached! Generating final path." << endl;
        generate_final_course(last_idx);
        break;
      }
    }
  }
}

node_ptr RRT_Star::choose_parent(node_ptr new_node, list_idx near_idxs) const {
  /**
   * Computes the cheapest point to new_node contained in the list
   * near_idxs and choose a new parent for new_node if a lower cost exists
   */
  if (near_idxs.empty()) return nullptr;
  bool lower_cost = false;

  double curr_cost = new_node->cost;
  for (int idx : near_idxs) {
    node_ptr near_node = _node_list[idx];
    node_ptr candidate_node = steer(near_node, new_node, _expand_dist);
    if (!check_collision(candidate_node, _obs, _robot_radius) &&
        candidate_node) {
      candidate_node->cost = calc_new_cost(near_node, candidate_node);
      if (candidate_node->cost < curr_cost) {
        curr_cost = candidate_node->cost;
        new_node = candidate_node;
        lower_cost = true;
      }
    }
  }

  if (!lower_cost) {
    cout << "No better parent found for node " << new_node << endl;
    return nullptr;
  }

  return new_node;
}

int RRT_Star::search_best_goal_node() const {
  vector<double> dists_to_goal;
  double cost_to_goal = INF;
  int best_idx = 0;

  for (int i = 0; i < _node_list.size(); i++) {
    node_ptr node = _node_list[i];
    double dist_to_goal = calc_dist_to_goal(node->x, node->y);
    if (dist_to_goal <= _expand_dist) {
      node_ptr tmp = steer(node, _end_node);
      if (!check_collision(tmp, _obs, _robot_radius) && tmp) {
        double tmp_cost = calc_new_cost(tmp, _end_node);
        if (tmp_cost < cost_to_goal) {
          best_idx = i;
        }
      }
    }
  }

  return best_idx;
}

list_idx RRT_Star::find_near_nodes(node_ptr new_node) const {
  /**
   * 1) Define a ball centered on new_node
   * 2) returns all nodes of the tree that are inside this ball
   */
  cout << "Finding near nodes\n";
  int num_nodes = size(_node_list) + 1;
  // see Sertac and Frazzoli Theorem 7: connectivity of random r-disc graphs
  double r = _connect_circle_dist * sqrt(log(num_nodes)/ num_nodes);
  cout << "r = " << r << " | expand_dist = " << _expand_dist << endl;
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

void RRT_Star::rewire(node_ptr new_node, list_idx near_idxs) {
  /**
   * For each node in near_idxs, check if it is cheaper to arrive
   * to them from new_node
   * If true, then re-assign the parents of nodes in near_idxs
   * to new_node 
   */
  for (int i : near_idxs) {
    node_ptr near_node = _node_list[i];
    node_ptr edge_node = steer(new_node, near_node);

    if (!edge_node) continue;

    if (!check_collision(edge_node, _obs, _robot_radius)){
      edge_node->cost = calc_new_cost(new_node, near_node);
      if (edge_node->cost < near_node->cost) {
        near_node = edge_node;
        propogate_cost_to_leaves(new_node);
      }  
    }
  }
} 

void RRT_Star::propogate_cost_to_leaves(node_ptr parent_node) {
  for (const auto node: _node_list) {
    if (node->parent == parent_node) {
      node->cost = calc_new_cost(parent_node, node);
      propogate_cost_to_leaves(node);
    }
  }
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
    //make_tuple(8.0, 10.0, 1.0),
    //make_tuple(6.0, 12.0, 1.0)
  }; // x, y, radius

  position start = make_pair(0.0, 0.0);
  position end = make_pair(6.0, 10.0);
  double rand_area[2] = {-2, 15};
  double expand_dist = 3.0;

  RRT_Star rrt_star = RRT_Star(start,
                               end,
                               obs,
                               rand_area,
                               expand_dist);

  rrt_star.plan_rrt(SHOW_ANIMATION);

  if (rrt_star.get_final_path().empty()) {
    cout << "Cannot find path..." << endl;
  }
  else {
    cout << "Path found!" << endl;

    if (SHOW_ANIMATION) {
      rrt_star.draw_graph(nullptr, "RRT*"); // draw all nodes in node list
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