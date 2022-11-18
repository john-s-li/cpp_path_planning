#include "RRT_Star.h"

const bool SHOW_ANIMATION = true;

namespace plt = matplotlibcpp;

void RRT_Star::plan_rrt(bool animation) {

}

node_ptr RRT_Star::choose_parent(node_ptr new_node, list_idx idxs) const {

}

int RRT_Star::search_best_goal_node() const {

}

list_idx RRT_Star::find_near_nodes(node_ptr new_node) const {

}

void RRT_Star::rewire(node_ptr new_node, list_idx idxs) {

}

void RRT_Star::propogate_cost_to_leaves(node_ptr parent_node) {

}

double RRT_Star::calc_new_cost(node_ptr from_node, node_ptr to_node) {

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