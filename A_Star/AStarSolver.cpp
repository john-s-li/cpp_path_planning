#include "AStarSolver.h"

bool AStarSolver::search(const GridWorld world, 
                         const position start, const position end) {
  const int MAX_ITERS = pow(world.size() / 2, 10);
  Node* goal_node = world.getNode(end);
  auto [goal_r, goal_c] = end;
  Node* start_node_ptr = world.getNode(start);

  start_node_ptr->setCosts(0.0, 0.0);

  // define priority queue (sorted by highest to lowest cost)
  auto node_compare = [](const Node* a, const Node* b) {
      return (a->getCosts().first + a->getCosts().second <
              b->getCosts().first + b->getCosts().second);
  };

  priority_queue<Node*, 
                  vector<Node*>, 
                  decltype(node_compare)> to_visit_pq(node_compare); 
                
  to_visit_pq.push(start_node_ptr);

  // check validity of start and end nodes
  if (!(world.isValid(start.first, start.second)) || 
      !(world.isValid(end.first, end.second))) {
      cout << "start or end position is not valid." << endl;
      return false;
  }

  // run A* search 
  int iters = 1;
  while(!(to_visit_pq.empty())) {
    if (iters > MAX_ITERS && !_goal_reached) {
      cout << "Max iterations reached. Path not found." << endl;
      break;
    }

    Node* curr_node_ptr = to_visit_pq.top();
    _visited.insert(curr_node_ptr);
    to_visit_pq.pop();
    
    if (curr_node_ptr == goal_node) {
      _goal_reached = true;
      break;
    }

    auto [curr_g, curr_h] = curr_node_ptr->getCosts();
    vector<Node*> neighbors = world.getNeighbors(curr_node_ptr);

    // neighbor with lowest g + h cost < curr_node will be set as new node
    for (auto neighbor: neighbors) {
      auto [neighbor_r, neighbor_c] = neighbor->getPosition();
      // Use euclidean distance as heuristic
      double curr_cost = curr_g + 1.0;
      double heuristic_cost = 
          pow(goal_r - neighbor_r, 2) + pow(goal_c - neighbor_c, 2);

      if (curr_cost < neighbor->getCosts().first) {
        // set new parent and costs for neighor
        neighbor->setParent(curr_node_ptr);
        neighbor->setCosts(curr_cost, heuristic_cost);
      }

      // add to priority queue if NOT visited already
      if (find(_visited.begin(), _visited.end(), neighbor) == _visited.end()) {
              to_visit_pq.push(neighbor);
      }
    }
      
    iters++;
  }

  return _goal_reached;
}

void AStarSolver::printPath(const GridWorld world, const position end) const {
  // pretty print the current path
  if (_goal_reached) {
    stack<Node*> node_path;
    node_path.push(world.getNode(end));
    Node* node_to_add;

    while(node_to_add != nullptr) {
      node_path.push(node_path.top()->getParent());
      node_to_add = node_to_add->getParent();
    }

    // Print out the paths
    Node* print_node_ptr;
    cout << "Path: " << endl;
    while(!node_path.empty()) {
      print_node_ptr = node_path.top();
      cout << "\t" << *print_node_ptr << endl;
    }
  }
  else {
    cout << "No path was found..." << endl;
  }
}

int main() {
    AStarSolver a_star_solver;
    GridWorld world;
    position start = make_pair(0,0);
    position end = make_pair(ROWS-1, COLS-1);

    bool found = a_star_solver.search(world, start, end);
    if (found) {
      a_star_solver.printPath(world, end);
    }
    else {
      cout << "No path found." << endl;
    }

    return 0;
}