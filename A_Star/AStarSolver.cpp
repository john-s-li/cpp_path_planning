#include "AStarSolver.h"

bool AStarSolver::search(const GridWorld world, position start, position end) {
    const int MAX_ITERS = pow(world.size() / 2, 10);
    bool goal_reached = false;
    Node* goal_node = world.getNode(end);
    auto [goal_r, goal_c] = end;
    Node* start_node_ptr = world.getNode(start);
    _curr_path.push_back(start_node_ptr);

    // define priority queue (sorted by lowest cost)
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
        iters++;
        if (iters > MAX_ITERS) {
            cout << "Max iterations reached. Path not found." << endl;
            break;
        }

        Node* curr_node_ptr = to_visit_pq.top();
        to_visit_pq.pop();
        
        if (curr_node_ptr == goal_node) {
            goal_reached = true;
            break;
        }

        auto [curr_g, curr_h] = curr_node_ptr->getCosts();
        list<Node*> neighbors = world.getNeighbors(curr_node_ptr);

        // neighbor with lowest g + h cost < curr_node will be set as new node
        for (auto neighbor: neighbors) {
            auto [neighbor_r, neighbor_c] = neighbor->getPosition();
            // Use euclidean distance as heuristic
            double heuristic_cost = 
                pow(goal_r - neighbor_r, 2) + pow(goal_c - neighbor_c, 2);
            neighbor->setCosts(curr_g + 1.0, heuristic_cost);
            if (find(_visited.begin(), _visited.end(), neighbor) == _visited.end()) {
                to_visit_pq.push(neighbor);
            }
        }

        _visited.push_back(curr_node_ptr);        
    }

    return goal_reached;
}

void AStarSolver::printPath() const {
    // pretty print the current path


}

int main() {
    AStarSolver a_star_solver;

    return 0;
}