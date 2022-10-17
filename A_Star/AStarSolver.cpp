#include "AStarSolver.h"
#include <iostream>
#include "math.h"

bool AStarSolver::search(const GridWorld world, position start, position end) {
    const int MAX_ITERS = pow(world.size() / 2, 10);
    bool goal_reached = false;
    Node* goal_node = world.getNode(end);

    auto [goal_r, goal_c] = end;

    Node* start_node_ptr = world.getNode(start);
    _to_visit.push_back(start_node_ptr);
    _curr_path.push_back(start_node_ptr);

    // check validity of start and end nodes
    if (!(world.isValid(start.first, start.second)) || 
        !(world.isValid(end.first, end.second))) {
        cout << "start or end position is not valid." << endl;
        return false;
    }

    // run A* search 
    int iters = 1;
    while(!(_to_visit.empty())) {
        iters++;
        if (iters > MAX_ITERS) {
            cout << "Max iterations reached. Path not found." << endl;
            break;
        }

        Node* curr_node_ptr = _to_visit.front();
        
        if (curr_node_ptr == goal_node) {
            goal_reached = true;
            break;
        }

        auto [curr_g, curr_h] = curr_node_ptr->getCosts();

        _to_visit.pop_front();
        list<Node*> neighbors = world.getNeighbors(curr_node_ptr);

        // neighbor with lowest g + h cost < curr_node will be set as new node
        for (auto neighbor: neighbors) {
            auto [neighbor_r, neighbor_c] = neighbor->getPosition();
            // Use Distance as heuristic
            double heuristic_cost = 
                pow(goal_r - neighbor_r,2) + pow(goal_c - neighbor_c,2);
            neighbor->setCosts(curr_g + 1.0, heuristic_cost);

        }

        
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