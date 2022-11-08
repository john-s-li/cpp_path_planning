#ifndef A_STAR_SOLVER_H
#define A_STAR_SOLVER_H

#include <iostream>
#include <unordered_set>
#include <tuple>
#include <stack>
#include "math.h"

#include "GridWorld.hpp"

using namespace std;

// priority queue with sorted storage
typedef tuple<double, Node*> node_with_cost;

class AStarSolver {
  public:
    bool search(const GridWorld world, const position start, const position end);
    void printPath(const GridWorld world, 
                   const position start, const position end) const;
  private:
    bool _goal_reached = false;
    // open and closed lists
    unordered_set<Node*> _visited; 
    priority_queue<node_with_cost, 
                   vector<node_with_cost>,
                   greater<node_with_cost>> _to_visit_pq; 
};

#endif
