#ifndef A_STAR_SOLVER_H
#define A_STAR_SOLVER_H

#include <queue>
#include <list>
#include <iostream>
#include <set>
#include <stack>
#include "math.h"

#include "GridWorld.hpp"

using namespace std;

class AStarSolver {
  public:
    bool search(const GridWorld world, const position start, const position end);
    void printPath(const GridWorld world, const position end) const;
  private:
    bool _goal_reached = false;
    set<Node*> _visited; 
};

#endif
