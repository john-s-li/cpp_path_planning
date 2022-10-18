#ifndef A_STAR_SOLVER_H
#define A_STAR_SOLVER_H

#include <queue>
#include <list>
#include <iostream>
#include "math.h"

#include "GridWorld.hpp"

using namespace std;

class AStarSolver {
    public:
        bool search(const GridWorld world, position start, position end);
        void printPath() const;
    private:
        vector<Node*> _visited;
        vector<Node*> _curr_path;
};

#endif
