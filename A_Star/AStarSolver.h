#ifndef A_STAR_SOLVER_H
#define A_STAR_SOLVER_H

#include "GridWorld.hpp"
#include <list>

using namespace std;

class AStarSolver {
    public:
        bool search(const GridWorld world, position start, position end);
        void printPath() const;
    private:
        vector<Node*> _visited;
        list<Node*> _to_visit;
        vector<Node*> _curr_path;
};

#endif
