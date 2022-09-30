#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <stdexcept>
#include <limits>
#include <string>

using namespace std;

// set a global inf variable
const int INF = numeric_limits<int>::max();

template <class T>
using inner_map = unordered_map<T, int>; // equivalent to a typedef (C++11)

template <class T>
using outer_map = unordered_map<T, inner_map<T> >;

template <class T> 
class Graph 
{
    public:
        Graph(vector<T> nodeList, T start_node);
        void addNodeNeighborsAndCost(T val, inner_map<T> NandC);
        void initCosts();
        void buildParents(T node);
        bool checkValid(T value) const;
        bool checkAllProcessed() const;
        vector<T> runDijkstra(T start, T target);
        vector<T> prettyPath(T start, T target) const;
        int getMinDist(T target) const;
        T findLowestCostNode() const;
    private:
        unordered_set<T> _nodes;
        unordered_set<T> _processed;
        unordered_map<T,T> _parents;
        outer_map<T> _map; // is the graph itself
        // hash table for costs @ each node processing step
        unordered_map<T,int> _costs;
        T _start_node;
};

#endif

