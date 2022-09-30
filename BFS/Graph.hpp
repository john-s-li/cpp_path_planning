#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>

using namespace std;

template <class T> 
class Graph 
{
    public:
        Graph(vector<T> nodeList);
        void addNodeNeighbors(T val, vector<T> Ns);
        bool runBFS(T start, T target) const;
    private:
        unordered_map<T, vector<T> > map;
};

template <class T>
Graph<T>::Graph(vector<T> nodeList)
{
    for(T node : nodeList)
    {
        vector<T> empty;
        this->map[node] = empty;
    }
}

template <class T>
void Graph<T>::addNodeNeighbors(T val, vector<T> Ns)
{
    // Look for this node
    if(this->map.find(val) == this->map.end()) // Does exist
    {
        // if not exist, add it to hash table
        vector<T> toAdd;
        this->map[val] = toAdd;
    }

    for (auto N : Ns)
    {
        // If the neighbors also are not in the hash table, add them
        this->map.at(val).push_back(N);
    }
}

template <class T>
bool Graph<T>::runBFS(T start, T target) const
{
    T curr; // temp var

    // Set up the queue
    queue<T> to_visit;
    unordered_set<T> visited; // for O(1) look up, can't random access a queue's elements

    // Add the first node into the queue
    if(this->map.find(start) == map.end())
    {
        std::cout << "Start val not in graph. Please try again.\n";
        return false;
    }

       if(this->map.find(target) == map.end())
    {
        std::cout << "Target val not in graph. Please try again.\n";
        return false;
    }

    to_visit.push(start);

    while(!to_visit.empty())
    {
        curr = to_visit.front();
        to_visit.pop();

        // Check if node value is target
        if(curr == target)
        {
            std::cout << "Found target!\n";
            std::cout << "Target value = " << curr << endl;
            return true;
        }

        visited.insert(curr);

        // Enqueue the node's neighbors
        for(auto neighbor : this->map.at(curr))
        {
            // if neighbor already visited, don't add
            if (visited.find(neighbor) != visited.end())
            {
                continue;
            }
            // Add neighbors
            to_visit.push(neighbor);
        }

        // Not target ... dequeue from to_visit and that's new head for next iteration
    }

    return false;
}

#endif
