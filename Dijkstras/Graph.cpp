#include "Graph.hpp"

using namespace std;

template <class T>
Graph<T>::Graph(vector<T> nodeList, T start_node)
{
    _start_node = start_node;

    for(T node : nodeList) // O(N)
    {   
        _nodes.insert(node);
    }
}


template <class T>
void Graph<T>::addNodeNeighborsAndCost(T node, inner_map<T> NandC)
{
    // Add neighbors and the respective cost into node's hash table
    _map[node] = NandC;
}


template <class T>
bool Graph<T>::checkValid(T value) const
{
    if(_map.find(value) == _map.end()) // O(1)
    {
        std::cout << "Value Node not in graph. Please try again.\n";
        return false;
    }
    return true;
}


template <class T>
T Graph<T>::findLowestCostNode() const
{
    int min = INF;
    T minNode;
    // Loops through the cost dictionary returns the node with current lowest cost

    for(auto node : _nodes) // O(N)
    {
        if (node == _start_node) continue;
       
        if(_costs.at(node) < min && 
                _processed.find(node) == _processed.end())
        {
            min = _costs.at(node); // O(1)
            minNode = node; // O(1)
        }
    }
    return minNode;
}

template <class T>
void Graph<T>::initCosts()
{
    // For all the nodes that cannot be seen, just set them to infinity
    for(auto node : _nodes)
    {
        if (node == _start_node) continue;

        // not in the immediate neighbors of the start node
        if((_map.at(_start_node)).find(node) == (_map.at(_start_node)).end())
        {
            _costs[node] = INF; // can't see the destination
        }
        else // can see it
        {
            _costs[node] = _map.at(_start_node).at(node);
        }
    }
}

template <class T>
void Graph<T>::buildParents(T start)
{
    for(auto neighbor : _map.at(start))
    {
        _parents[neighbor.first] = start;
    }
}


template <class T>
vector<T> Graph<T>::prettyPath(T start, T target) const
{
    vector<T> path;
    T curr = target;
    path.push_back(curr);

    while(curr != start)
    {
        // Get the parent of curr and append to path
        if(_parents.find(curr) == _parents.end())
        {
            throw runtime_error("Parent link not existent.");
        }
        curr = _parents.at(curr);
        path.insert(path.begin(), curr);
    }

    return path;
}

template <class T>
int Graph<T>::getMinDist(T target) const
{
    return _costs.at(target);
}

template <class T> 
bool Graph<T>::checkAllProcessed() const
{
    return _processed.size() == _nodes.size();
}

template <class T>
vector<T> Graph<T>::runDijkstra(T start, T target)
{
    int cost;
    int new_cost;
    T minNode;

    // Check to see if the start and targets are in the graph
    if(!checkValid(start) && !checkValid(target)) 
    {
        throw invalid_argument("Either start or target node not in graph.");
    }

    // Build the costs
    initCosts();
    // Build parents
    buildParents(_start_node);
    _processed.insert(_start_node);

    // Get the neighbors of the start node and then start on the cheapest neighbor
    minNode = findLowestCostNode();

    while(true)
    {
        cost = _costs.at(minNode);

        // Update costs for neighbors
        for (auto neighbor: _map.at(minNode))
        {
            new_cost = cost + neighbor.second;
            if (new_cost < _costs.at(neighbor.first) &&
                _processed.find(neighbor.first) == _processed.end())
            {
                _costs[neighbor.first] = new_cost;
                // Update the parent as well
                _parents[neighbor.first] = minNode;
            }
        }

        // Now the min node is processed
        _processed.insert(minNode);
      
        if(checkAllProcessed()) break;
        // get the next min cost node
        minNode = findLowestCostNode();
        
        // Add new parents & new costs for this new minNode
        buildParents(minNode);
    }

    // Get the cleaned up path
    return prettyPath(start, target);
}

