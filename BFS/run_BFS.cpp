#include "Graph.hpp"
#include <iostream>
#include <string>

using namespace std;

int main()
{
    // Build the graph
    vector<string> peeps = {"John", "Jose", "Jenny", "Robert", "Rick", "Sally", "Beth"};
    Graph<string> myGraph(peeps);

    vector<string> myNeighbors = {"Jose", "Jenny", "Robert"};
    myGraph.addNodeNeighbors("John", myNeighbors);

    vector<string> joseNeighbors = {"Rick", "Sally"};
    vector<string> jennyNeighbors = {"Beth", "Robert"};
    myGraph.addNodeNeighbors("Jose", joseNeighbors);
    myGraph.addNodeNeighbors("Jenny", jennyNeighbors);

    string target = "Beth";

    cout << "Running BFS, looking for " << target << endl;
    cout << boolalpha;
    myGraph.runBFS("John", target);

    return 0;
}