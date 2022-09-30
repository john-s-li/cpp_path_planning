#include "Graph.cpp"
#include <iostream>

using namespace std;

int main()
{
    vector<string> nodes = {"start", "A", "B", "finish"};
    Graph<string> myGraph(nodes, "start");

    unordered_map<string, int> startNeighbors;
    startNeighbors["A"] = 6;
    startNeighbors["B"] = 2;
    myGraph.addNodeNeighborsAndCost("start", startNeighbors);

    unordered_map<string, int> ANeighbors;
    ANeighbors["finish"] = 1;
    myGraph.addNodeNeighborsAndCost("A", ANeighbors);

    unordered_map<string, int> BNeighbors;
    BNeighbors["A"] = 3;
    BNeighbors["finish"] = 5;
    myGraph.addNodeNeighborsAndCost("B", BNeighbors);

    unordered_map<string, int> finishNeighbors;
    myGraph.addNodeNeighborsAndCost("finish", finishNeighbors); // need to add so node "exists"

    string target = "finish";
    vector<string> path = myGraph.runDijkstra("start", target);
    cout << "Path to " << target << " is " << endl;
    for(string node : path)
    {
        cout << node << " -> "; 
    }
    cout << "Done!" << endl;
    cout << "Minimum Distance = " << myGraph.getMinDist(target) << endl;

    return 0;
}