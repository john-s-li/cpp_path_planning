#ifndef GRIDWORLD_H
#define GRIDWORLD_H

#include <vector>
#include <unordered_map>
#include <list>
#include <memory>
#include <limits>

using namespace std;

typedef pair<int, int> position;

const int ROWS = 9;
const int COLS = 10;

const double MAX = numeric_limits<double>::max();

class Node{
    public: 
        Node(position pos, double g, double h) :
            _position(pos),
            _g(g),
            _h(h) {}

        bool operator== (const Node* node) const {
            return _position == node->getPosition();
        }
        Node* getParent() const { return _parent; }
        position getPosition() const { return _position; }
        pair<double, double> getCosts() const { return make_pair(_g, _h); }

        void setCosts(double cost_g, double cost_h) { 
            _g = cost_g; 
            _h = cost_h; }
        
        void setParent(Node* new_parent) {
          _parent = new_parent;
        }

    private:
        Node* _parent;
        position _position;
        double _g; // exact cost
        double _h; // heuristic value
}; // end Node

ostream& operator<< (ostream& out, const Node& node) {
  position node_pos = node.getPosition();
  return out << "(" << node_pos.first << ", " << node_pos.second << ")";
} // for printing node easier

class GridWorld {
    public:
        GridWorld() {
          for (int r = 0; r < ROWS; r++) {
            vector<Node*> nodes_to_add;
            for (int c = 0; c < COLS; c++) {
              position coord = make_pair(r, c);
              // TODO: fill in these values later
              double g = MAX;
              double h = MAX;
              Node* new_node_ptr = new Node(coord, g, h);
              nodes_to_add.push_back(new_node_ptr);
            }
              _node_objs_vec.push_back(nodes_to_add);
          }
        }

        bool isValid(int r, int c) const {
          return (r >= 0 && r < ROWS ) && 
                 (c >= 0 && c < COLS) && 
                  _grid[r][c] == 1;
        }

        vector<Node*> getNeighbors(const Node* node) const {
          // returns up to 8 neighbors of the current point on the grid
          vector<Node*> neighbors;
          position curr_node_pos = node->getPosition();
          for (auto move: _valid_moves) {
            int delta_r = curr_node_pos.first + move.first;
            int delta_c = curr_node_pos.second + move.second;
            if (isValid(delta_r, delta_c)) {
              Node* node_ptr = _node_objs_vec[delta_r][delta_c];
              neighbors.push_back(node_ptr);
            }
          }

          return neighbors;
        }

        Node* getNode(position pos) const {
            int r = pos.first;
            int c = pos.second;
            return _node_objs_vec[r][c];
        }

        int size() const {
            return ROWS * COLS;
        }
        
    private:
        int _grid[ROWS][COLS] = 
        {
            { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
            { 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 },
            { 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 },
            { 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 },
            { 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 },
            { 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 },
            { 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
            { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
            { 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 }
        };

        vector<vector<Node*>> _node_objs_vec;

        vector<pair<int, int>> _valid_moves = {
            make_pair(-1,  0), // up
            make_pair( 1,  0), // down
            make_pair( 0,  1), // right
            make_pair( 0, -1), // left
            make_pair(-1, -1), // ne
            make_pair(-1,  1), // nw
            make_pair( 1, -1), // se
            make_pair( 1,  1)  // sw
        };
};

#endif