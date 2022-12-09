#include <vector>
#include <iostream>

#include "a_star_helper.h"

using namespace std;

ostream& operator<< (ostream& out, AStarHelper::node_ptr node) {
  return out << "(" << node->x << ", " << node->y << ")";
}

const vector<pair<int, int>> AStarHelper::Params::motion_set = {
  make_pair(-1,  0),
  make_pair(-1,  1),
  make_pair( 0,  1),
  make_pair( 1,  1),
  make_pair( 1,  0),
  make_pair( 1, -1),
  make_pair( 0, -1),
  make_pair(-1, -1),
};

heuristic_map AStarHelper::calc_holonomic_heuristic_with_obs(
  float node_x, float node_y, const vector<int> obs_x, const vector<int> obs_y, 
  float reso, float robot_radius) {

  node_ptr n_goal = make_shared<Node>(round(node_x / reso),
                                      round(node_y / reso), 
                                      0.0, -1.0);

  vector<float> ox, oy;
  for (int i = 0; i < obs_x.size(); i++) {
    // cout << "Obs: (" << obs_x[i] / reso << " , " << obs_y[i] / reso << ")\n";
    ox.push_back(obs_x[i] / reso);
    oy.push_back(obs_y[i] / reso);
  }
  
  params_ptr params = calc_params(ox, oy, robot_radius, reso);
  obs_map o_map(params->x_width, vector<bool>(params->y_width, false));
  calc_obs_map(o_map, ox, oy, robot_radius, params);

  // for(auto r: o_map) {
  //   for(auto c: r) {
  //     cout << c << " ";
  //   }
  //   cout << endl;
  // }

  AStarHelper::node_map open_set, closed_set;
  open_set[calc_index(n_goal, params)] = n_goal;

  priority_queue<node_idx_with_cost,
                 vector<node_idx_with_cost>,
                 greater<node_idx_with_cost>> node_pq;

  node_pq.push(make_tuple(n_goal->cost, calc_index(n_goal, params)));

  while(true) {
    if (open_set.empty()) break;
    
    auto [curr_cost, curr_node_idx] = node_pq.top();
    node_pq.pop();
    node_ptr curr_node = open_set[curr_node_idx];
    closed_set[curr_node_idx] = curr_node;
    open_set.erase(curr_node_idx);

    for (auto &m: Params::motion_set) {
      node_ptr cand_node = make_shared<Node>(
                                          curr_node->x + m.first,
                                          curr_node->y + m.second,
                                          curr_node->cost + movement_cost(m),
                                          curr_node_idx);
                                
      if (!check_node(cand_node, params, o_map)) continue;

      float cand_node_idx = calc_index(cand_node, params);

      // node not visited before
      if (closed_set.find(cand_node_idx) == closed_set.end()) {
        if (open_set.find(cand_node_idx) != open_set.end()) {
          if (open_set[cand_node_idx]->cost > cand_node->cost) {
            open_set[cand_node_idx]->cost = cand_node->cost;
            open_set[cand_node_idx]->parent_idx = curr_node_idx;
          }
        }
        else { // node visited
          open_set[cand_node_idx] = cand_node;
          node_pq.push(
            make_tuple(cand_node->cost, calc_index(cand_node, params))
          );
        }
      }
    }
  }

  float INF = numeric_limits<float>::infinity();

  heuristic_map hmap(params->x_width,
                     vector<float>(params->y_width, INF));

  for (auto p: closed_set) {
    node_ptr n = p.second;
    hmap[int(n->x - params->min_x)][int(n->y - params->min_y)] = n->cost;
  }

  return hmap;
}

AStarHelper::params_ptr AStarHelper::calc_params(
  const vector<float> ox, const vector<float> oy,
  float robot_radius, float reso) {

  float min_x = round(*min_element(ox.begin(), ox.end())); 
  float min_y = round(*min_element(oy.begin(), oy.end()));
  float max_x = round(*max_element(ox.begin(), ox.end())); 
  float max_y = round(*max_element(oy.begin(), oy.end()));

  cout << "A* Helper Params: \n";
  cout << "min_x = " << min_x << endl;
  cout << "min_y = " << min_y << endl;
  cout << "max_x = " << max_x << endl;
  cout << "max_y = " << max_y << endl;

  float xw = max_x - min_x, yw = max_y - min_y;

  return make_shared<Params>(min_x, min_y, max_x, max_y,
                             int(xw), int(yw), reso);
}

void AStarHelper::calc_obs_map(obs_map &omap,
                               const vector<float> obs_x,
                               const vector<float> obs_y,
                               float robot_radius,
                               params_ptr P) {

  for(int x = 0; x < P->x_width; x++) {
    float xx = x + P->min_x;
    for(int y = 0; y < P->y_width; y++) {
      float yy = y + P->min_y;
      for (int i = 0; i < obs_x.size(); i++) {
        float oxx = obs_x[i], oyy = obs_y[i];
        if (hypot<float>(oxx - xx, oyy - yy) <= robot_radius / P->reso) {
          omap[x][y] = true;
          break;
        }
      }
    }
  }
}

float AStarHelper::calc_index(node_ptr node, params_ptr P) {
  return (node->y - P->min_y) * P->x_width + (node->x - P->min_x);
}

float AStarHelper::movement_cost(pair<int, int> motion) {
  return hypot<float>(motion.first, motion.second);
}

bool AStarHelper::check_node(node_ptr node, params_ptr P, obs_map omap) {
  if (node->x <= P->min_x || node->x >= P->max_x ||
      node->y <= P->min_y || node->y >= P->max_y) {
    cout << node << " out of bounds" << endl;
    return false;
  }

  if (omap[int(node->x - P->min_x)][int(node->y - P->min_y)]) {
    cout << node << " in obstacle" << endl;
    return false;
  }

  return true;
}