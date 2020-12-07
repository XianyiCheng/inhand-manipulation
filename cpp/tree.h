#include <vector>
#include <cfloat>
#include <cstdlib>
#include "utilities.h"

#define NUM_FINGERS 4
double dist(double q1[7], double q2[7]);
struct Node {
    double config[7];
    int finger_locations[NUM_FINGERS];
    int parent = -1;
    std::vector<int> children;
    double cost = 0;
    Node(double data[7], int finger_locations_[NUM_FINGERS]){
        for(int i = 0; i<7; i++){
            config[i] = data[i];
        }
        for(int i = 0; i<NUM_FINGERS; i++){
            finger_locations[i] = finger_locations_[i];
        }
      }
};

class Tree{
public:
  std::vector<Node> nodes;
  int nearest_neighbor(double q[7]);
  void nearest_neighbors(double q[7], std::vector<int>* nearest_neighbors);
  void backtrack(int last_node_idx, std::vector<int>* node_path);
  void neighborhood(int node_idx, double radius, std::vector<int>* neighbors);
  void add_node(Node* n, int parent);
  void initial_node(Node* n);
  void remove_parent(int node_idx);
  void set_parent(int parent, int child);
};
