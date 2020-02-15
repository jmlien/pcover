/*
 * Hamiltonian.h
 *
 *  Created on: Oct 9, 2016
 *      Author: zxi
 */

#ifndef SRC_UTIL_HAMILTONIAN_H_
#define SRC_UTIL_HAMILTONIAN_H_

#include <vector>
#include <set>
#include <unordered_set>
#include <random>
using namespace std;

class GraphTSP
{
public:

  typedef vector<pair<int, float>> TSPPath; //vid, weight

  // Find random Hamiltonian paths for the given mesh up to max_paths
  //(if possible) using Concorde library.
  void FindTSPConcorde(vector< vector< int >  > & matrix, //the graph
                       vector<TSPPath> &paths, //output
                       const int max_paths);

  string toString(TSPPath& path);
  std::mt19937 random_g_;
};

#endif /* SRC_UTIL_HAMILTONIAN_H_ */
