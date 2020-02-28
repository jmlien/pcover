/*
 * Hamiltonian.cpp
 *
 *  Created on: Oct 9, 2016
 *      Author: zxi
 */

#include "GraphTSP.h"

#include <iostream>
#include <fstream>
#include <assert.h>
#include <random>       // std::default_random_engine
#include <map>
#include <sstream>
#include <cstdlib>

//#include <opencv2/imgproc.hpp>
using namespace std;

//#include "mathtool/Box.h"
#include "tsp_writer.h"

void GraphTSP::FindTSPConcorde(vector< vector< int >  > & matrix, //the graph
                     vector<GraphTSP::TSPPath> &paths, //output
                     const int max_paths)
{

  vector<string> path_strings;

  const string tsp_file_path = "pcover.tsp";
  TSPWriter tsp_writer;
  tsp_writer.WriteTSP(tsp_file_path,matrix);

  string binary_posfix;

#ifdef _WIN32
  binary_posfix = ".exe";
#elif __APPLE__
  binary_posfix = ".osx";
#elif __linux__
  binary_posfix = ".linux64";
#endif

  const string path_to_solver = "./tsp/concorde/concorde" + binary_posfix;
  const int max_failed_attempts=500;
  int failed_attempts=0;

  for (int i = 0; i < 100 * max_paths; ++i)
  {

    if(failed_attempts>=max_failed_attempts) break;

    const string clock_str = std::to_string(std::rand()); //clock());
    const string solution_path = "tsp_solution_" + clock_str + ".txt";
    const string cmd_output = "tsp_solution_" + clock_str + "_dump.txt";
    const string init_upper_bound = std::to_string(matrix.size());

    const map<string, string> args = { /***** {Key, Value} ******/
    //{ "-q", "" }, /* do not cut the root lp */
    { "-x", "" }, /* delete files on completion (sav pul mas) */
    { "-V", "" }, /* just run fast cuts */
        { "-u", init_upper_bound }, /* init upperbound */
        { "-s", clock_str }, /* random seed */
        { "-o", solution_path } /* solution pah */
    };

    string paramters;

    for (const auto& kv : args) {
      paramters += kv.first + " " + kv.second + " ";
    }

    paramters += tsp_file_path;

    string command = path_to_solver + " " + paramters;

    //cerr << "[concorde] Running command: " << command << endl;

    // Run external solver
    command=command+" &> "+cmd_output;
    int rtn = system(command.c_str());
    std::remove(cmd_output.c_str());

    ifstream in(solution_path);

    if (!in.good()) {
      cerr << "[concorde] !Error: Failed to solve the TSP problem!" << endl;
      continue;
    }

    int n;
    in >> n;

    assert(n == matrix.size());

    int node;
    TSPPath path;
    for (int i = 0; i < n; ++i) {
      in >> node;
      path.push_back( make_pair(node,0) );
    }

    //update weights
    for(auto it=path.begin(); it!=path.end(); it++)
    {
      auto next=it; next++;
      if(next==path.end()) next=path.begin();
      it->second = matrix[it->first][next->first];
    }

    //check if the path is unique
    string ps = toString(path);
    bool found=false;
    for(string & ps2 : path_strings)
    {
      if(ps2==ps){ found=true; break; }
    }

    // Delete the solution file.
    std::remove(solution_path.c_str());

    if(found) {
      cout<<"- found duplicates: "<<ps<<endl;
      failed_attempts++;
      continue; //duplicated path
    }

    paths.push_back(path);
    path_strings.push_back(ps);
    failed_attempts=0;

    if (paths.size() >= max_paths)
      break;
  }

  //std::remove(tsp_file_path.c_str());

  //cout<<"TSP DONE: "<<paths.size()<<" paths found"<<endl;
  return;
}

string GraphTSP::toString(GraphTSP::TSPPath& path)
{
  std::stringstream ss;
  for(auto& n : path) ss<<n.first<<"-";
  return ss.str();
}
///////////////////////////////////////////////////
