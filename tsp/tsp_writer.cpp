/*
 * tsp_writer.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: zxi
 */

#include "tsp_writer.h"

#include <iostream>
#include <fstream>
using namespace std;

bool TSPWriter::WriteTSP(const std::string& path, vector< vector< int >  > & matrix)
{

  //cerr << "- Output to " << path << endl;

  ofstream out(path);

  if (!out.good()) {
    cerr << "! Error: Failed to open file: " << path << endl;
    return false;
  }

  int size=matrix.size();

  out << "NAME: model" << endl;
  out << "TYPE: TSP" << endl;
  out << "DIMENSION: " << size << endl;
  out << "EDGE_WEIGHT_TYPE: EXPLICIT" << endl;
  out << "EDGE_WEIGHT_FORMAT: LOWER_DIAG_ROW" << endl;
  out << "EDGE_DATA_FORMAT: EDGE_LIST" << endl;
  out << "DISPLAY_DATA_TYPE: NO_DISPLAY" << endl;

  // Output edge list <-- this part is ignored by the Concorde solver...
  /*
  out << "EDGE_DATA_SECTION" << endl;
  for (int i = 0; i < model_->dual_graph_weights.size(); ++i) {
    for (int j = 0; j <= i; ++j) {
      const int w = model_->dual_graph_weights[i][j];
      if (w != 1)
        continue;
      out << i << " " << j << endl;
      out << j << " " << i << endl;
    }
  }
  out << "-1" << endl;
  */

  // Output weights
  out << "EDGE_WEIGHT_SECTION" << endl;


  for (int i = 0; i < size; ++i)
  {
    for (int j = 0; j <= i; ++j) {
      out << " " << matrix[i][j];
    }
    out << endl;
  }

  out << "EOF" << endl;

  out.close();

  return true;
}
