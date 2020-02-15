/*
 * tsp_writer.h
 *
 *  Created on: Dec 19, 2016
 *      Author: zxi
 */

#ifndef SRC_APP_HAMILTONIAN_TSP_WRITER_H_
#define SRC_APP_HAMILTONIAN_TSP_WRITER_H_

#include <string>
#include <vector>

class TSPWriter
{
public:

  TSPWriter() {}
  ~TSPWriter() {}

  // Write the tsp problem into a file in TSP format, return true if success, false otherwise.
  bool WriteTSP(const std::string& path, std::vector< std::vector< int >  > & matrix);

};

#endif /* SRC_APP_HAMILTONIAN_TSP_WRITER_H_ */
