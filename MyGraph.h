#pragma

#include "mathtool/Point.h"

namespace GMUCS425
{

//a simple dummy graph
class MyBasicGraph {

  struct Node{
    int id;
    mathtool::Point2d pos;
  };

  int add_node(const mathtool::Point2d& pos){ Node n; n.pos=pos; n.id=nodes.size(); nodes.push_back(n); return n.id; }
  void add_edge(int s, int e, float weight){ edges.push_back( make_pair(make_pair(s,e),weight)); }

  vector< pair< pair<int,int>, float> > edges;
  vector<Node> nodes;
};


}//end namespace GMUCS425
