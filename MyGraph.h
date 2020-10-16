#pragma

#include "mathtool/Point.h"

namespace GMUCS425
{

//a simple dummy graph
template<typename T>
class MyBasicGraph {
  
public:
  struct Node{
    T data;
    int id;
  };

  int add_node(const T& data){ Node n; n.data=data; n.id=nodes.size(); nodes.push_back(n); return n.id; }
  void add_edge(int s, int e, float weight){ edges.push_back( make_pair(make_pair(s,e),weight)); }

  vector< pair< pair<int,int>, float> > edges;
  vector<Node> nodes;
};


}//end namespace GMUCS425
