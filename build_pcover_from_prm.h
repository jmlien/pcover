#pragma once

#include "MyPCoverPlanner.h"


namespace GMUCS425
{

//build a graph from grid
inline bool build_graph_from_grid(MyPCoverPlanner& pcover, const mathtool::Point2d& start)
{
  using namespace std;
  using namespace mathtool;

  //check if the start is valid (i.e., inside the screen)
  if(start[0]<0 || start[1]>=getMyGame()->getScreenWidth()) return false;

  std::vector< std::vector<MyPCoverPlanner::Node> > grid(pcover.m_height, std::vector<MyPCoverPlanner::Node>(pcover.m_width,MyPCoverPlanner::Node()) );
  float cell_w=getMyGame()->getScreenWidth()*1.0f/pcover.m_width;
  float cell_h=getMyGame()->getScreenHeight()*1.0f/pcover.m_height;

  //TODO: go through the nodes, and init the data for each node
  int nid=0;
  for(int i=0;i<pcover.m_height;i++)
  {
    for(int j=0;j<pcover.m_width;j++)
    {
      MyPCoverPlanner::Node & n=grid[i][j];

      n.latest_valid_time=pcover.m_latency; //must be visited before the initial latency

      n.pos.set( cell_w*(j+0.5f), cell_h*(i+0.5f) );
      //check if the node is free of collision
      n.free = !pcover.collision_detection(n.pos);
      //cout<<"node ("<<j<<","<<i<<") is free="<<n.free<<endl;
      if(n.free) n.id=pcover.m_graph.add_node(n);
    }//end j
  }//end i

  //Make connections
  for(int i=0;i<pcover.m_height;i++)
  {
    for(int j=0;j<pcover.m_width;j++)
    {
      MyPCoverPlanner::Node & n=grid[i][j];
      if(!n.free) continue; //this node is in collision, no neighbors
      //connect the neighboring cells
      for(int dx=-1;dx<2;dx++)
      {
        int nj=j+dx;
        if(nj<0 || nj>=pcover.m_width) continue;
        for(int dy=-1;dy<2;dy++)
        {
          int ni=i+dy;
          if(ni<0 || ni>=pcover.m_height) continue;
          MyPCoverPlanner::Node & nei = grid[ni][nj];
          if(&nei==&n) continue; //the node itself

          if(pcover.collision_detection(n.pos, nei.pos)) continue; //not passable
          float dist=pcover.cost(n.pos, nei.pos);
          if(nei.free){
            pcover.m_graph.nodes[n.id].data.neighbors.push_back( make_pair(&pcover.m_graph.nodes[nei.id].data,dist));
            pcover.m_graph.add_edge(n.id, nei.id, dist);
          }
        }//end for dy
      }//end for dx
      //cout<<"n has "<<n.neighbors.size()<<" neis"<<endl;
    }//end j
  }//end i

  MyPCoverPlanner::Node * S=&grid[(int)(start[1]/cell_h)][(int)(start[0]/cell_w)];
  if(pcover.m_charging_station!=NULL) pcover.m_charging_station->b_charging_station=false;
  pcover.m_graph.nodes[S->id].data.b_charging_station = true;
  pcover.m_charging_station=&pcover.m_graph.nodes[S->id].data;

  return true;
}

}//end namespace
