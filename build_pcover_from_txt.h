#pragma once

#include "MyPCoverPlanner.h"
#include <fstream>
#include "mathtool/Box.h"

namespace MASC_PCOVER
{

  //
  //
  // //overall number of vertices, where
  // // nrVertices = 1 + nrChargeVertices + nrInspectVertices + nrOtherVertices
  // // 1 is for the initial vertex
  // nrVertices
  //
  // //id of the initial vertex and its position
  // vidInit vidInitx vidInity
  //
  // //overall number of charge vertices followed by
  // //id and position of each charge vertex
  // nrChargeVertices
  // vid1 pos1x pos1y
  // vid2 pos2x pos2y
  // ...
  //
  // //overall number of inspection vertices followed by
  // //id, position, and region id for each inspection vertex
  // nrInspectVertices
  // vid1 pos1x pos1y idRegion1
  // vid2 pos2x pos2y idRegion2
  // ...
  //
  // //overall number of other vertices followed by
  // //id and position for each vertex
  // nrOtherVertices
  // vid1 pos1x pos1y
  // vid2 pos2x pos2y
  // ...
  //
  // //number of edges followed by
  // //idFrom, idTo and distance for each edge (idFrom, idTo)
  // //graph is undirected, so edge appears only once with idFrom < idTo
  // nrEdges
  // vidFrom1 vidTo1 distance
  // ...
  //
  //
  //

//build a graph from grid
inline bool build_graph_from_txt(MyPCoverPlanner& pcover, const string& filename)
{
  using namespace mathtool;

  int nrVertices, nrChargeVertices, nrInspectVertices, nrOtherVertices;
  int nrEdges;
  auto& G=pcover.m_graph;
  unordered_map<int,int> vidmap;

  std::ifstream fin;
  fin.open(filename);
  if (!fin.is_open()) // oops. there was a problem opening the file
  {
    std::cerr << "ERROR: FILE ("<<filename<<") COULD NOT BE OPENED" << std::endl;	// Hmm. No output?
    return false;
  }

cout<<"filename="<<filename<<endl;

  //read vertices
  MyPCoverPlanner::Node start;
  fin>>nrVertices>>start.id>>start.pos[0]>>start.pos[1];
  start.b_charging_station=true;
  start.b_inspect=true;
  vidmap[start.id]=G.add_node(start);
  pcover.m_charging_station=&G.nodes[vidmap[start.id]].data;

  fin>>nrChargeVertices;
  for(int i=0;i<nrChargeVertices;i++){
    MyPCoverPlanner::Node n;
    fin>>n.id>>n.pos[0]>>n.pos[1];
    n.b_charging_station=true;
    n.b_inspect=false;
    vidmap[n.id]=G.add_node(n);
  }

  fin>>nrInspectVertices;
  for(int i=0;i<nrInspectVertices;i++){
    MyPCoverPlanner::Node n;
    int region_id;
    fin>>n.id>>n.pos[0]>>n.pos[1]>>region_id;
    n.b_charging_station=false;
    n.b_inspect=true;
    vidmap[n.id]=G.add_node(n);
  }

  fin>>nrOtherVertices;
  for(int i=0;i<nrOtherVertices;i++){
    MyPCoverPlanner::Node n;
    fin>>n.id>>n.pos[0]>>n.pos[1];
    n.b_charging_station=false;
    n.b_inspect=false;
    vidmap[n.id]=G.add_node(n);
  }

  //read edges
  fin>>nrEdges;

  int s,e;
  float w;
  for(int i=0;i<nrEdges;i++){
    fin>>s>>e>>w;
    s=vidmap[s];
    e=vidmap[e];
    G.add_edge(s,e,w);
    G.add_edge(e,s,w);
    auto& S=G.nodes[s].data;
    auto& E=G.nodes[e].data;
    S.neighbors.push_back( {&E, w } );
    E.neighbors.push_back( {&S, w } );
  }

  //reset the ids and compute the bounding box
  std::vector<Vector2d> points;
  for(int i=0;i<G.nodes.size();i++){
    G.nodes[i].data.id=i;
    points.push_back(Vector2d(G.nodes[i].data.pos.get()));
  }
  Box2d bbox;
  bbox=bbox.setFromPoints(points);

  //add boundary to the box
  bbox.x-=bbox.width/100;
  bbox.y-=bbox.height/100;
  bbox.width+=bbox.width/50;
  bbox.height+=bbox.height/50;

  //scale all nodes so they fit to the screen...
  for(auto& data : G.nodes){
    auto& node = data.data;
    node.pos[0]-=bbox.x;
    node.pos[1]-=bbox.y;
  }
  //cout<<"box="<<bbox<<endl;

  return true;
    /*
  //check if the start is valid (i.e., inside the screen)
  if(start[0]<0 || start[1]>=getMyGame()->getScreenWidth()) return false;

  std::vector< std::vector<MyPCoverPlanner::Node> > grid( m_height, std::vector<MyPCoverPlanner::Node>(pcover.m_width,MyPCoverPlanner::Node()) );
  float cell_w=getMyGame()->getScreenWidth()*1.0f/pcover.m_width;
  float cell_h=getMyGame()->getScreenHeight()*1.0f/pcover.m_height;

  //TODO: go through the nodes, and init the data for each node
  int nid=0;
  for(int i=0;i<pcover.m_height;i++)
  {
    for(int j=0;j<pcover.m_width;j++)
    {
      MyPCoverPlanner::Node & n=grid[i][j];
      n.pos.set( cell_w*(j+0.5f), cell_h*(i+0.5f) );
      //check if the node is free of collision
      n.free = !collision_detection(n.pos);
      //cout<<"node ("<<j<<","<<i<<") is free="<<n.free<<endl;
      if(n.free) n.id=m_graph.add_node(n);
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
        if(nj<0 || nj>=m_width) continue;
        for(int dy=-1;dy<2;dy++)
        {
          int ni=i+dy;
          if(ni<0 || ni>=m_height) continue;
          MyPCoverPlanner::Node & nei = grid[ni][nj];
          if(&nei==&n) continue; //the node itself

          if(pcover.collision_detection(n.pos, nei.pos)) continue; //not passable
          float dist=pcover.cost(n.pos, nei.pos);
          if(nei.free){
            m_graph.nodes[n.id].data.neighbors.push_back( make_pair(&m_graph.nodes[nei.id].data,dist));
            m_graph.add_edge(n.id, nei.id, dist);
          }
        }//end for dy
      }//end for dx
      //cout<<"n has "<<n.neighbors.size()<<" neis"<<endl;
    }//end j
  }//end i

  MyPCoverPlanner::Node * S=&grid[(int)(start[1]/cell_h)][(int)(start[0]/cell_w)];
  if(m_charging_station!=NULL) m_charging_station->b_charging_station=false;
  m_graph.nodes[S->id].data.b_charging_station = true;
  m_charging_station=&m_graph.nodes[S->id].data;
*/

  return true;
}

} //namespace MASC_PCOVER
