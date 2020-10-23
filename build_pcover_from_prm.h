#pragma once

#include "MyPCoverPlanner.h"


namespace MASC_PCOVER
{

//build a graph from grid
inline bool build_graph_from_prm(MyPCoverPlanner& pcover, const mathtool::Point2d& start)
{
  using namespace std;
  using namespace mathtool;
  float sensor_w=pcover.getSensorWidth();
  float sensor_h=pcover.getSensorHeight();
  float screen_w=getMyGame()->getScreenWidth();
  float screen_h=getMyGame()->getScreenHeight();

  auto addnode = [&](const Point2d& pos){
    MyPCoverPlanner::Node n;
    n.pos=pos;
    n.latest_valid_time=pcover.m_latency; //must be visited before the initial latency
    n.free =true;
    int id=pcover.m_graph.add_node(n);
    pcover.m_graph.nodes[id].data.id=id;
  };

  if(pcover.collision_detection(start)) {
    cerr<<"! Error: build_graph_from_prm start="<<start<<" is in collision"<<endl;
    return false;
  }
  addnode(start);

  auto tooclose = [&](const Point2d& pos)->bool
  {
    for(auto& node : pcover.m_graph.nodes){
      Vector2d vec=node.data.pos-pos;
      if( fabs(vec[0])<sensor_w && fabs(vec[1])<sensor_h ) return true;
    }
    return false;
  };

  auto tooclose2e = [&](const Point2d& pos,int i, int j)->bool
  {
    for(auto& node : pcover.m_graph.nodes){
      if(node.data.id==i || node.data.id==j) continue; //ignore
      Vector2d vec=node.data.pos-pos;
      if( fabs(vec[0])<sensor_w && fabs(vec[1])<sensor_h ) return true;
    }
    return false;
  };



  //check if the start is valid (i.e., inside the screen)
  if(start[0]<0 || start[1]>=screen_w) return false;

  //create (pcover.m_height * pcover.m_width samples)/2
  int needed_samples=pcover.m_height * pcover.m_width/2;
  int sampled_size=0;
  while(sampled_size<needed_samples)
  {
    Point2d pos(sensor_w/4+ (screen_w-sensor_w/4)*drand48(),sensor_h/4+ (screen_h-sensor_h/4)*drand48());
    if(pcover.collision_detection(pos)) continue;
    if(tooclose(pos)) continue;
    addnode(pos);
    sampled_size++;
  }

  //add connections
  const int K=3;
  sampled_size=pcover.m_graph.nodes.size();
  for(int i=0;i<sampled_size;i++){
    auto & node_i=pcover.m_graph.nodes[i].data;
    vector< pair<double,int> > K_nearest;
    for(int j=0;j<sampled_size;j++){
      if(i==j) continue;
      auto & node_j=pcover.m_graph.nodes[j].data;
      //check if node[i] and node[j] can be connected without getting too close to other nodes
      Vector2d vec=(node_i.pos-node_j.pos);
      K_nearest.push_back({vec.normsqr(), j});
    }
    sort(K_nearest.begin(),K_nearest.end());
    int k=std::min(K,(int)K_nearest.size());
    while( (k--)>0 ){
      int j=K_nearest[k].second;
      auto & node_j=pcover.m_graph.nodes[ j ].data;
      Vector2d vec=(node_j.pos-node_i.pos);
      float dist=vec.norm();
      int ticks=20;
      vec=vec/ticks;
      //float step_size=1;
      //vec=vec/dist;
      //int ticks=(int)(dist/step_size);
      bool valid=true;
      for(int step=1;step<ticks;step++){
        Point2d pos=node_i.pos+vec*step;
        if(pcover.collision_detection(pos)){ valid=false; break; }
        //if(tooclose2e(pos, node_i.id, node_j.id)) {valid=false; break;}
      }
      if(!valid){continue;}//not a valid connection

      dist=pcover.dist2time(dist);
      node_i.neighbors.push_back( make_pair(&node_j,dist));
      node_j.neighbors.push_back( make_pair(&node_i,dist));
      pcover.m_graph.add_edge(i, j, dist);
    }
  }

  //find which node is closest to S
  MyPCoverPlanner::Node * S=NULL;
  float D=FLT_MAX;
  for(int i=0;i<sampled_size;i++){
    auto & node=pcover.m_graph.nodes[i].data;
    float d=(node.pos-start).normsqr();
    if(d<D){
      D=d;
      S=&node;
    }
  }

  cout<<"- Created a graph using PRM with "<<sampled_size<<" nodes and "<<pcover.m_graph.edges.size()<<" edges"<<endl;

  if(S!=NULL){
    if(pcover.m_charging_station!=NULL) pcover.m_charging_station->b_charging_station=false;
    S->b_charging_station = true;
    pcover.m_charging_station=S;
    return true;
  }

  return false; //S==NULL
}

}//end namespace
