#pragma once

#include "MyScene.h"

namespace GMUCS425
{

class MyPathPlanner
{
public:

  typedef mathtool::Point2d Point2d;

  MyPathPlanner(MyScene * scene, MyAgent * agent);
  virtual ~MyPathPlanner(){}

  virtual bool build()=0; //initialize the planner

  //finding a path
  virtual bool find_path( const Point2d& start, const Point2d& goal, std::list<Point2d>& path )=0;

  //TODO: shorten and smooth the path
  void smooth(std::list<Point2d>& path);

protected:

  //TODO:
  //return true if m_agent collide with a non-movable object
  //at a given location
  bool collision_detection(const Point2d& pos);

  bool collision_detection(const Point2d& pos1, const Point2d& pos2);

  //TODO:
  //estimate the cost of travelling from pos1 to pos2
  //using the values generate by Perlin noise
  float cost(const Point2d& pos1, const Point2d& pos2);

  MyAgent * m_agent;
  MyScene * m_scene;
};


class MyGridPathPlanner : public MyPathPlanner
{
public:

  MyGridPathPlanner(MyScene * scene, MyAgent * agent, int W, int H)
  :MyPathPlanner(scene,agent){
    m_width=W;
    m_height=H;
  }

  virtual ~MyGridPathPlanner(){
    m_grid.clear();
  }

  virtual bool build(); //build a grid

  //finding a path using A*
  virtual bool find_path( const Point2d& start, const Point2d& goal, std::list<Point2d>& path );

protected:

  int m_width, m_height; //width and height of the grid

  struct Node
  {
    Node(){ f=g=FLT_MAX; parent=NULL; visited=false; free=true; }
    Point2d pos; //location of the node
    float f, g; //f is the total cost, g is to cost to node
    list<Node *> neighbors;
    Node * parent;
    bool visited;
    bool free; //is this node free of obstacles?
  };

  std::vector< std::vector<Node> > m_grid; //a grid for motion planning
  static bool comp(Node * a, Node * b){ return a->f>b->f;}
};

}//end namespace GMUCS425
