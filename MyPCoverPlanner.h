#pragma once

#include "MyScene.h"
#include <set>

namespace GMUCS425
{

class MyDragonAgent;

class MyPCoverPlanner
{

protected:

  typedef mathtool::Point2d Point2d;

  struct Node
  {
      Node()
      {
        id=-1;
        //f=g=FLT_MAX;
        parent=NULL;
        visited=false;
        free=true;
        b_charging_station=false;
        t=0;
        time2station=dist2station=0;
      }

      int id;
      Point2d pos; //location of the node
      //float f, g; //f is the total cost, g is to cost to node

      float dist2station; //distance to charging station
      float time2station; //time to charging station
      list<Point2d> path2station; //path to charging station

      float t; //age, in milliseconds

      list< pair<Node *, float> > neighbors;
      Node * parent;
      bool visited;
      bool free; //is this node free of obstacles?
      bool b_charging_station;
  };

public:

  struct MySchedule : public list<Point2d>
  {
    MySchedule(){ duration=0; chicken_needed=1; }

    //set<Node *> nodes;
    list<Node *> nodes;
    float duration;
    int chicken_needed;
  };


  ///--------------------------------------------------

  MyPCoverPlanner
  (MyScene * scene, MyDragonAgent * agent, int W, int H,
   string method, float battery, float charging, float latency)
  {
    m_scene=scene;
    m_agent=agent;
    m_width=W;
    m_height=H;
    m_latency=latency;
    m_battery=battery;
    m_charging=charging;
    m_charging_station=NULL;
    m_opt_method=method;
    m_num_valid_cells=-1;
  }

  virtual ~MyPCoverPlanner(){
    m_grid.clear();
  }

  virtual bool build(); //build a grid

  virtual void update(); //update the timmer of the grid

  virtual void display();

  //schedule, start is the charging station
  virtual bool schedule( const Point2d& start );

  //give a point (x,y) in screen coordinate, find the node containing the point
  Node * getNode(float x, float y);

  //check if (x,y) is inside the given node
  bool isInside(Node * node, float x, float y)
  {
    if(node==NULL) return false; //cannot be inside a null
    return node==getNode(x,y);
  }

  //charging station info
  Node * getChargingStation() { return m_charging_station; }
  Point2d getChargingStationPosition(){ return m_charging_station->pos; }

  //get computed schedule
  const vector<MySchedule> & getSchedules() const { return m_schedules; }

protected:

  //schedulers
  bool schedule_tsp_segments_lp(int trials); //return false if failed
  bool schedule_tsp_segments_lp2(int trials);//return false if failed
  void schedule_tsp_segments_greedy(int trials);
  void schedule_shorest_paths_lp(); //

  //return true if m_agent collide with a non-movable object
  //at a given location
  bool collision_detection(const Point2d& pos);
  bool collision_detection(const Point2d& pos1, const Point2d& pos2);

  //compute path to charging station and check if
  //all path length to charging station is < battery/2
  bool paths2station();

  //estimate the cost of travelling from pos1 to pos2
  //using the values generate by Perlin noise
  float cost(const Point2d& pos1, const Point2d& pos2);

  //convert distane to time
  float dist2time(float dist);

  //nodes visited by the given path
  list<Node *> visitedNodes(const list<Point2d>& path, float arrival_time);

  //check if the schedule time is valid
  //bool isvalid(MySchedule& s, Node * new_n);

  struct LP_constraints
  {
    LP_constraints(){ type = 1;  upper_bound = lower_bound = 0; }
    vector<int> vids; //varibles involed in this constraint

    int type;  //GLP_FR    free (unbounded) variable, (1)
           //GLP_LO    lower bound
           //GLP_UP    upper bound
           //GLP_DB    double bound
           //GLP_FX    fixed

    int upper_bound;
    int lower_bound;
  };

  //LP stuff

  //generate constranits
  void generate_constraints( const vector<MySchedule>& schedules,  list<LP_constraints>& constraints);

  //find the optimal subset of schdules
  //return the total number of chickens needed
  int SolveLP(vector<MySchedule>& schedules, vector<MySchedule>& opt);

  //solve LP problem from the given schedule and constraints
  //called by "int SolveLP" and return true is optimal solution is found
  bool SolveLP(vector<MySchedule>& schedules,
              list<LP_constraints>& constaints,
              vector<float>& solution);


  //compute a matrix represention of the graph
  typedef vector< pair<Node *, float> > TSP;
  void tsp(Node * start, vector<TSP>& TSPs, int number); //nodes and weights
  void tsp(const vector<Node *>& subg, Node * start, vector<TSP>& TSPs, int number);

  //build a schedule from a tour and its start
  TSP::const_iterator build_valid_schedule_from_tsp
  (const TSP& tour, TSP::const_iterator start, MySchedule& schedule);

  void build_valid_schedule_from_tsp
  (const MyPCoverPlanner::TSP& tour,
   MyPCoverPlanner::TSP::const_iterator start,
   vector<MySchedule>& schedules);

  std::vector< std::vector<Node> > m_grid; //a grid for motion planning
  //static bool comp(Node * a, Node * b){ return a->f>b->f;}

  MyDragonAgent * m_agent;
  MyScene * m_scene;
  int m_width, m_height; //width and height of the grid
  int m_num_valid_cells; //number of valid cells
  float m_latency; //the latency constraint, in milliseconds
  float m_battery; //the battery constraint, in milliseconds
  float m_charging; //the charging constraint, in milliseconds
  Node * m_charging_station;
  string m_opt_method; //optimization method specified
  vector<MySchedule> m_schedules; //one for each chicken
};

}//end namespace GMUCS425
