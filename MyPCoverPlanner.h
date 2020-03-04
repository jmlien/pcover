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
        dist=FLT_MAX;
        flag=INT_MAX;
      }

      int id;
      int flag; //flags for any special use
      Point2d pos; //location of the node
      //float f, g; //f is the total cost, g is to cost to node

      float dist2station; //distance to charging station
      float time2station; //time to charging station
      list<Point2d> path2station; //path to charging station

      float t; //age, in milliseconds
      bool free; //is this node free of obstacles?
      bool b_charging_station;

      list< pair<Node *, float> > neighbors;

      //for shortest path tree
      Node * parent;
      list<Node*> children;
      bool visited;
      float dist; //distance to root
  };

  static bool compareNode(const Node * n1, const Node * n2)
  { return n1->dist>n2->dist; }

  float getWeight(Node * n1, Node * n2)
  {
    for(auto nei : n1->neighbors){
      if(nei.first==n2) return nei.second;
    }
    cerr<<"! Error: getWeight: node "<<n1->id<<" and node "<<n2->id<<" are not connected"<<endl;
    exit(1);
  }

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
  void schedule_lollipop_lp();
  void schedule_lollipop_lp2();
  void schedule_dijkstra_lp();
  bool schedule_hybrid(int trials);

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

  //build shortest path tree
  void dijkstra(Node * root);
  //get desendents on dijkstra's tree
  void get_desendents(Node * n, vector<Node*> & decendents);

  struct Lollipop_Node
  {
    Lollipop_Node(Node * data=NULL){
      this->data=data;
      pre=next=this;
      pre_cost=next_cost=0;
      inserted=false;
    }

    float net_cost() const {
      if(inserted) {
        cerr<<"! Error: This node has been inserted. No net cost."<<endl;
        return FLT_MAX;
      }
      return pre_cost+next_cost-pre->next_cost;
    }

    //insert this node between pre and next
    void insert_this()
    {
        pre->next=this;
        next->pre=this;
        pre->next_cost=pre_cost;
        next->pre_cost=next_cost;
        inserted=true;
    }

    bool operator<(const Lollipop_Node & other) const
    {
      return this->net_cost()>other.net_cost();
    }

    bool inserted;     //if this node has been inserted to a linked list
    Node * data;
    Lollipop_Node * pre, * next;
    float pre_cost, next_cost;
  };

  struct Lollipop
  {
    Lollipop(){
      head=NULL;
      time_needed=FLT_MAX;
      count=0;
      entrance=exit=NULL;
    }

    void destroy(){ reset(); }

    Lollipop_Node * find(Node * n)
    {
      Lollipop_Node * ptr=head;
      do{
        if(ptr->data==n) return ptr;
        ptr=ptr->next;
      }
      while(ptr!=head);
      return NULL;
    }

    Lollipop clone()
    {
      Lollipop other;
      other.count=count;
      other.time_needed=time_needed;
      other.entrance=entrance;
      other.exit=exit;
      //
      other.head=new Lollipop_Node(this->head->data);
      assert(other.head);
      other.head->next_cost=head->next_cost;
      other.head->pre_cost=head->pre_cost;
      other.head->inserted=true;

      auto ptr=head->next;
      auto other_pre=other.head;
      do {
        auto other_ptr=new Lollipop_Node(ptr->data);
        assert(other_ptr);
        other_ptr->next_cost=ptr->next_cost;
        other_ptr->pre_cost=ptr->pre_cost;
        other_ptr->pre=other_pre;
        other_pre->next=other_ptr;
        other_ptr->inserted=true;
        ptr=ptr->next;
        other_pre=other_ptr;
      }
      while(ptr!=head);
      other_pre->next=other.head;
      other.head->pre=other_pre;

      return other;
    }

    bool operator<(const Lollipop& other) const
    {
        if(count!=other.count) return count<other.count;
        if(time_needed!=other.time_needed) return time_needed<other.time_needed;

        Lollipop_Node * ptr1=head;
        Lollipop_Node * ptr2=other.head;
        do{
          if(ptr1->data->id!=ptr2->data->id)
            return ptr1->data->id<ptr2->data->id;
          ptr1=ptr1->next;
          ptr2=ptr2->next;
        }
        while(ptr1!=head);
        return false;
    }

    void insert(Lollipop_Node * ln)
    {
      count++;
      if(head==NULL)
      {
        head=ln;
        entrance=exit=ln->data;
        time_needed=0;
        return;
      }

      time_needed=time_needed-ln->pre->next_cost+ln->pre_cost+ln->next_cost;
      ln->insert_this();
    }

    void reset()
    {
      if(head==NULL) return; //nothing here

      Lollipop_Node * ptr=head;
      do{
        Lollipop_Node * next=ptr->next;
        delete ptr;
        ptr=next;
      }while(ptr!=head);
      head=NULL;
      entrance=exit=NULL;
      time_needed=0;
      count=0;
    }

    Lollipop_Node * head;
    float time_needed;
    int count;

    list<Node *> best_tsp; //tsp from entrance to exit, if entrance!=head
    Node * entrance, * exit;
  };

  //build a lollipop tour and return the time needed
  Lollipop build_lollipop(Node * n, float start_time);
  vector<Lollipop> build_lollipops(Node * n, float start_time);
  Lollipop init_lollipop(Node * n, float battery, float latency);

  bool expand_lollipop(Lollipop & lollipop, float battery, float latency);
  bool expand_lollipop(Lollipop & lollipop, set<Lollipop>& expand, float battery, float latency);
  Lollipop_Node * find_next_expansion(Lollipop & lollipop, float battery, float latency);

  bool optimize_lollipop(Lollipop & lollipop, float battery, float latency);
  bool optimize_lollipop_simple(Lollipop & lollipop, float battery, float latency);
  bool optimize_lollipop_simple2(Lollipop & lollipop, float battery, float latency);
  MySchedule lollipop2schedule(Lollipop & lollipop);
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
  typedef vector< pair<Node *, float> > TSP; //<node, time>

  void tsp(Node * start, vector<TSP>& TSPs, int number); //nodes and weights
  void tsp(const vector<Node *>& subg, Node * start, vector<TSP>& TSPs, int number);
  void tsp_easy( vector<Node *>& subg, Node * start, vector<TSP>& TSPs, int number); //nodes and weights

  //convert tsp to lollipop
  Lollipop tsp2lollipop(TSP& tsp);

  //build a schedule from a tour and its start
  TSP::const_iterator build_valid_schedule_from_tsp
  (const TSP& tour, TSP::const_iterator start, MySchedule& schedule);

  void build_valid_schedule_from_tsp
  (const MyPCoverPlanner::TSP& tour,
   MyPCoverPlanner::TSP::const_iterator start,
   vector<MySchedule>& schedules);

   //some helpers
   //find a common neighbor of n1 and n2
   pair<Node *, pair<float, float> >
   getClosestCommonNeighbor(Node * n1, Node * n2, int strict_level=1)
   {
     int flag=n1->flag;
     if(flag!=n2->flag)
     {
       cerr<<"! Error: getClosestCommonNeighbor: flag error"<<endl;
       exit(1);
     }

     float shortest_d=FLT_MAX;
     pair<Node *, pair<float, float> > best(NULL, make_pair(FLT_MAX,FLT_MAX));
     for(auto nei1 : n1->neighbors)
     {
       for(auto nei2 : n2->neighbors)
       {
         if(nei1.first==nei2.first)
         {
           Node * nei=nei1.first;
         //cout<<"nei id="<<nei->id<<" flag="<<nei->flag<<endl;
           if(nei->flag==flag) continue; //already included
           //we want the nei to be further away from n1 and n2
         //cout<<"nei dist="<<nei->dist<<" n1->dist="<<n1->dist<<" n2->dist="<<n2->dist<<endl;
           //if( (strict_level==1) && (nei->dist<=n1->dist || nei->dist<=n2->dist) ) continue;
           //if( (strict_level==2) && (nei->dist<=n1->dist && nei->dist<=n2->dist) ) continue;
           if( (strict_level==1) && (nei->time2station<n1->time2station || nei->time2station<n2->time2station) ) continue;
           if( (strict_level==2) && (nei->time2station<n1->time2station && nei->time2station<n2->time2station) ) continue;
           float d=nei1.second+nei2.second;
           if(d<shortest_d)
           {
             shortest_d=d;
             best.first=nei;
             best.second.first =nei1.second;
             best.second.second=nei2.second;
           }
         }
       }//end for nei2
     }//end for nei1

     if(shortest_d==FLT_MAX && strict_level!=2)
      return getClosestCommonNeighbor(n1,n2,strict_level+1);
     return best;
   }

   //get Nodes that are connected to n and further away from
   //the charging station than n
   list<Node *> getFurtherNodes(Node * n)
   {
     list<Node *> ans;
     for(auto nei : n->neighbors)
     {
       if(nei.first->time2station>n->time2station) ans.push_back(nei.first);
     }
     return ans;
   }

   pair<Node *, float> getClosestFurtherNodes(Node * n)
   {
     pair<Node *, float> best(NULL, FLT_MAX);
     for(auto nei : n->neighbors)
     {
       if(nei.first->dist>n->dist && nei.second<best.second)
       {
         best=nei;
       }
     }
     return best;
   }
  //------------>
  float traceback(Node * n, list<Node *>& path);

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


  static int PCOVER_FLAG;
  int getFlag(){ return PCOVER_FLAG--;}
//  TSP_FLAG--; //a unique flag for each TSP call

};

}//end namespace GMUCS425
