#pragma once

#include "MyScene.h"
#include "MyInterval.h"
#include "MyGraph.h"
#include <set>
#include <unordered_map>

namespace MASC_PCOVER
{

class MyDragonAgent;

//the main persistent covering planner
class MyPCoverPlanner
{
public:

  struct MySchedule;      //defined later
  struct MyTimedSchedule; //defined later

  friend bool build_graph_from_grid(MyPCoverPlanner& pcover, const mathtool::Point2d& start);
  friend bool build_graph_from_txt(MyPCoverPlanner& pcover, const string& filename);
  friend bool build_graph_from_prm(MyPCoverPlanner& pcover, const mathtool::Point2d& start);

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
        b_inspect=true; //by default we inspect all nodes
        t=0;
        time2station=dist2station=0;
        dist=FLT_MAX;
        flag=INT_MAX;
        latest_valid_time=0;
      }

      int id;
      int flag; //flags for any special use
      Point2d pos; //location of the node
      //float f, g; //f is the total cost, g is to cost to node

      float dist2station; //distance to charging station
      float time2station; //time to charging station
      list<Point2d> path2station; //note: this is in fact path *FROM* charging station

      float t; //age, in milliseconds
      bool free; //is this node free of obstacles?
      bool b_charging_station;
      bool b_inspect; //should this node be inspected persistently

      list< pair<Node *, float> > neighbors;

      //for shortest path tree
      Node * parent;
      list<Node*> children;
      bool visited;
      float dist; //distance to the root

      //
      // data for timed schedule
      //

      //PCover for this node
      typedef vector<MySchedule*> PCover;

      //a vector of timed tours passing through this node
      //double: arrivial time at this node
      //MySchedule *: a schedule that arrived this node
      vector< pair<double, MySchedule*> > timed_schedules;

      double latest_valid_time; //the lastest time that this node must be visited

      // a list of valid PCover for this node
      list<PCover> valid_pcovers;
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

  //same as getWeight but wont exit if n1 and n2 are not neighbors
  //return -FLT_MAX when no edge connecting n1 & n2
  float getWeight_no_exit(Node * n1, Node * n2)
  {
    if(n1==n2) return 0;
    for(auto nei : n1->neighbors){
      if(nei.first==n2) return nei.second;
    }
    return -FLT_MAX;
  }

public:

  struct MySchedule : public list<Point2d> //trajectory....
  {
    MySchedule(){ duration=0; chicken_needed=1; }
    MySchedule(const MySchedule& other){
      this->insert(this->end(), other.begin(),other.end());
      this->nodes=other.nodes;
      this->duration=other.duration;
      this->chicken_needed=other.chicken_needed;
    }

    std::string to_string();

    list<Node *> nodes; //all nodes this schedule visits before the latency contraint
                        //is violated (assumeing all nodes start at 0)

    float duration;     //time to finish the tour
    int chicken_needed; //number of chickens needed for this schedule
  };


  //schedule with information of arrivial times for visited nodes
  //this class assumes one chicken per timed schedule
  struct MyTimedSchedule : public MySchedule
  {
    MyTimedSchedule(){ start_time=0; }
    MyTimedSchedule(const MySchedule& schedule) : MySchedule(schedule)
    {
      nodes=schedule.nodes;
      duration=schedule.duration;
      chicken_needed=schedule.chicken_needed;
      start_time=0;
    }
    MyTimedSchedule(const MyTimedSchedule& schedule) : MySchedule(schedule) {
      nodes=schedule.nodes;
      duration=schedule.duration;
      chicken_needed=schedule.chicken_needed;
      start_time=0;
      arrival_times=schedule.arrival_times;
      start_time=schedule.start_time;
      node2tt=schedule.node2tt;
    }

    //extra data
    //arrival time of each node
    list<double> arrival_times; //one for each node, so nodes.size==arrival_times.size

    double start_time;

    //get travel time from the charging station to each node
    const unordered_map<Node *, list<double>> & getTT();
    const unordered_map<Node *, list<double>> & getTT() const;

  private:
    unordered_map< Node *, list<double> > node2tt; //mapping node to traval time
  };

  // 2nd version of time schedule
  // schedule with information of arrivial times for visited nodes
  //
  struct MyFlexibleTimedSchedule : public MyTimedSchedule
  {
    MyFlexibleTimedSchedule() {
      start_time=0;
      ///in this case the interval is invalid
    }
    MyFlexibleTimedSchedule(const MySchedule& schedule)
    : MyTimedSchedule(schedule)
    {
      start_time=DBL_MAX; //start_time is ignored in this class.
      start_interval.set(0,DBL_MAX);
      if(arrival_times.size()==nodes.size() && nodes.size()>0){
        auto ia=arrival_times.begin();
        for(auto in=nodes.begin(); in!=nodes.end();in++,ia++)
          node_2_arrival_times[*in]=*ia;
      }
      else cerr<<"! Warning: MyFlexibleTimedSchedule is not properly built"<<endl;
    }

    MyFlexibleTimedSchedule(const MyFlexibleTimedSchedule& schedule)
    : MyTimedSchedule(schedule)
    {
      start_interval=schedule.start_interval;
      node_2_arrival_times=schedule.node_2_arrival_times;
    }

    //given a node and the required arrival time interval (arrive_low, arrive_hi)
    //where arrive_low<=arrive_hi, update this->start_interval if possible
    //return true if start_interval is updated
    bool update_start_interval(Node * n, double arrive_low, double arrive_hi);

    //update start interval using all nodes in this schedule
    void update_start_interval(double arrive_low, double arrive_hi);

    //can this schedule cover the given node between the given times (arrive_low, arrive_hi)
    //this also considers the needed charging time
    bool is_covered(Node * n, double arrive_low, double arrive_hi, double charging);

    unordered_map<Node *, double> node_2_arrival_times; //mapping node to arrival time assime the chicken starts at time 0
    MyInterval start_interval; //the chicken can start within the time interval
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
    //m_grid.clear();
  }

  virtual void update(); //update the timmer of the grid

  virtual void display();

  //the main schedule method, which calls the specific scheduler
  virtual bool schedule( );

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

  //get computed schedules
  const vector<MySchedule> & getSchedules() const { return m_schedules; }
  const vector<MyTimedSchedule> & getTimedSchedules() const { return m_timed_schedules; }

  //sensor model of the chicken
  float getSensorWidth();
  float getSensorHeight();

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
  bool schedule_rolling();

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
  //return nodes along the path that can be visited before the deadline
  list<Node *> visitedNodes(const list<Point2d>& path, float arrival_time);

  //this version gets all visited nodes along the path regardless the latency constraint
  inline list<Node *> visitedNodes(const list<Point2d>& path);

  //build shortest path tree
  void dijkstra(Node * root);

  //get desendents on dijkstra's tree
  void get_desendents(Node * n, vector<Node*> & decendents);

  //create schedules from TSP and/or lollipop tours
  void collect_schedules(vector<MySchedule>& schedules, int tsp_trials, bool do_lollipop=false);

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

  //build a lollipop tour
  Lollipop build_lollipop(Node * n, float start_time);
  //build multiple lollipop tours
  vector<Lollipop> build_lollipops(Node * n, float start_time);
  //build multiple lollipop tour only considering battery constraint
  vector<Lollipop> build_lollipops_ignoring_latency(Node * n);
  //create an initial lollipop
  Lollipop init_lollipop(Node * n, float battery, float latency);

  //expand the given lollipop
  bool expand_lollipop(Lollipop & lollipop, float battery, float latency);
  //expand the given lollipop into multiple lollipops
  bool expand_lollipop(Lollipop & lollipop, set<Lollipop>& expand, float battery, float latency);
  //find a Lollipop_Node that the given lollipop can expand to
  //NOTE: this function can return different node for the same lollipop
  //      by manipulating node->flag
  Lollipop_Node * find_next_expansion(Lollipop & lollipop, float battery, float latency);

  //sometimes we can reduce the travel time of a given lollipop
  bool optimize_lollipop(Lollipop & lollipop, float battery, float latency);
  bool optimize_lollipop_simple(Lollipop & lollipop, float battery, float latency);
  bool optimize_lollipop_simple2(Lollipop & lollipop, float battery, float latency);

  //convert a lollipop to a schedule
  MySchedule lollipop2schedule(Lollipop & lollipop);

  //convert a schedule to a vector of timed schedules
  void schedule2timedschedules(MySchedule & base, vector<MyTimedSchedule>& Tschedules);

  //build a list of valid pcovers for each node
  //post: fills list< Node::PCover > valid_pcovers
  void build_pcovers_from_timed_schedules(Node * n);

  //check if a node pcover is valid
  bool valid_node_pcover(list<vector< pair<double,MySchedule*> >::iterator> & pcover);

  //check if the schedule time is valid
  //bool isvalid(MySchedule& s, Node * new_n);


  //check if a node is pcoverd by the timed schedule (ts) with the schedules in
  //n->timed_schedules
  bool is_pcovered(Node * n, MyTimedSchedule * ts, double& witness);

  //check if a node is pcoverd by the timed schedules in n->timed_schedules
  //assume n->timed_schedules is sorted by arrival time
  bool is_pcovered(Node * n, double& witness);

  //check if a node is pcoverd by the provided timed schedules
  //assume timed_schedules is sorted by arrival time
  bool is_pcovered
  (Node * n, vector< pair<double, MySchedule*> >& timed_schedules, double& witness);

  struct LP_constraints
  {
    LP_constraints(){ type = 1;  upper_bound = lower_bound = 0; }
    vector<int> vids; //varibles involed in this constraint
    vector<float> coeffs; //coefficient for each variable, if empty, coeffs are 1

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
  void generate_constraints( const list<Node*> nodes, vector<MyTimedSchedule>& schedules,  list<LP_constraints>& constraints);

  //find the optimal subset of schdules that covers all the nodes
  //return the number of chickens needed
  int SolveLP(vector<MySchedule>& schedules, vector<MySchedule>& opt);

  //find the optimal subset of timed schdules from node pcovers
  int SolveLP(vector<MyTimedSchedule>& opt);

  //check if proposed_ts to conver all nodes in "nodes"
  int SolveLP
  (list<Node *>& nodes, vector<MyTimedSchedule>& proposed_ts, vector<MyTimedSchedule>& opt_ts);

  //solve LP problem from the given schedule and constraints
  //return true if optimal solution is found
  //calling SolveLP(vector<int>& variables, ...)
  bool SolveLP(vector<MySchedule>& schedules,
              list<LP_constraints>& constraints,
              vector<float>& solution);

  //solve LP problem from the given variables and constraints
  //"variables" stores the weights of each variable
  //returns true if optimal soltion is found
  bool SolveLP(vector<int>& variables, list<LP_constraints>& constaints, vector<float>& solution);

  //compute a matrix represention of the graph
  typedef vector< pair<Node *, float> > TSP; //<node, time>

  void tsp(Node * start, vector<TSP>& TSPs, int number); //nodes and weights
  void tsp(const vector<Node *>& subg, Node * start, vector<TSP>& TSPs, int number);
  void tsp_easy( vector<Node *>& subg, Node * start, vector<TSP>& TSPs, int number); //nodes and weights

  //convert tsp to lollipop
  Lollipop tsp2lollipop(TSP& tsp);

  //build a schedule from a tour and its start
  //return the iterator of tour that cannot fit into this current schedule
  TSP::const_iterator build_valid_schedule_from_tsp
  (const TSP& tour, TSP::const_iterator start, MySchedule& schedule);

  //build a schedule from a tour and its start and
  //add the schedule to "schedules"
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

  //std::vector< std::vector<Node> > m_grid; //a grid for motion planning
  MyBasicGraph<Node> m_graph; //the graph to be covered

  MyDragonAgent * m_agent;
  MyScene * m_scene;


  int m_width, m_height; //width and height of the grid
  int m_num_valid_cells; //number of valid cells
  float m_latency; //the latency constraint, in milliseconds
  float m_battery; //the battery constraint, in milliseconds
  float m_charging; //the charging constraint, in milliseconds
  Node * m_charging_station;
  string m_opt_method; //optimization method specified
  vector<MySchedule> m_schedules; //may have multiple chickens
  vector<MyTimedSchedule> m_timed_schedules; //one for each chicken


  static int PCOVER_FLAG;
  int getFlag(){ return PCOVER_FLAG--;}
};

}//end namespace MASC_PCOVER
