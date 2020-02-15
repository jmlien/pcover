#pragma once

#include "MyAgent.h"
#include "mathtool/Point.h"

namespace GMUCS425
{
    class MyPathPlanner; //define in MyPathPlanner.h
    class MyPCoverPlanner; //define in MyPCoverPlanner.h
    class MyChickenAgent;

    class MyDragonAgent : public MyAgent
    {
    public:

      typedef mathtool::Point2d Point2d;

      MyDragonAgent(string method, float battery, float charging, float latency, int width, int height)
      :MyAgent(true,true)
      {
        dx=dy=0;
        collide_with=NULL;
        has_schedule=has_goal=left=false;
        planner=NULL;
        pcover=NULL;
        this->battery=battery;
        this->charging_time=charging;
        this->latency=latency;
        this->pcover_grid_width=width;
        this->pcover_grid_height=height;
        this->pcover_opt_method=method;
      }

      virtual void handle_event(SDL_Event & e);
      virtual void update();
      virtual void draw_HUD();
      //render this agent
      virtual void display();

      virtual mathtool::Box2d get_BoundingBox();

      //determine path and return path length
      float pathing(const Point2d& s, const Point2d& g, list<Point2d>& path);

    private:

      void move_to_next_waypoint();
      void pathing();
      void schedule();
      void create_planner();

      //display goal and path if there is any
      void draw_goal_and_path();

      //path planning related data
      bool has_goal;
      mathtool::Point2d goal;
      std::list<mathtool::Point2d> path;
      MyPathPlanner * planner;
      MyPCoverPlanner * pcover;

      //persistent covering related
      string pcover_opt_method;
      bool has_schedule;
      float battery;
      float charging_time;
      float latency;
      int pcover_grid_width;
      int pcover_grid_height;

      //basic data
      int dx, dy;
      bool left;
      MyAgent * collide_with;

      //
      list<MyChickenAgent *> chickens;
    };

}//end namespace
