#pragma once

#include "MyAgent.h"
#include "MyPathPlanner.h"
#include "MyPhysicsEngine.h"
#include "mathtool/Box.h"

namespace GMUCS425
{
    class MyDragonAgent;
    class MyPCoverPlanner;

    class MyChickenAgent : public MyAgent
    {
      public:

        typedef mathtool::Point2d  Point2d;
        typedef mathtool::Vector2d Vector2d;
        typedef mathtool::Box2d Box2d;

        MyChickenAgent(float battery, float charging_time, MyPCoverPlanner * pcover=NULL)
        :MyAgent(true,true)
        {
          //NEW
          m_battery=m_battery_capacity=battery;
          m_charging_time=charging_time;
          m_charging_time_left=0;
          m_state='i';
          left=false;
          clock_started=false;
          m_time_delay=0;
          m_pcover=pcover;
          m_angle=0;
        }

        MyChickenAgent(MyChickenAgent * other)
        :MyAgent(true,true)
        {
          //NEW
          m_battery=m_battery_capacity=other->m_battery_capacity;
          m_charging_time=other->m_charging_time;
          m_charging_time_left=other->m_charging_time_left;
          m_state=other->m_state;
          left=other->left;
          clock_started=other->clock_started;
          m_time_delay=other->m_time_delay;
        }

        virtual void update();
        virtual void display();
        virtual void handle_event(SDL_Event & e);

        // virtual void tranlate(float x, float y){
        //   this->x+=x; this->y+=y;
        //   this->pos[0]+=x;
        //   this->pos[1]+=y;
        // }
        //
        // virtual void tranlateTo(float x, float y){
        //   this->x=x; this->y=y;
        //   this->pos[0]=x;
        //   this->pos[1]=y;
        // }

        char getState() const { return m_state; }
        // void setState(char s)
        // {
        //   if(s=='c' || s=='m' || s=='i' || s=='f')
        //     m_state=s;
        //   else
        //     cerr<<"! Error: Unknown state: "<<s<<endl;
        // }

        void setPath(const std::list<mathtool::Point2d> & path)
        {
          m_path=path;
          m_path_bkup=path;
        }

        void setTimeDelay(float time_delay){
          m_time_delay=time_delay;
        }

       virtual void tranlate(float x, float y)
       {
         this->x+=x; this->y+=y;
         m_f_pos[0]+=x;
         m_f_pos[1]+=y;
       }

       virtual void tranlateTo(float x, float y)
       {
         this->x=x; this->y=y;
         m_f_pos[0]=x;
         m_f_pos[1]=y;
       }


      protected:

        //get the bounding box of this chicken
        mathtool::Box2d get_BoundingBox();
        //move this chicken to the next way point, if there is any
        void move_to_next_waypoint();
        //check if this chicken is inside the charging station
        bool is_inside_charing_station();

      private:

        std::list<mathtool::Point2d> m_path;
        std::list<mathtool::Point2d> m_path_bkup;

        char m_state; //c : charging, m: in mission, i: idle, f: failed

        float m_battery_capacity;   //max flying time, in millisecond
        float m_battery;            //flying time left, in millisecond

        float m_charging_time;      //charging time to full capacity
        float m_charging_time_left; //charging time left, in millisecond

        float m_time_delay; //delay before take off for the first time
        bool clock_started; //does this chicken started the simulation

        bool left; //facing left?

        float m_angle;

        MyPCoverPlanner * m_pcover;

        Point2d m_f_pos; //more accurate position...
    };

}//end namespace
