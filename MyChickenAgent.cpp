#include "MyChickenAgent.h"
#include "MyGame.h"
#include "MyDragonAgent.h"
#include "MyPCoverPlanner.h"
#include "mathtool/intersection.h"

using namespace mathtool;

namespace GMUCS425
{

  void MyChickenAgent::display()
  {
    if(!this->visible) return; //not visible...
    //setup positions and ask sprite to draw something
    float my_W=this->sprite->getWidth(scale);
    float my_H=this->sprite->getHeight(scale);
    this->sprite->display(x-my_W/2, y-my_H/2, scale, 0, NULL, left?SDL_FLIP_HORIZONTAL:SDL_FLIP_NONE);

    //SDL_Renderer * renderer=getMyGame()->getRenderer();
    //if(this->leader) SDL_SetRenderDrawColor(renderer,255,0,0,100);
    //else
    //SDL_SetRenderDrawColor(renderer,200,200,200,100);

    // draw_bounding_box();
    //
    // if(this->leader) SDL_SetRenderDrawColor(renderer,255,0,0,100);
    // else SDL_SetRenderDrawColor(renderer,200,200,200,100);
    // draw_circle(x,y,view_radius);
  }

  void MyChickenAgent::handle_event(SDL_Event & e)
  {

  }

  // MyDragonAgent * MyChickenAgent::get_dragon()
  // {
  //   MyDragonAgent* dragon=NULL;
  //
  //   //find the dragon agent and get its position
  //   const std::list<MyAgent * > &  agents=getMyGame()->getSceneManager()->get_active_scene()->get_agents();
  //   for( MyAgent * agent : agents)
  //   {
  //     if( dynamic_cast<MyDragonAgent*>(agent)==NULL ) //not a dragon
  //       continue;
  //     dragon=dynamic_cast<MyDragonAgent*>(agent);
  //     break;
  //   }
  //
  //   return dragon;
  // }


  void MyChickenAgent::update()
  {
    //update agent position using particle position
    //degree=atan2(this->vel[1],this->vel[0])* 180 / PI;
    //if(this->vel[0]<0) degree-=180;
    //cout<<"m_state="<<m_state<<" battery="<<m_battery<<endl;
    if(m_state=='m') //in mission
    {
      //finished the mission?
      if(m_path.empty() || (m_battery<=0 && is_inside_charing_station()) )
      {
        m_state='c';
        cout<<"- chicken arrived at charging base with battery: "<<m_battery<<endl;
        m_charging_time_left=m_charging_time;
        assert(m_pcover);
        //move the chicken to the center
        mathtool::Point2d charging_center=m_pcover->getChargingStationPosition();
        x=charging_center[0];
        y=charging_center[1];
      }
      else //still have unfinished task and outside charging station
      {
        if(m_battery<=0) //out of battery
        {
          m_state='f'; //failed
          cout<<"- chicken failed with no battery left..."<<endl;
        }
        else //still has some power
        {
          if(!m_path.empty()) //still has mission
          {
            //cout<<"move_to_next_waypoint "<<this->m_path.size()<<endl;
            move_to_next_waypoint();
            m_battery-=getMyGame()->getTimeStep();
          }
        }
      }//end still have power
    }

    if(m_state=='c') //charging
    {
      m_charging_time_left-=getMyGame()->getTimeStep();
      if(m_charging_time_left<=0) //fully charged
      {
        m_battery=m_battery_capacity;
        m_charging_time_left=0;
        m_state='i';
        m_path=m_path_bkup;
        cout<<"- chicken fully charged, ready to serve!"<<endl;
      }
    }

    if(m_state=='i')//idle
    {
      if(m_time_delay>0)
      {
        if(clock_started)
          m_time_delay-=getMyGame()->getTimeStep();;
      }
      else
      {
        if(!m_path.empty()) //has path
          m_state='m';
      }
      //starting now!
      clock_started=true;
    }

  }

  mathtool::Box2d MyChickenAgent::get_BoundingBox()
  {
    mathtool::Box2d box;
    box.width=this->sprite->getWidth(scale);
    box.height=this->sprite->getHeight(scale);
    box.x=x-box.width/2;
    box.y=y-box.height/2;
    return box;
  }

  void MyChickenAgent::move_to_next_waypoint()
  {
    //every getMyGame()->getTimeStep(), move 2 steps forward
    float delta=2;//(getMyGame()->getTimeStep());
    auto waypt=this->m_path.front(); //way point
    //mathtool::Point2d pos(x,y); //current position;
    mathtool::Vector2d v=waypt-m_f_pos;
    float vnorm = v.norm();

    //cout<<"v normsqr="<<v.normsqr()<<endl;
    while( vnorm<delta ){
      delta-=vnorm;
      this->m_path.pop_front();
      if(this->m_path.empty())
      {
        return; //no more way points....
      }
      m_f_pos=waypt;
      waypt=this->m_path.front();
      v=waypt-m_f_pos;
      vnorm = v.norm();
    }

    //float d=v.norm();
    v=v*(delta/vnorm);
    left=(v[0]<0); //facing left now?
    m_f_pos=m_f_pos+v;

    //check terrain, if the dragon is in watery area, slow down
    // const Uint32 * terrain = getMyGame()->getSceneManager()->get_active_scene()->get_terrain();
    // int terrain_width = getMyGame()->getScreenWidth();
    // int terrain_height = getMyGame()->getScreenHeight();
    // Uint32 watery=terrain[((int)y)*terrain_width+((int)x)] & 255;
    // float scale=(2-watery*1.0f/255); //1~2
    // v=v*scale;

    //update
    //x+=(int)v[0];
    //y+=(int)v[1];
    x=m_f_pos[0];
    y=m_f_pos[1];
  }

  bool MyChickenAgent::is_inside_charing_station()
  {
    assert(m_pcover);
    bool r=m_pcover->isInside(m_pcover->getChargingStation(), x, y);
    return r;
  }


}//end namespace
