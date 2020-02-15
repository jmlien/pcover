#include "MyDragonAgent.h"
#include "MyChickenAgent.h"
#include "MyGame.h"
#include "MyPathPlanner.h"
#include "MyPCoverPlanner.h"
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <sstream>

namespace GMUCS425
{

  void MyDragonAgent::handle_event(SDL_Event & e)
  {
    const int delta=5;

    //collision event
    // if(e.type==SDL_USEREVENT)
    // {
    //   if(e.user.code == 1)
    //   {
    //     if(e.user.data1==this || e.user.data2==this)
    //     {
    //       this->collision=true;
    //       this->collide_with = (MyAgent *)((e.user.data1!=this)?e.user.data1:e.user.data2);
    //     }
    //   }
    // }

    //mouse events
    if( e.type == SDL_MOUSEBUTTONDOWN )
    {
      //SDL_MouseButtonEvent * me=(SDL_MouseButtonEvent)
      if(e.button.clicks==2 && SDL_BUTTON(SDL_BUTTON_LEFT))
      {
          //set goal!
          int mouse_x, mouse_y;
          SDL_GetMouseState(&mouse_x, &mouse_y);
          //has_goal=true;
          //
          MyChickenAgent * ready_chicken=NULL; //
          for(MyChickenAgent * chicken : chickens)
          {
            if(chicken->getState()=='i') ready_chicken=chicken;
          }

          if(ready_chicken==NULL)
          {
            if(chickens.size()>=5)
            {
              cout<<"All chickens are sleeping!"<<endl;
              return;
            }

            //create a chicken and send the chicken there
            MyChickenAgent * chicken = new MyChickenAgent(battery,charging_time, pcover);
            assert(chicken);
            MySprite * sprite=getMyGame()->getSpriteManager()->get("k");
            assert(sprite);
            chicken->setSprite(sprite);
            chicken->tranlateTo(x,y);
            chicken->scaleTo(0.1);
            getMyGame()->getSceneManager()->get_active_scene()->add_agent(chicken);
            this->chickens.push_back(chicken);
            ready_chicken=chicken;
          }

          //find the path for chicken
          this->goal=mathtool::Point2d(mouse_x,mouse_y);
          this->has_goal=true;
          pathing();
          auto tmp=path;
          tmp.pop_back();
          tmp.reverse();

          path.insert(path.end(),tmp.begin(),tmp.end());
          ready_chicken->setPath(this->path);

          //reset
          this->has_goal=false;
          this->path.clear();
      }
      else
      {
          //clear goal and path
          has_goal=false;
          this->path.clear();
      }
    }

    //No goal assigned, control by arrow keys
    if(e.type==SDL_KEYDOWN && !this->has_goal)
    {
      dx=dy=0;
      const Uint8* currentKeyStates = SDL_GetKeyboardState( NULL );
      if( currentKeyStates[ SDL_SCANCODE_UP ] )
      {
        dy=-delta;
      }
      else if( currentKeyStates[ SDL_SCANCODE_DOWN ] )
      {
        dy=delta;
      }
      else if( currentKeyStates[ SDL_SCANCODE_LEFT ] )
      {
        left=true;
        dx=-delta;
      }
      else if( currentKeyStates[ SDL_SCANCODE_RIGHT ] )
      {
        left=false;
        dx=delta;
      }

      x+=dx;
      y+=dy;
    }
  }

  void MyDragonAgent::update()
  {
    if(!has_schedule) //have no schedule, then make one
    {
      schedule();
      if(has_schedule)
      {
        const vector<MyPCoverPlanner::MySchedule>& schdules = pcover->getSchedules();

        if(chickens.empty()) //create chickens
        {
          for(const MyPCoverPlanner::MySchedule& schedule : schdules)
          {
            for(int i=0;i<schedule.chicken_needed;i++)
            {
              //create a chicken and send the chicken there
              MyChickenAgent * chicken = new MyChickenAgent(battery,charging_time,pcover);
              assert(chicken);
              MySprite * sprite=getMyGame()->getSpriteManager()->get("k");
              assert(sprite);
              chicken->setSprite(sprite);
              chicken->tranlateTo(x,y);
              chicken->scaleTo(0.1);
              getMyGame()->getSceneManager()->get_active_scene()->add_agent(chicken);
              this->chickens.push_back(chicken);
            }//end for i
            //break;
          }//end for schedule

          cout<<"- Created "<<this->chickens.size()<<" chickens"<<endl;

          //assign schedule to chickens
          auto cit=chickens.begin();
          for(const MyPCoverPlanner::MySchedule& schedule : schdules)
          {
            float time_delay=0;
            for(int i=0;i<schedule.chicken_needed;i++)
            {
              auto chicken=*cit;
              chicken->setTimeDelay(time_delay);
              chicken->setPath(schedule);;
              time_delay+=this->latency;
              cit++;
            }
          }//end for schedule
        }//end if(chickens.empty())
      }//end if(has_schedule)
    }//end if(!has_schedule)

    //has collision with some guy...
    //resolve collision
    if(this->collision && this->collide_with)
    {
      int vx=this->collide_with->getX()-x;
      int vy=this->collide_with->getY()-y;
      if( (dx==0 && dy==0) || (vx*dx+vy*dy)<=0 )
      {
        float d=1.0/sqrt((vx*vx)+(vy*vy));
        dx=vx*d;
        dy=vy*d;
      }

      x-=dx;
      y-=dy;
      dx/=2;
      dy/=2;
    }

    this->collision=false;

    //if there is goal, move to the goal
    if(this->has_goal)
    {
      if( this->path.empty() ) pathing(); //find path
      else move_to_next_waypoint();
    }

    if(this->pcover!=NULL) this->pcover->update();
  }

  void MyDragonAgent::schedule()
  {
    if(pcover==NULL)
    {
      MyScene * scene=getMyGame()->getSceneManager()->get_active_scene();
      pcover=new MyPCoverPlanner(scene, this, pcover_grid_width, pcover_grid_height, this->pcover_opt_method, this->battery, this->charging_time, this->latency);
      assert(pcover);
      if( !pcover->build() ){
        std::cerr<<"! Error: Failed to build a persistent covering planner"<<std::endl;
        delete pcover;
        pcover=NULL;
        return;
      }
    }

    has_schedule=pcover->schedule(mathtool::Point2d(x,y));

    mathtool::Point2d charing_center=pcover->getChargingStationPosition();
    this->x=charing_center[0];
    this->y=charing_center[1];
  }

  void MyDragonAgent::create_planner()
  {
    MyScene * scene=getMyGame()->getSceneManager()->get_active_scene();
    planner=new MyGridPathPlanner(scene, this, scene->get_width()*2, scene->get_height()*2);
    assert(planner);
    if( !planner->build() ){
      std::cerr<<"! Error: Failed to build a motion planner"<<std::endl;
      delete planner;
      planner=NULL;
      return;
    }
  }

  //determine path and return path length
  float MyDragonAgent::pathing(const Point2d& s, const Point2d& g, list<Point2d>& path)
  {
    if(planner==NULL) create_planner();
    bool r=planner->find_path(s,g,path);
    if(!r) return 0;

    //compute length
    float length=0;
    for(auto it=path.begin();it!=path.end();it++)
    {
      auto next=it;next++;
      if(next==path.end()) break;
      length+=((*it)-(*next)).norm();
    }//end it
    return length;
  }

  void MyDragonAgent::pathing()
  {
    //TODO: find path, please replace the code below
    if(planner==NULL) create_planner();

    bool r=planner->find_path(mathtool::Point2d(x,y),this->goal,this->path);
    if(!r) //failed to fina path
    {
      this->has_goal=false;
    }
  }

  void MyDragonAgent::move_to_next_waypoint()
  {
    const int delta=2;
    auto waypt=this->path.front(); //way point
    mathtool::Point2d pos(x,y); //current position;
    mathtool::Vector2d v=waypt-pos;
    while( v.normsqr()<2 ){
      this->path.pop_front();
      if(this->path.empty())
      {
        this->has_goal=false;
        return; //no more way points....
      }
      waypt=this->path.front();
      v=waypt-pos;
    }
    float d=v.norm();
    if(d>delta) v=v*(delta/d);

    left=(v[0]<0); //facing left now?

    //check terrain, if the dragon is in watery area, slow down
    const Uint32 * terrain = getMyGame()->getSceneManager()->get_active_scene()->get_terrain();
    int terrain_width = getMyGame()->getScreenWidth();
    int terrain_height = getMyGame()->getScreenHeight();
    Uint32 watery=terrain[((int)y)*terrain_width+((int)x)] & 255;
    float scale=(2-watery*1.0f/255); //1~2
    v=v*scale;

    dx=(int)v[0];
    dy=(int)v[1];

    //update
    x+=dx;
    y+=dy;
  }

  void MyDragonAgent::display()
  {
    if(this->has_goal)
    {
      draw_goal_and_path();
    }//end if

    if(pcover!=NULL)
    {
      pcover->display();
    }

    SDL_Renderer * renderer=getMyGame()->getRenderer();
    const vector<MyPCoverPlanner::MySchedule>& schdules = pcover->getSchedules();
    //draw path
    SDL_SetRenderDrawColor(renderer,200,0,0,100);
    auto & waypt=schdules.front().front();
    SDL_RenderDrawLine(renderer, x, y, waypt[0], waypt[1]);
    for(auto i = schdules.front().begin();i!=schdules.front().end();i++)
    {
      auto j=i; j++;
      if(j==schdules.front().end()) continue;
      SDL_RenderDrawLine(renderer, (*i)[0], (*i)[1], (*j)[0], (*j)[1]);
    }//end i


    //MyAgent::display();
    if(!this->visible) return; //not visible...
    //setup positions and ask sprite to draw something
    //float my_W=this->sprite->getWidth(scale);
    //float my_H=this->sprite->getHeight(scale);
    //this->sprite->display(x-my_W/2, y-my_H/2, scale, degree, NULL, this->left?SDL_FLIP_HORIZONTAL:SDL_FLIP_NONE);

    //SDL_SetRenderDrawColor(getMyGame()->getRenderer(),255,100,0,100);
    //draw_bounding_box();
    //display goal & path

  }

  void MyDragonAgent::draw_goal_and_path()
  {
    SDL_Rect box; //create a rect
    box.x = this->goal[0]-3;  //controls the rect's x coordinate
    box.y = this->goal[1]-3; // controls the rect's y coordinte
    box.w = 6; // controls the width of the rect
    box.h = 6; // controls the height of the rect
    SDL_Renderer * renderer=getMyGame()->getRenderer();
    SDL_SetRenderDrawColor(renderer,100,255,0,100);
    SDL_RenderDrawRect(renderer,&box);

    //draw path
    SDL_SetRenderDrawColor(renderer,200,255,0,100);
    auto & waypt=this->path.front();
    SDL_RenderDrawLine(renderer, x, y, waypt[0], waypt[1]);
    for(auto i = this->path.begin();i!=this->path.end();i++)
    {
      auto j=i; j++;
      if(j==this->path.end()) continue;
      SDL_RenderDrawLine(renderer, (*i)[0], (*i)[1], (*j)[0], (*j)[1]);
    }//end i
  }

  void MyDragonAgent::draw_HUD()
  {
    std::stringstream ss;
    ss<<"time: "<<getMyGame()->getTime()*0.01; //change to seconds
    SDL_Renderer * renderer=getMyGame()->getRenderer();
    static TTF_Font* font = NULL;

    if( font == NULL )
    {
      font = TTF_OpenFont("fonts/Demo_ConeriaScript.ttf", 32); //this opens a font style and sets a size
      if( font == NULL )
      {
        std::cerr<<"Failed to load font! SDL_ttf Error: "<< TTF_GetError() <<std::endl;
        return;
      }
    }

    SDL_Color color = {255, 255, 255};  // this is the color in rgb format, maxing out all would give you the color white, and it will be your text's color
    SDL_Surface* surfaceMessage = TTF_RenderText_Solid(font, ss.str().c_str(), color); // as TTF_RenderText_Solid could only be used on SDL_Surface then you have to create the surface first
    SDL_Texture* Message = SDL_CreateTextureFromSurface(renderer, surfaceMessage); //now you can convert it into a texture

    SDL_Rect Message_rect; //create a rect
    Message_rect.w = 120; // controls the width of the rect
    Message_rect.h = 30; // controls the height of the rect
    Message_rect.x = getMyGame()->getScreenWidth()-Message_rect.w;  //controls the rect's x coordinate
    Message_rect.y = 10; // controls the rect's y coordinte

    //Mind you that (0,0) is on the top left of the window/screen, think a rect as the text's box, that way it would be very simple to understance
    //Now since it's a texture, you have to put RenderCopy in your game loop area, the area where the whole code executes
    SDL_RenderCopy(renderer, Message, NULL, &Message_rect); //you put the renderer's name first, the Message, the crop size(you can ignore this if you don't want to dabble with cropping), and the rect which is the size and coordinate of your texture

    //Don't forget too free your surface and texture
    SDL_FreeSurface( surfaceMessage );
    SDL_DestroyTexture( Message );
  }

  mathtool::Box2d MyDragonAgent::get_BoundingBox()
  {
    mathtool::Box2d box;
    box.width=this->sprite->getWidth(scale);
    box.height=this->sprite->getHeight(scale);
    box.x=x-box.width/2;
    box.y=y-box.height/2;
    return box;
  }

}//end namespace