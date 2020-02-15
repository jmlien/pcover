#include "MyAgent.h"
#include "MyGame.h"
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <sstream>
#include "mathtool/Box.h"

namespace GMUCS425
{

  void MyAgent::handle_event(SDL_Event & e)
  {
    if(this->movable)
    {
      //do nothing now...
    }
  }

  //update this agent's motion, looks, sound, etc
  void MyAgent::update()
  {
    //do nothing by default...
  }

  void MyAgent::display()
  {
    if(!this->visible) return; //not visible...
    //setup positions and ask sprite to draw something
    this->sprite->display(x, y, scale, degree);
  }

  //show HUD (heads-up display) or status bar
  void MyAgent::draw_HUD()
  {
    //draw nothing by defaut, your task to display the location of an agent
    //on the upper left corner
    //read http://lazyfoo.net/tutorials/SDL/16_true_type_fonts/index.php
  }

  mathtool::Box2d MyAgent::get_BoundingBox()
  {
    mathtool::Box2d box;
    box.x=x;
    box.y=y;
    box.width=this->sprite->getWidth(scale);
    box.height=this->sprite->getHeight(scale);

    return box;
  }

  bool MyAgent::collide(MyAgent * other)
  {
    mathtool::Box2d box1, box2;
    box1=this->get_BoundingBox();
    box2=other->get_BoundingBox();
    return box1.intersect(box2);
  }

  void MyAgent::draw_bounding_box()
  {
    mathtool::Box2d mbox=this->get_BoundingBox();

    SDL_Rect box; //create a rect
    box.x = mbox.x;  //controls the rect's x coordinate
    box.y = mbox.y; // controls the rect's y coordinte
    box.w = mbox.width; // controls the width of the rect
    box.h = mbox.height; // controls the height of the rect

    SDL_Renderer * renderer=getMyGame()->getRenderer();
    //SDL_SetRenderDrawColor(renderer,255,100,0,100);
    //SDL_RenderDrawPoint(renderer, x, y);
    SDL_RenderDrawRect(renderer,&box);

    box.x = x;
    box.y = y;
    box.w/=10;
    box.h=box.w;
    SDL_SetRenderDrawColor(renderer,0,0,0,100);
    SDL_RenderFillRect(renderer,&box);
  }

  //new
  void MyAgent::draw_circle(int32_t centreX, int32_t centreY, int32_t radius)
  {
     SDL_Renderer * renderer=getMyGame()->getRenderer();
     const int32_t diameter = (radius * 2);

     static const int pt_size = 25;
     static SDL_Point points[pt_size];
     static float theta=PI2/(pt_size-1);
     for(int i=0;i<pt_size-1;i++)
     {
       points[i].x=(int32_t)(radius*cos(theta*i))+centreX;
       points[i].y=(int32_t)(radius*sin(theta*i))+centreY;
     }
     points[pt_size-1]=points[0]; //close up

     SDL_RenderDrawLines(renderer, points, pt_size);
  }


}//end namespace
