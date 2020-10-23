#pragma once

#include "MyAgent.h"
#include "MyBehaviorTree.h"

namespace MASC_PCOVER
{
    class MyZombieAgent : public MyAgent
    {
      public:
        MyZombieAgent(bool movable=true, bool collision=true)
        :MyAgent(movable,collision)
        {
          collide_with=NULL;
          behavior=NULL;
        }

        virtual void update();
        virtual void display();
        virtual void handle_event(SDL_Event & e);

      protected:

          //NEW
          void build_behavior_tree();

      private:

        MyAgent * collide_with;
        int collision_free_timer=10;
        MyBehaviorTree * behavior; //NEW, build this!
    };

}//end namespace
