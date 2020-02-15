#include "MyZombieAgent.h"

namespace GMUCS425
{
  //The base class of all zombie behavior nodes
  //again, this is an abstract class because "run" is not implemented
  class MyZombieTaskNode : public MyTaskNode
  {
  public:
    MyZombieTaskNode(MyZombieAgent * agent){ zombie=agent; }
  protected:
    MyZombieAgent * zombie;
  };

  //TODO: Define more tasks here
  class MyZombieTask01 : public MyZombieTaskNode
  {
  public:
    MyZombieTask01(MyZombieAgent * agent):MyZombieTaskNode(agent)
    {
      foo=false;  //REMOVE this
    }
    bool run(){
      //TODO: implement this
      return false;
    }
  private:
    bool foo; //remove this
  };

  //TODO: Define more tasks here
  class MyZombieTask02 : public MyZombieTaskNode
  {
  public:
    MyZombieTask02(MyZombieAgent * agent, int remove_me):MyZombieTaskNode(agent)
    {
      bar=remove_me; //REMOVE this
    }
    bool run(){
      //TODO: implement this
      return false;
    }
  private:
    int bar; //remove this
  };

  //TODO: make sure that you define more tasks here, ex: MyZombieTask03~MyZombieTask99

  //TODO: build your behavior tree here
  void MyZombieAgent::build_behavior_tree()
  {
    MyTaskNode * root=NULL;

    //TODO:
    //Create a root
    //Create more nodes using MyZombieTask1~MyZombieTask100
    //use add_kid to build the rest of the tree!

    //EXAMPLE: REMOVE THIS BLOCK
    MySelectorNode * dummy_node=new MySelectorNode();
    MyZombieTask01 * remove_this_node=new MyZombieTask01(this);
    MyZombieTask02 * remove_this_node_too=new MyZombieTask02(this,10000);
    dummy_node->add_kid(remove_this_node);
    remove_this_node->add_kid(remove_this_node_too);
    root=dummy_node;
    //EXAMPLE: REMOVE THIS BLOCK

    //build the tree using the root
    this->behavior=new MyBehaviorTree(root);
  }

  void MyZombieAgent::display()
  {
    if(!this->visible) return; //not visible...
    //setup positions and ask sprite to draw something
    this->sprite->display(x, y, scale, degree);
    draw_bounding_box();
  }

  void MyZombieAgent::update()
  {
    if(this->behavior==NULL)
    {
        this->build_behavior_tree();
        assert(this->behavior);
    }

    if(!this->collision)
    {
      if(collision_free_timer>=0) this->collision_free_timer--;
      else collide_with=NULL; //no collision
    }

    this->collision=false;
  }

  void MyZombieAgent::handle_event(SDL_Event & e)
  {

    if(this->collision && collide_with!=NULL)
    {
      return;
    }

    if(e.type==SDL_USEREVENT)
    {
      if(e.user.code == 1)
      {
        if(e.user.data1==this || e.user.data2==this)
        {
          MyAgent * other = (MyAgent *)((e.user.data1!=this)?e.user.data1:e.user.data2);

          if(other!=collide_with)
          {
            collide_with=other;
          }
          this->collision_free_timer=10;
          this->collision=true;
        }
      }
    }
  }

}//end namespace
