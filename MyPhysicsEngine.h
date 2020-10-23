#pragma once

#include "MyGame.h"
#include "mathtool/Vector.h"
#include "mathtool/Point.h"

namespace MASC_PCOVER
{

//particles
struct MyParticle
{
    //this function updates this->force
    virtual void compute_force()=0;

    float mass;     //mass of this particle
    mathtool::Vector2d force; //force applied to this particle
    mathtool::Point2d pos;      //position;
    mathtool::Vector2d vel;     //velocity
};

//
class MyPhysicsEngine
{
public:

  typedef mathtool::Point2d  Point2d;
  typedef mathtool::Vector2d Vector2d;

  void add_particle(MyParticle * p) { m_particles.push_back(p); }
  void step(float h);

protected:

  struct State
  {
    Point2d  x;
    Vector2d v;
  };

  struct dState //derivative of State
  {
    Vector2d  dx;
    Vector2d  dv;
  };

  //step forward by h secs
  void step(float h, const vector<State>& states, vector<State>& new_states);

  //TODO: compute the derivatives
  void derive(const vector<State>& states, const vector<Vector2d>& forces, vector<dState>& dxdv);

  //TODO: implement midpoint method
  void ode(float h, const vector<State>& states, const vector<dState>& dxdv,  vector<State>& new_states);

  //TODO: implement Euler's method
  void euler(float h, const vector<State>& states, const vector<dState>& dxdv,  vector<State>& new_states);

  //get states
  void particles_to_states(vector<State>& states) const
  {
    states.resize(m_particles.size());
    auto it=states.begin();
    for(MyParticle * p : m_particles)
    {
      it->x=p->pos;
      it->v=p->vel;
      it++;
    }
  }

  //set states
  void states_to_particles(const vector<State>& states)
  {
    auto it=states.begin();
    for(MyParticle * p : m_particles)
    {
      p->pos=it->x;
      p->vel=it->v;
      it++;
    }
  }

private:

  list<MyParticle *> m_particles;

};

}//end namespace MASC_PCOVER
