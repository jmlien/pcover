#include "MyPhysicsEngine.h"

namespace MASC_PCOVER
{

void MyPhysicsEngine::step(float h)
{
  vector<State> states, new_states;
  particles_to_states(states);
  new_states.resize(states.size());
  step(h, states, new_states);
  states_to_particles(new_states);
}

//step forward by h secs
void MyPhysicsEngine::step(float h, const vector<State>& states, vector<State>& new_states)
{
    vector<Vector2d> forces;
    vector<dState> dxdv;

    //1. update forces
    for(MyParticle * p : m_particles)
    {
      p->compute_force();
      forces.push_back(p->force);
    }

    //2. compute derivatives
    dxdv.resize(states.size());
    derive(states, forces, dxdv);

    //3. solve ode to get new states
    ode(h, states, dxdv, new_states);
}

//TODO: compute the derivatives
void MyPhysicsEngine::derive(const vector<MyPhysicsEngine::State>& states,
                             const vector<Vector2d>& forces,
                             vector<MyPhysicsEngine::dState>& dxdv)
{
    int size=states.size();
    auto itr=m_particles.begin();

    for(int i=0;i<size;i++,itr++)
    {
      dxdv[i].dx=states[i].v;
      dxdv[i].dv=forces[i]/(*itr)->mass;
    }
}

//TODO: implement midpoint method
void MyPhysicsEngine::ode(float h,
                          const vector<MyPhysicsEngine::State>& states,
                          const vector<MyPhysicsEngine::dState>& dxdv,
                          vector<MyPhysicsEngine::State>& new_states)
{
  // new_states=states; //remove this line
  euler(h,states,dxdv,new_states);
  return;














  vector<State> mid_states;
  vector<Vector2d> mid_forces;
  vector<dState> mid_dxdv;

  mid_states.resize(states.size());
  //mid_forces.resize(states.size());
  mid_dxdv.resize(states.size());

  //get mid point derivative
  euler(h/2,states,dxdv,mid_states);
  states_to_particles(mid_states);
  for(MyParticle * p : m_particles)
  {
    p->compute_force();
    mid_forces.push_back(p->force);
  }

  derive(mid_states, mid_forces, mid_dxdv);
  euler(h,states,mid_dxdv,new_states);
}

//TODO: implement Euler's method
void MyPhysicsEngine::euler(float h,
                            const vector<MyPhysicsEngine::State>& states,
                            const vector<MyPhysicsEngine::dState>& dxdv,
                            vector<MyPhysicsEngine::State>& new_states)
{
  //new_states=states; //remove this line
  int size=states.size();
  for(int i=0;i<size;i++)
  {
    new_states[i].x=states[i].x+dxdv[i].dx*h;
    new_states[i].v=states[i].v+dxdv[i].dv*h;
  }
}

}//end namespace MASC_PCOVER
