#pragma once

#include <unordered_map>
#include <list>
#include <string>
#include <cassert>

namespace MASC_PCOVER
{

// the base class of all managers
template<class T>
class MyManager
{
  public:

    MyManager(){} //does nothing

    virtual ~MyManager()
    {
      for(auto t : this->m_Factory)
      {
        delete t.second;
      }
      this->m_Factory.clear();
    }

    virtual bool init(){ return true; } //does nothing by default
    virtual void kill(){} //does nothing by default

    virtual void add(std::string name, T * obj)
    {
      m_Factory[name]=obj;
    }

    virtual T * get(std::string name)
    {
      return m_Factory[name];
    }

    //get all objects mantained by this manager
    virtual void getAll(std::list<T *>& allobjs)
    {
      for(auto& item : m_Factory)
      {
        allobjs.push_back(item.second);
      }
    }


  protected:

    std::unordered_map<std::string, T *> m_Factory;
};


}//end namespace MASC_PCOVER
