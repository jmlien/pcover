#pragma once

#include <list>

class MyTaskNode //a node of the behavior tree, this is an abstract class
{
public:

  virtual bool run() =0; //execute this task

  void add_kid(MyTaskNode * node) { children.push_back(node); }

protected:
  std::list<MyTaskNode *> children;
};

class MySelectorNode : public MyTaskNode
{
public:

  //run the tasks in sequence, return true immediately after a successful task is done
  //return false when all tasks failed
  virtual bool run(); //execute this task
};

class MySequenceNode : public MyTaskNode
{
public:
  //run the tasks in sequence, return true immediately after a successful task,
  //return false when all tasks failed
  virtual bool run(); //execute this task
};

class MySquenceNode : public MyTaskNode
{
public:

  //run the tasks in sequence, return false immediately after a failed task,
  //return true when all tasks finish successfully
  virtual bool run(); //execute this task
};

//A dummy behavior tree
class MyBehaviorTree
{
public:
  MyBehaviorTree(MyTaskNode * root){ m_root=root; }
  bool run(){ if(m_root==NULL) return false; return m_root->run(); }

private:

  MyTaskNode * m_root; //root of the behavior tree
};
