#ifndef ICONTROLLER_HPP
#define ICONTROLLER_HPP

#include <string>
#include "arms_controller/iros_node.hpp"

class IROSNode;

class IController
{
public:
  IController(std::shared_ptr<IROSNode> node) { node_ = node; };
  virtual ~IController() {};


  // observer?!
  virtual void update() = 0;

protected:
  std::shared_ptr<IROSNode> node_;

};

#endif // ICONTROLLER_HPP
