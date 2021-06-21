#ifndef ROS_NODE_HPP
#define ROS_NODE_HPP

#include <QThread>
#include "arms_controller/iros_node.hpp"


class ROSNode : public IROSNode, public QThread
{
public:
  ROSNode();
  virtual ~ROSNode();

  void run();
};

#endif // ROS_NODE_HPP













