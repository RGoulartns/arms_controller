#ifndef IROS_NODE_HPP
#define IROS_NODE_HPP

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "arms_controller/icontroller.hpp"

class IController;

class IROSNode : public rclcpp::Node
{
public:
  IROSNode(std::string nodeName) : rclcpp::Node(nodeName) { setvbuf(stdout, NULL, _IONBF, BUFSIZ); };
  virtual ~IROSNode() {};

  // Wanna be observable?! :P
  virtual void bind(std::shared_ptr<IController> controller) { controller_ = controller; };
  virtual void notify() { controller_.get()->update(); };

protected:
  std::shared_ptr<IController> controller_;
};


#endif // IROS_NODE_HPP
