#include "../include/arms_controller/ros_node.hpp"

ROSNode::ROSNode()
  : IROSNode("arms_controller_node")
{}

ROSNode::~ROSNode()
{
  rclcpp::shutdown();
  while(rclcpp::ok())
    usleep(100);

  this->quit();
  this->wait();
}

void ROSNode::run()
{
  rclcpp::spin(this->get_node_base_interface());
  rclcpp::shutdown();
}
