#ifndef ARMROS_HPP
#define ARMROS_HPP

#include <string>
#include <unordered_map>
#include "arms_controller/iros_node.hpp"
#include "arms_controller/arm.hpp"

#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/empty.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


class ArmROS
{
public:
  using GripperCmd = control_msgs::action::GripperCommand;
  using GripperCmdHandle = rclcpp_action::ClientGoalHandle<GripperCmd>;

  ArmROS(std::string armName, std::shared_ptr<IROSNode> node);
  virtual ~ArmROS();

  // Controller call these methods
  void calibrateGripper();
  void releaseGripper();
  void setGripperPosition(const uint8_t pos, const uint8_t torque);
  void setJointPosition(const std::string& name, const uint8_t pos);
  uint8_t getGripperPosition() const;
  uint8_t getJointPosition(std::string &name) const;

private:
  // ROS callbacks (action and topic)
  void gripperActionResultCB(const GripperCmdHandle::WrappedResult& result);
  void gripperPositionCB(const std_msgs::msg::Int8::SharedPtr msg);

  std::unique_ptr<Arm> arm_;
  std::shared_ptr<IROSNode> rosNode_;
  const float serviceTimeout_;

  // ROS topic subscription, clients and action
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr > joints_pub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr gripperPos_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gripperCalib_clt_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gripperRelease_clt_;
  rclcpp_action::Client<GripperCmd>::SharedPtr gripperMove_act_;
};

#endif // ARMROS_HPP
