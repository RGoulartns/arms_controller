#include "arms_controller/arm_ros.hpp"
#include <QDebug>


ArmROS::ArmROS(std::string armName, std::shared_ptr<IROSNode> node)
  : rosNode_(node)
  , serviceTimeout_(0.5)
{
  //TODO: read parameters -> use armName to find gripper + joints
  std::vector<std::string> joints;
  joints.push_back("wrist");

  arm_ = std::make_unique<Arm>(armName, joints);

  for(const auto& joint : joints)
    joints_pub_.emplace(joint, rosNode_->create_publisher<std_msgs::msg::Int8>(armName + "/joints/" + joint + "/position", 1) );

  gripperPos_sub_ = rosNode_->create_subscription<std_msgs::msg::Int8>(armName + "/gripper/position", 1, std::bind(&ArmROS::gripperPositionCB, this, std::placeholders::_1) );
  gripperCalib_clt_ = rosNode_->create_client<std_srvs::srv::Empty>(armName + "/gripper/calibrate");
  gripperRelease_clt_ = rosNode_->create_client<std_srvs::srv::Empty>(armName + "/gripper/release");
  gripperMove_act_ = rclcpp_action::create_client<GripperCmd>(rosNode_, armName + "/gripper/goto");

  if( !gripperCalib_clt_->wait_for_service(std::chrono::duration<double>(serviceTimeout_)) )
  {
    RCLCPP_ERROR(rosNode_->get_logger(), "Client not available after waiting");
    //throw std::runtime_error(std::string("Client not available after waiting"));
  }

  if( !gripperRelease_clt_->wait_for_service(std::chrono::duration<double>(serviceTimeout_)) )
  {
    RCLCPP_ERROR(rosNode_->get_logger(), "Client not available after waiting");
    //throw std::runtime_error(std::string("Client not available after waiting"));
  }

  if ( !gripperMove_act_->wait_for_action_server(std::chrono::duration<double>(serviceTimeout_)) )
  {
    RCLCPP_ERROR(rosNode_->get_logger(), "Action server not available after waiting");
    //throw std::runtime_error(std::string("Action server not available after waiting"));
  }
}

ArmROS::~ArmROS()
{}

void ArmROS::calibrateGripper()
{
  auto request = std::make_unique<std_srvs::srv::Empty::Request>();
  gripperCalib_clt_->async_send_request(std::move(request));
}

void ArmROS::releaseGripper()
{
  auto request = std::make_unique<std_srvs::srv::Empty::Request>();
  gripperRelease_clt_->async_send_request(std::move(request));
}

void ArmROS::setGripperPosition(const uint8_t pos, const uint8_t torque)
{
  auto goal_msg = GripperCmd::Goal();
  goal_msg.command.position = pos;
  goal_msg.command.max_effort = torque;

  auto actionOptions = rclcpp_action::Client<GripperCmd>::SendGoalOptions();
  actionOptions.result_callback = std::bind(&ArmROS::gripperActionResultCB, this, std::placeholders::_1);

  // App crashes if server doesn't exist. try-catch doesn't work.
  gripperMove_act_->async_send_goal(goal_msg, actionOptions);
}

void ArmROS::setJointPosition(const std::string& name, const uint8_t pos)
{
  std_msgs::msg::Int8 msg;
  msg.data = pos;
  joints_pub_.at(name)->publish(msg);
}

uint8_t ArmROS::getGripperPosition() const
{
  return arm_->getGripperPosition();
}

uint8_t ArmROS::getJointPosition(std::string& name) const
{
  return arm_->getJointPosition(name);
}

void ArmROS::gripperActionResultCB(const GripperCmdHandle::WrappedResult& result)
{
  //might do something..
  //this->notify();
}

void ArmROS::gripperPositionCB(const std_msgs::msg::Int8::SharedPtr msg)
{
  arm_->setGripperPosition(msg->data);
  rosNode_->notify();
}
