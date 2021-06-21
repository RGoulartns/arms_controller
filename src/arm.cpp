#include "arms_controller/arm.hpp"

std::atomic<uint8_t> Arm::idProvider_ {0};

Arm::Arm(std::string armName, std::vector<std::string> joints)
  : id_(idProvider_.fetch_add(1, std::memory_order_relaxed) + 1)
  , name_(armName)
{
  gripper_ = std::make_unique<Gripper>();
  for(const auto & joint : joints)
    joints_.emplace(joint, std::make_unique<Joint>(joint));
}

Arm::~Arm(){}

uint8_t Arm::getId() const
{
  return id_;
}
std::string Arm::getName() const
{
  return name_;
}

void Arm::setJointPosition(const std::string name, const uint8_t pos)
{
  joints_.at(name)->setPosition(pos);
}

uint8_t Arm::getJointPosition(const std::string name) const
{
  return joints_.at(name)->getPosition();
}

void Arm::setGripperPosition(const uint8_t pos)
{
  gripper_->setPosition(pos);
}
uint8_t Arm::getGripperPosition() const
{
  return gripper_->getPosition();
}

void Arm::setGripperTorque(const uint8_t torque)
{
  gripper_->setTorque(torque);
}
uint8_t Arm::getGripperTorque() const
{
  return gripper_->getTorque();
}
