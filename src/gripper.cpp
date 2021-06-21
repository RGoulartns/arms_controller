#include "../include/arms_controller/gripper.hpp"

Gripper::Gripper(uint8_t torque)
 : position_(0)
 , torque_(torque)
{}

void Gripper::setPosition(const uint8_t pos)
{
  position_ = pos;
}

void Gripper::setTorque(const uint8_t torque)
{
  torque_ = torque;
}

uint8_t Gripper::getPosition() const
{
  return position_;
}

uint8_t Gripper::getTorque() const
{
  return torque_;
}

void Gripper::close()
{
  setPosition(0);
}

void Gripper::open()
{
  setPosition(100);
}
