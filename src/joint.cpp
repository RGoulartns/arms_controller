#include "../include/arms_controller/joint.hpp"

Joint::Joint(std::string name)
  : name_(name)
  , position_(0)
{}

void Joint::setPosition(const uint8_t pos)
{
  position_ = pos;
}

uint8_t Joint::getPosition() const
{
  return position_;
}
