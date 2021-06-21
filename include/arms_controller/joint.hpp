#ifndef JOINT_HPP
#define JOINT_HPP

#include <string>

class Joint
{
public:
  Joint(std::string name);

  void setPosition(const uint8_t pos);
  uint8_t getPosition() const;

private:
  const std::string name_;
  uint8_t position_;
};

#endif // JOINT_HPP
