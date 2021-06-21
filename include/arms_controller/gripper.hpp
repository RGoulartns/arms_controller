#ifndef GRIPPER_HPP
#define GRIPPER_HPP

#include <string>

class Gripper
{
public:
  Gripper(uint8_t torque=50);

  void setPosition(const uint8_t pos);
  void setTorque(const uint8_t torque);

  uint8_t getPosition() const;
  uint8_t getTorque() const;

  void close();
  void open();

private:
  uint8_t position_;
  uint8_t torque_;
};

#endif // GRIPPER_HPP
