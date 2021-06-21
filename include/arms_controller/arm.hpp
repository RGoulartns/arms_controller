#ifndef ARM_HPP
#define ARM_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <atomic>

#include "arms_controller/gripper.hpp"
#include "arms_controller/joint.hpp"


class Arm
{
public:

  explicit Arm(std::string armName, std::vector<std::string> joints);
  virtual ~Arm();

  uint8_t getId() const;
  std::string getName() const;

  uint8_t getJointPosition(const std::string name) const;
  void setJointPosition(const std::string name, const uint8_t pos);

  uint8_t getGripperPosition() const;
  uint8_t getGripperTorque() const;
  void setGripperPosition(const uint8_t pos);
  void setGripperTorque(const uint8_t torque);


private:
  // extra: added unique id to each arm. Not using it though.
  static std::atomic<uint8_t> idProvider_;
  const uint8_t id_;

  // name is used to create ROS subscription/client/action
  const std::string name_;
  // each arm can only have 1 arm
  std::unique_ptr<Gripper> gripper_;
  // each arm can have 'n' joints
  std::unordered_map<std::string, std::unique_ptr<Joint> > joints_;
};

#endif // ARM_HPP
