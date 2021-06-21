#include "../include/arms_controller/arms_controller.hpp"

ArmsController::ArmsController(std::shared_ptr<IROSNode> node, QObject *parent)
  : QObject(parent)
  , IController(node)
{
  // TODO: ROSPARAM
  armsList_.push_back("left_arm");
  armsList_.push_back("right_arm");
  armsList_.push_back("middle_arm");
  armsList_.push_back("bottom_arm");
  // Joints are hardcoded for now and twist position is boolean


  for(const auto & armName : qAsConst(armsList_))
  {
    try
    {
      arms_.emplace(armName, std::make_unique<ArmROS>(armName.toStdString(), node_));
      grippersPos_[armName] = 0;
    }
    catch (const std::runtime_error& error)
    {
      //TODO: create error dialog box to the user
      //emit qwer();
    }
  }
}

ArmsController::~ArmsController(){}

//QML Properties
QStringList ArmsController::armsList() const { return armsList_; }
QVariantMap ArmsController::grippersPos() const { return grippersPos_; }

void ArmsController::update()
{
  for(const auto& name : qAsConst(armsList_))
    grippersPos_[name] = arms_.at(name)->getGripperPosition();
  emit grippersPosChanged(grippersPos_);
}

void ArmsController::twistWrist(QString gripper, bool twist) const
{
  arms_.at(gripper)->setJointPosition("wrist", twist);
}

void ArmsController::twistAllWrists(bool twist) const
{
  for(const auto& name : armsList_)
    arms_.at(name)->setJointPosition("wrist", twist);
}

void ArmsController::closeGripper(QString gripper) const
{
  arms_.at(gripper)->setGripperPosition(0, 15);
}

void ArmsController::openGripper(QString gripper) const
{
  arms_.at(gripper)->setGripperPosition(100, 15);
}

void ArmsController::calibrateGripper(QString gripper) const
{
  arms_.at(gripper)->calibrateGripper();
}

void ArmsController::releaseGripper(QString gripper) const
{
  arms_.at(gripper)->releaseGripper();
}

void ArmsController::closeAllGrippers() const
{
  for(const auto& name : armsList_)
    arms_.at(name)->setGripperPosition(0, 100);
}

void ArmsController::openAllGrippers() const
{
  for(const auto& name : armsList_ )
    arms_.at(name)->setGripperPosition(100, 100);
}

void ArmsController::calibrateAllGrippers() const
{
  for(const auto& name : armsList_ )
    arms_.at(name)->calibrateGripper();
}

void ArmsController::releaseAllGrippers() const
{
  for(const auto& name : armsList_ )
    arms_.at(name)->releaseGripper();
}
