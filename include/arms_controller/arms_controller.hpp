#ifndef ARMSCONTROLLER_HPP
#define ARMSCONTROLLER_HPP

#include <QObject>
#include <QString>
#include <QList>
#include <QtQml>
#include <QVariantMap>
#include <unordered_map>
#include "arms_controller/arm_ros.hpp"
#include "arms_controller/icontroller.hpp"

class ArmsController : public QObject, public IController
{
  //QML properties (they are used by QML -> ArmControl.qml)
  Q_OBJECT
  Q_PROPERTY(QStringList armsList READ armsList CONSTANT)
  Q_PROPERTY(QVariantMap grippersPos READ grippersPos NOTIFY grippersPosChanged)

public:
  explicit ArmsController(std::shared_ptr<IROSNode> node, QObject *parent = nullptr);
  ~ArmsController();

  // QML getters
  QStringList armsList() const;
  QVariantMap grippersPos() const;

  // observer stuff
  void update();

//QML signals
signals:
  void armsListChanged(QStringList);
  void grippersPosChanged(QVariantMap);

//  Methods used by QML -> ArmControl.qml
public slots:
  void twistWrist(QString gripper, bool twist) const;
  void twistAllWrists(bool twist) const;
  void closeGripper(QString gripper) const;
  void openGripper(QString gripper) const;
  void calibrateGripper(QString gripper) const;
  void releaseGripper(QString gripper) const;
  void closeAllGrippers() const;
  void openAllGrippers() const;
  void calibrateAllGrippers() const;
  void releaseAllGrippers() const;


private:
  //QML Properties
  QVariantMap grippersPos_;
  QList<QString> armsList_; // TODO: try to enforce it to be const

  // store all arms objects
  std::unordered_map<QString, std::unique_ptr<ArmROS> >  arms_;
};

#endif // ARMSCONTROLLER_HPP
