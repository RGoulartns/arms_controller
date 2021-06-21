#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QVector>
#include <QPair>
#include <QString>
#include <QProcess>

#include "arms_controller/ros_node.hpp"
#include "arms_controller/arms_controller.hpp"

/**
 * TODO
 * read param file
 * user control torque
 */

int main(int argc, char *argv[])
{
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

  //ROS2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ROSNode>();
  node->start();
  auto armsController = std::make_shared<ArmsController>(node);
  node->bind(std::dynamic_pointer_cast<IController>(armsController));


  //create GUI stuff after c++ classes to ensure proper destruction
  const QUrl url(QStringLiteral("qrc:/main.qml"));
  QGuiApplication app(argc, argv);
  QQmlApplicationEngine engine;

  //This exposes the ArmsController class to QML
  engine.rootContext()->setContextProperty("ArmsController", armsController.get());

  //debugging feature: warns if imperative instructions destroy bindings
  QLoggingCategory::setFilterRules(QStringLiteral("qt.qml.binding.removal.info=true"));

  QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                   &app, [url](QObject *obj, const QUrl &objUrl) {
    if (!obj && url == objUrl)
      QCoreApplication::exit(-1);
  }, Qt::QueuedConnection);
  engine.load(url);

  return app.exec();
}
