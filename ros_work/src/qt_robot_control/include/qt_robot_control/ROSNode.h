#ifndef QT_ROBOT_CONTROL_ROS_NODE_H
#define QT_ROBOT_CONTROL_ROS_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <QObject>
#include <QTimer>
#include <QString>

// Forward declaration
class MainWindow;

class ROSNode : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  explicit ROSNode(QObject *parent = nullptr);
  ~ROSNode();

  void spin_some();

signals:
  void batteryVoltageUpdated(float voltage);
  void odometryUpdated(double vx, double vy, double vz, double x, double y, double z);
  void imuUpdated(double ax, double ay, double az, double wx, double wy, double wz, double qx, double qy, double qz, double qw);
  void topicsListUpdated(const QStringList &topics);
  void nodesListUpdated(const QStringList &nodes);
  void logMessage(const QString &level, const QString &message);

private slots:
  void updateTopicsList();
  void updateNodesList();

private:
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;

  // Callbacks
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg);

  // Timers
  QTimer *topics_timer_;
  QTimer *nodes_timer_;
};

#endif // QT_ROBOT_CONTROL_ROS_NODE_H


