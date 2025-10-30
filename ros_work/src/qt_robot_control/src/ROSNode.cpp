#include "qt_robot_control/ROSNode.h"
#include <iostream>

ROSNode::ROSNode(QObject *parent) 
  : QObject(parent), Node("qt_robot_control_node")
{
  // Create subscribers
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&ROSNode::odomCallback, this, std::placeholders::_1));
  
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu/data_raw", 10, std::bind(&ROSNode::imuCallback, this, std::placeholders::_1));
  
  battery_sub_ = create_subscription<std_msgs::msg::Float32>(
    "PowerVoltage", 10, std::bind(&ROSNode::batteryCallback, this, std::placeholders::_1));
  
  // Create timers
  topics_timer_ = new QTimer(this);
  connect(topics_timer_, &QTimer::timeout, this, &ROSNode::updateTopicsList);
  topics_timer_->start(2000); // Update every 2 seconds
  
  nodes_timer_ = new QTimer(this);
  connect(nodes_timer_, &QTimer::timeout, this, &ROSNode::updateNodesList);
  nodes_timer_->start(2000); // Update every 2 seconds
  
  RCLCPP_INFO(this->get_logger(), "ROSNode initialized");
}

ROSNode::~ROSNode()
{
  topics_timer_->stop();
  nodes_timer_->stop();
}

void ROSNode::spin_some()
{
  rclcpp::spin_some(shared_from_this());
}

void ROSNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double vx = msg->twist.twist.linear.x;
  double vy = msg->twist.twist.linear.y;
  double vz = msg->twist.twist.angular.z;
  
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  
  // Extract yaw from quaternion
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  
  double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  
  emit odometryUpdated(vx, vy, vz, x, y, yaw);
}

void ROSNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double ax = msg->linear_acceleration.x;
  double ay = msg->linear_acceleration.y;
  double az = msg->linear_acceleration.z;
  
  double wx = msg->angular_velocity.x;
  double wy = msg->angular_velocity.y;
  double wz = msg->angular_velocity.z;
  
  double qx = msg->orientation.x;
  double qy = msg->orientation.y;
  double qz = msg->orientation.z;
  double qw = msg->orientation.w;
  
  emit imuUpdated(ax, ay, az, wx, wy, wz, qx, qy, qz, qw);
}

void ROSNode::batteryCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  emit batteryVoltageUpdated(msg->data);
}

void ROSNode::updateTopicsList()
{
  QStringList topics;
  
  // Get all topics
  auto topic_list = this->get_topic_names_and_types();
  
  for (const auto &topic_info : topic_list) {
    topics << QString::fromStdString(topic_info.first);
  }
  
  topics.sort();
  emit topicsListUpdated(topics);
}

void ROSNode::updateNodesList()
{
  QStringList nodes;
  
  // Get all nodes
  auto node_list = this->get_node_names();
  
  for (const auto &node_name : node_list) {
    nodes << QString::fromStdString(node_name);
  }
  
  nodes.sort();
  emit nodesListUpdated(nodes);
}


