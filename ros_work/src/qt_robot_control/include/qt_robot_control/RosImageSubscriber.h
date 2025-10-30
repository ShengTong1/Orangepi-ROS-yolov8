#ifndef QT_ROBOT_CONTROL_ROS_IMAGE_SUBSCRIBER_H
#define QT_ROBOT_CONTROL_ROS_IMAGE_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <QObject>
#include <QPixmap>
#include <QImage>
#include <memory>

class RosImageSubscriber : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  explicit RosImageSubscriber(const std::string &topic_name, QObject *parent = nullptr);
  ~RosImageSubscriber();

  void subscribeToTopic(const std::string &topic_name);
  std::string getCurrentTopic() const { return current_topic_; }
  void spin_some();

signals:
  void imageReceived(const QPixmap &pixmap);

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  QImage cvMatToQImage(const cv::Mat &mat);

  std::string current_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

#endif // QT_ROBOT_CONTROL_ROS_IMAGE_SUBSCRIBER_H


