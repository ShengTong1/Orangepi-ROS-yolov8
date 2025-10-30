#include "qt_robot_control/RosImageSubscriber.h"
#include <QtGui>
#include <QTimer>

RosImageSubscriber::RosImageSubscriber(const std::string &topic_name, QObject *parent)
  : QObject(parent), Node("ros_image_subscriber_" + std::to_string(reinterpret_cast<uintptr_t>(this)))
{
  current_topic_ = topic_name;
  // Delay subscription to avoid initialization issues
  QTimer::singleShot(100, this, [this, topic_name]() {
    subscribeToTopic(topic_name);
  });
}

RosImageSubscriber::~RosImageSubscriber()
{
  if (image_sub_) {
    image_sub_.reset();
  }
}

void RosImageSubscriber::subscribeToTopic(const std::string &topic_name)
{
  // Unsubscribe from old topic if exists
  if (image_sub_) {
    image_sub_.reset();
  }
  
  // Subscribe to new topic
  current_topic_ = topic_name;
  
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    topic_name, 
    10, 
    std::bind(&RosImageSubscriber::imageCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", topic_name.c_str());
}

void RosImageSubscriber::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr;
    
    // Convert ROS image message to OpenCV image
    if (msg->encoding == "bgr8") {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } else if (msg->encoding == "rgb8") {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
      // Depth image
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      
      // Normalize depth image for display
      cv::Mat depth_display;
      cv_ptr->image.convertTo(depth_display, CV_8UC1, 255.0 / 5000.0, 0);
      cv::applyColorMap(depth_display, depth_display, cv::COLORMAP_JET);
      cv_ptr->image = depth_display;
      cv_ptr->encoding = "bgr8";
    } else {
      cv_ptr = cv_bridge::toCvCopy(msg);
    }
    
    // Convert OpenCV Mat to QImage
    QImage qimage = cvMatToQImage(cv_ptr->image);
    
    // Convert to QPixmap for display
    QPixmap pixmap = QPixmap::fromImage(qimage);
    
    emit imageReceived(pixmap);
    
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void RosImageSubscriber::spin_some()
{
  rclcpp::spin_some(shared_from_this());
}

QImage RosImageSubscriber::cvMatToQImage(const cv::Mat &mat)
{
  switch (mat.type()) {
    case CV_8UC4: {
      QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGBA8888);
      return image.copy();
    }
    case CV_8UC3: {
      QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_BGR888);
      return image.copy();
    }
    case CV_8UC1: {
      QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
      return image.copy();
    }
    default:
      qWarning() << "Unsupported CV mat type:" << mat.type();
      return QImage();
  }
}

