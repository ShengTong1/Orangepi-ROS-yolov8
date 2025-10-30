#include "qt_robot_control/MainWindow.h"
#include <QApplication>
#include <QStyleFactory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Create Qt application
  QApplication app(argc, argv);
  
  // Set application style
  app.setStyle(QStyleFactory::create("Fusion"));
  
  // Set dark color palette
  QPalette darkPalette;
  darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
  darkPalette.setColor(QPalette::WindowText, Qt::white);
  darkPalette.setColor(QPalette::Base, QColor(25, 25, 25));
  darkPalette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
  darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
  darkPalette.setColor(QPalette::ToolTipText, Qt::white);
  darkPalette.setColor(QPalette::Text, Qt::white);
  darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
  darkPalette.setColor(QPalette::ButtonText, Qt::white);
  darkPalette.setColor(QPalette::BrightText, Qt::red);
  darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
  darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
  darkPalette.setColor(QPalette::HighlightedText, Qt::black);
  app.setPalette(darkPalette);
  
  // Create and show main window
  MainWindow window;
  window.show();
  
  // Run application
  int result = app.exec();
  
  // Shutdown ROS2
  rclcpp::shutdown();
  
  return result;
}


