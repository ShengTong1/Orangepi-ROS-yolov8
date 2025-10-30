cmake_minimum_required(VERSION 3.8)
project(qt_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

# Enable MOC, UIC, RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Qt5 Core components
find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)

# Include directories
include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
)

# Source files
set(PROJECT_SOURCES
  src/main.cpp
  src/MainWindow.cpp
  src/ROSNode.cpp
  src/RosImageSubscriber.cpp
  src/RVizWidget.cpp
  src/SystemMonitorWidget.cpp
  include/qt_robot_control/MainWindow.h
  include/qt_robot_control/ROSNode.h
  include/qt_robot_control/RosImageSubscriber.h
  include/qt_robot_control/RVizWidget.h
  include/qt_robot_control/SystemMonitorWidget.h
)

# Create executable
add_executable(qt_robot_control ${PROJECT_SOURCES})

# Link all libraries together (ROS2 dependencies with ament_target_dependencies)
ament_target_dependencies(qt_robot_control
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  tf2_ros
)

# Link Qt libraries and OpenCV
target_link_libraries(qt_robot_control
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  ${OpenCV_LIBS}
)

# Install
install(TARGETS qt_robot_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

