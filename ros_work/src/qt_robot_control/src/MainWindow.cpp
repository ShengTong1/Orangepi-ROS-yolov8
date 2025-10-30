#include "qt_robot_control/MainWindow.h"
#include "qt_robot_control/ROSNode.h"
#include "qt_robot_control/RosImageSubscriber.h"
#include "qt_robot_control/SystemMonitorWidget.h"
#include "qt_robot_control/RVizWidget.h"
#include <QApplication>

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent)
  , ros_node_(nullptr)
  , rgb_subscriber_(nullptr)
  , depth_subscriber_(nullptr)
  , system_monitor_(nullptr)
  , rviz_widget_(nullptr)
  , update_timer_(new QTimer(this))
  , log_file_(nullptr)
  , log_stream_(nullptr)
{
  setWindowTitle("机器人控制面板");
  setMinimumSize(1024, 600);
  resize(1024, 600);
  
  // Create log file
  current_log_file_ = getLogFileName();
  log_file_ = new QFile(current_log_file_);
  if (log_file_->open(QIODevice::WriteOnly | QIODevice::Append)) {
    log_stream_ = new QTextStream(log_file_);
    *log_stream_ << "\n=== Session started at " << QDateTime::currentDateTime().toString() << " ===\n";
  }
  
  setupUI();
  startROSNode();
  
  // Update timer
  connect(update_timer_, &QTimer::timeout, this, &MainWindow::onUpdateTimer);
  update_timer_->start(33); // ~30fps
  
  // Initial log message
  saveLog("INFO", "系统初始化完成");
}

MainWindow::~MainWindow()
{
  // Stop all processes
  for (auto &proc_info : processes_) {
    if (proc_info.process && proc_info.process->state() == QProcess::Running) {
      proc_info.process->terminate();
      proc_info.process->waitForFinished(2000);
    }
  }
  
  // Close log file
  if (log_stream_) {
    *log_stream_ << "=== Session ended at " << QDateTime::currentDateTime().toString() << " ===\n";
    log_stream_->flush();
    delete log_stream_;
  }
  if (log_file_) {
    log_file_->close();
    delete log_file_;
  }
}

void MainWindow::setupUI()
{
  central_widget_ = new QWidget(this);
  setCentralWidget(central_widget_);
  
  main_layout_ = new QHBoxLayout(central_widget_);
  main_layout_->setSpacing(5);
  
  setupFunctionButtons();
  setupCameraDisplay();
  setupLayout();
}

void MainWindow::setupFunctionButtons()
{
  // Function panel on the left
  function_panel_ = new QWidget(this);
  function_panel_->setFixedWidth(180);
  function_panel_->setStyleSheet("background-color: #34495e;");
  
  function_layout_ = new QVBoxLayout(function_panel_);
  function_layout_->setSpacing(10);
  function_layout_->setContentsMargins(10, 10, 10, 10);
  
  // Title
  QLabel *title = new QLabel("功能控制", function_panel_);
  title->setStyleSheet("color: white; font-size: 16px; font-weight: bold; padding: 10px;");
  title->setAlignment(Qt::AlignCenter);
  function_layout_->addWidget(title);
  
  // Buttons
  btn_base_ = new QPushButton("底盘控制", function_panel_);
  btn_base_->setStyleSheet("QPushButton { background-color: #3498db; color: white; padding: 12px; border-radius: 5px; font-size: 14px; } QPushButton:pressed { background-color: #2980b9; }");
  connect(btn_base_, &QPushButton::clicked, this, &MainWindow::onStartBaseClicked);
  function_layout_->addWidget(btn_base_);
  
  btn_lidar_ = new QPushButton("激光雷达", function_panel_);
  btn_lidar_->setStyleSheet("QPushButton { background-color: #3498db; color: white; padding: 12px; border-radius: 5px; font-size: 14px; } QPushButton:pressed { background-color: #2980b9; }");
  connect(btn_lidar_, &QPushButton::clicked, this, &MainWindow::onStartLidarClicked);
  function_layout_->addWidget(btn_lidar_);
  
  btn_camera_ = new QPushButton("摄像头", function_panel_);
  btn_camera_->setStyleSheet("QPushButton { background-color: #3498db; color: white; padding: 12px; border-radius: 5px; font-size: 14px; } QPushButton:pressed { background-color: #2980b9; }");
  connect(btn_camera_, &QPushButton::clicked, this, &MainWindow::onStartCameraClicked);
  function_layout_->addWidget(btn_camera_);
  
  btn_gemini_ = new QPushButton("Gemini深度相机", function_panel_);
  btn_gemini_->setStyleSheet("QPushButton { background-color: #3498db; color: white; padding: 12px; border-radius: 5px; font-size: 14px; } QPushButton:pressed { background-color: #2980b9; }");
  connect(btn_gemini_, &QPushButton::clicked, this, &MainWindow::onStartGeminiClicked);
  function_layout_->addWidget(btn_gemini_);
  
  btn_all_sensors_ = new QPushButton("一键启动传感器", function_panel_);
  btn_all_sensors_->setStyleSheet("QPushButton { background-color: #2ecc71; color: white; padding: 12px; border-radius: 5px; font-size: 14px; } QPushButton:pressed { background-color: #27ae60; }");
  connect(btn_all_sensors_, &QPushButton::clicked, this, &MainWindow::onStartAllSensorsClicked);
  function_layout_->addWidget(btn_all_sensors_);
  
  btn_keyboard_ = new QPushButton("键盘控制", function_panel_);
  btn_keyboard_->setStyleSheet("QPushButton { background-color: #9b59b6; color: white; padding: 12px; border-radius: 5px; font-size: 14px; } QPushButton:pressed { background-color: #8e44ad; }");
  connect(btn_keyboard_, &QPushButton::clicked, this, &MainWindow::onStartKeyboardControlClicked);
  function_layout_->addWidget(btn_keyboard_);
  
  btn_gpio_ = new QPushButton("GPIO控制", function_panel_);
  btn_gpio_->setStyleSheet("QPushButton { background-color: #9b59b6; color: white; padding: 12px; border-radius: 5px; font-size: 14px; } QPushButton:pressed { background-color: #8e44ad; }");
  connect(btn_gpio_, &QPushButton::clicked, this, &MainWindow::onStartGpioControlClicked);
  function_layout_->addWidget(btn_gpio_);
  
  function_layout_->addStretch();
  
  main_layout_->addWidget(function_panel_);
}

void MainWindow::setupCameraDisplay()
{
  // Camera panel in the center-right
  camera_panel_ = new QWidget(this);
  camera_panel_->setStyleSheet("background-color: white;");
  
  camera_layout_ = new QVBoxLayout(camera_panel_);
  camera_layout_->setSpacing(5);
  camera_layout_->setContentsMargins(5, 5, 5, 5);
  
  // Title
  QLabel *camera_title = new QLabel("摄像头显示", camera_panel_);
  camera_title->setStyleSheet("color: #333; font-size: 14px; font-weight: bold; padding: 5px;");
  camera_layout_->addWidget(camera_title);
  
  // Topic selection
  QHBoxLayout *topic_layout = new QHBoxLayout();
  
  rgb_topic_combo_ = new QComboBox(camera_panel_);
  rgb_topic_combo_->addItem("/image_raw");
  rgb_topic_combo_->addItem("/camera/rgb/image_raw");
  connect(rgb_topic_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onRgbTopicChanged);
  topic_layout->addWidget(new QLabel("RGB话题:", camera_panel_));
  topic_layout->addWidget(rgb_topic_combo_);
  
  depth_topic_combo_ = new QComboBox(camera_panel_);
  depth_topic_combo_->addItem("/camera/depth/image_raw");
  depth_topic_combo_->addItem("/depth/image_raw");
  connect(depth_topic_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onDepthTopicChanged);
  topic_layout->addWidget(new QLabel("深度话题:", camera_panel_));
  topic_layout->addWidget(depth_topic_combo_);
  
  camera_layout_->addLayout(topic_layout);
  
  // Image displays
  QHBoxLayout *image_layout = new QHBoxLayout();
  
  rgb_image_label_ = new QLabel(camera_panel_);
  rgb_image_label_->setMinimumSize(200, 150);
  rgb_image_label_->setMaximumSize(200, 150);
  rgb_image_label_->setStyleSheet("border: 1px solid #bdc3c7; background-color: black;");
  rgb_image_label_->setAlignment(Qt::AlignCenter);
  rgb_image_label_->setText("等待图像...");
  rgb_image_label_->setStyleSheet("border: 1px solid #bdc3c7; background-color: black; color: white;");
  image_layout->addWidget(rgb_image_label_);
  
  depth_image_label_ = new QLabel(camera_panel_);
  depth_image_label_->setMinimumSize(200, 150);
  depth_image_label_->setMaximumSize(200, 150);
  depth_image_label_->setStyleSheet("border: 1px solid #bdc3c7; background-color: black;");
  depth_image_label_->setAlignment(Qt::AlignCenter);
  depth_image_label_->setText("等待图像...");
  depth_image_label_->setStyleSheet("border: 1px solid #bdc3c7; background-color: black; color: white;");
  image_layout->addWidget(depth_image_label_);
  
  camera_layout_->addLayout(image_layout);
  camera_layout_->addStretch();
}

void MainWindow::setupLayout()
{
  // Left: RViz and camera
  QVBoxLayout *left_layout = new QVBoxLayout();
  
  // RViz widget
  rviz_widget_ = new RVizWidget(this);
  rviz_widget_->setMinimumHeight(250);
  left_layout->addWidget(rviz_widget_);
  
  // Start RViz button
  QPushButton *start_rviz_btn = new QPushButton("启动RViz", this);
  start_rviz_btn->setStyleSheet("QPushButton { background-color: #e67e22; color: white; padding: 8px; border-radius: 3px; } QPushButton:pressed { background-color: #d35400; }");
  connect(start_rviz_btn, &QPushButton::clicked, rviz_widget_, &RVizWidget::startRViz);
  left_layout->addWidget(start_rviz_btn);
  
  left_layout->addWidget(camera_panel_);
  left_layout->setStretchFactor(camera_panel_, 1);
  
  // Right: System monitor
  system_monitor_ = new SystemMonitorWidget(this);
  QScrollArea *scroll = new QScrollArea(this);
  scroll->setWidget(system_monitor_);
  scroll->setWidgetResizable(true);
  scroll->setMinimumWidth(220);
  scroll->setMaximumWidth(220);
  
  // Add layouts
  main_layout_->addLayout(left_layout);
  main_layout_->addWidget(scroll);
}

void MainWindow::startROSNode()
{
  // Initialize ROS node in a separate thread
  ros_node_ = new ROSNode(this);
  
  // Connect signals
  connect(ros_node_, &ROSNode::batteryVoltageUpdated, system_monitor_, &SystemMonitorWidget::updateBatteryVoltage);
  connect(ros_node_, &ROSNode::odometryUpdated, system_monitor_, &SystemMonitorWidget::updateOdometry);
  connect(ros_node_, &ROSNode::imuUpdated, system_monitor_, &SystemMonitorWidget::updateImuData);
  connect(ros_node_, &ROSNode::topicsListUpdated, system_monitor_, &SystemMonitorWidget::updateTopicsList);
  connect(ros_node_, &ROSNode::nodesListUpdated, system_monitor_, &SystemMonitorWidget::updateNodesList);
  connect(ros_node_, &ROSNode::logMessage, system_monitor_, &SystemMonitorWidget::addLogMessage);
  connect(ros_node_, &ROSNode::logMessage, this, [this](const QString &level, const QString &msg) {
    saveLog(level, msg);
  });
  
  // Create image subscribers with proper node management
  rgb_subscriber_ = new RosImageSubscriber("/image_raw", this);
  depth_subscriber_ = new RosImageSubscriber("/camera/depth/image_raw", this);
  
  connect(rgb_subscriber_, &RosImageSubscriber::imageReceived, this, [this](const QPixmap &pixmap) {
    if (!pixmap.isNull()) {
      rgb_image_label_->setPixmap(pixmap.scaled(rgb_image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
  });
  
  connect(depth_subscriber_, &RosImageSubscriber::imageReceived, this, [this](const QPixmap &pixmap) {
    if (!pixmap.isNull()) {
      depth_image_label_->setPixmap(pixmap.scaled(depth_image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
  });
  
  saveLog("INFO", "ROS节点初始化完成");
}

// Function button handlers
void MainWindow::onStartBaseClicked()
{
  QString process_name = "底盘控制";
  if (isProcessRunning(process_name)) {
    stopProcess(process_name);
  } else {
    startProcess("ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py", process_name);
  }
}

void MainWindow::onStartLidarClicked()
{
  QString process_name = "激光雷达";
  if (isProcessRunning(process_name)) {
    stopProcess(process_name);
  } else {
    startProcess("ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py", process_name);
  }
}

void MainWindow::onStartCameraClicked()
{
  QString process_name = "摄像头";
  if (isProcessRunning(process_name)) {
    stopProcess(process_name);
  } else {
    startProcess("ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py", process_name);
  }
}

void MainWindow::onStartGeminiClicked()
{
  QString process_name = "Gemini深度相机";
  if (isProcessRunning(process_name)) {
    stopProcess(process_name);
  } else {
    startProcess("ros2 launch astra_camera astra_uvc_rgb.launch.xml", process_name);
  }
}

void MainWindow::onStartAllSensorsClicked()
{
  QString process_name = "一键启动传感器";
  if (isProcessRunning(process_name)) {
    stopProcess(process_name);
  } else {
    startProcess("ros2 launch turn_on_wheeltec_robot wheeltec_sensors.launch.py", process_name);
  }
}

void MainWindow::onStartKeyboardControlClicked()
{
  QString process_name = "键盘控制";
  if (isProcessRunning(process_name)) {
    stopProcess(process_name);
  } else {
    startProcess("ros2 run wheeltec_robot_keyboard wheeltec_keyboard", process_name);
  }
}

void MainWindow::onStartGpioControlClicked()
{
  QString process_name = "GPIO控制";
  if (isProcessRunning(process_name)) {
    stopProcess(process_name);
  } else {
    startProcess("ros2 run wheeltec_gpio_control gpio_control_node", process_name);
  }
}

// Image topic handlers
void MainWindow::onRgbTopicChanged(int index)
{
  if (rgb_subscriber_ && index >= 0 && index < rgb_topic_combo_->count()) {
    QString topic = rgb_topic_combo_->itemText(index);
    rgb_subscriber_->subscribeToTopic(topic.toStdString());
    saveLog("INFO", QString("切换到RGB话题: %1").arg(topic));
  }
}

void MainWindow::onDepthTopicChanged(int index)
{
  if (depth_subscriber_ && index >= 0 && index < depth_topic_combo_->count()) {
    QString topic = depth_topic_combo_->itemText(index);
    depth_subscriber_->subscribeToTopic(topic.toStdString());
    saveLog("INFO", QString("切换到深度话题: %1").arg(topic));
  }
}

// Timer handler
void MainWindow::onUpdateTimer()
{
  // Spin all ROS nodes to process incoming messages
  if (ros_node_) {
    try {
      ros_node_->spin_some();
    } catch (const std::exception &e) {
      // Log error but continue running
      saveLog("ERROR", QString("ROS spin error: %1").arg(e.what()));
    }
  }
  
  // Spin image subscribers
  if (rgb_subscriber_) {
    try {
      rgb_subscriber_->spin_some();
    } catch (const std::exception &e) {
      saveLog("ERROR", QString("RGB subscriber spin error: %1").arg(e.what()));
    }
  }
  
  if (depth_subscriber_) {
    try {
      depth_subscriber_->spin_some();
    } catch (const std::exception &e) {
      saveLog("ERROR", QString("Depth subscriber spin error: %1").arg(e.what()));
    }
  }
  
  updateFunctionButtonStatus();
}

// Process management
void MainWindow::startProcess(const QString &command, const QString &process_name)
{
  ProcessInfo proc_info;
  proc_info.name = process_name;
  proc_info.process = new QProcess(this);
  
  connect(proc_info.process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, [this, process_name](int code, QProcess::ExitStatus status) {
            onProcessFinished(code, status, process_name);
          });
  
  proc_info.process->start("bash", QStringList() << "-c" << command);
  
  // Find corresponding button
  if (process_name == "底盘控制") proc_info.button = btn_base_;
  else if (process_name == "激光雷达") proc_info.button = btn_lidar_;
  else if (process_name == "摄像头") proc_info.button = btn_camera_;
  else if (process_name == "Gemini深度相机") proc_info.button = btn_gemini_;
  else if (process_name == "一键启动传感器") proc_info.button = btn_all_sensors_;
  else if (process_name == "键盘控制") proc_info.button = btn_keyboard_;
  else if (process_name == "GPIO控制") proc_info.button = btn_gpio_;
  
  processes_.append(proc_info);
  
  saveLog("INFO", QString("启动进程: %1").arg(process_name));
  
  updateFunctionButtonStatus();
}

void MainWindow::stopProcess(const QString &process_name)
{
  for (auto it = processes_.begin(); it != processes_.end(); ++it) {
    if (it->name == process_name && it->process) {
      it->process->terminate();
      
      if (!it->process->waitForFinished(2000)) {
        it->process->kill();
      }
      
      processes_.erase(it);
      break;
    }
  }
  
  saveLog("INFO", QString("停止进程: %1").arg(process_name));
  
  updateFunctionButtonStatus();
}

bool MainWindow::isProcessRunning(const QString &process_name)
{
  for (const auto &proc_info : processes_) {
    if (proc_info.name == process_name) {
      return proc_info.process && proc_info.process->state() == QProcess::Running;
    }
  }
  return false;
}

void MainWindow::updateFunctionButtonStatus()
{
  for (const auto &proc_info : processes_) {
    if (proc_info.button) {
      bool running = proc_info.process && proc_info.process->state() == QProcess::Running;
      
      if (running) {
        proc_info.button->setStyleSheet("QPushButton { background-color: #e74c3c; color: white; padding: 12px; border-radius: 5px; font-size: 14px; } QPushButton:pressed { background-color: #c0392b; }");
        proc_info.button->setText(proc_info.name + " (运行中)");
      }
    }
  }
}

void MainWindow::onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus, const QString &process_name)
{
  Q_UNUSED(exitCode)
  Q_UNUSED(exitStatus)
  
  updateFunctionButtonStatus();
  saveLog("INFO", QString("进程结束: %1").arg(process_name));
}

// Logging
void MainWindow::saveLog(const QString &level, const QString &message)
{
  QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
  QString log_entry = QString("[%1] [%2] %3").arg(timestamp, level, message);
  
  if (log_stream_) {
    *log_stream_ << log_entry << "\n";
    log_stream_->flush();
  }
}

QString MainWindow::getLogFileName()
{
  QDir log_dir = QDir::home();
  log_dir.mkpath("robot_logs");
  
  QString log_filename = QString("robot_logs/log_%1.txt")
                          .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
  
  return log_dir.absoluteFilePath(log_filename);
}

