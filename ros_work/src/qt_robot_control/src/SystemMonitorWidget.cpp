#include "qt_robot_control/SystemMonitorWidget.h"
#include <QPushButton>

SystemMonitorWidget::SystemMonitorWidget(QWidget *parent)
  : QWidget(parent)
{
  setupUI();
}

void SystemMonitorWidget::setupUI()
{
  main_layout_ = new QVBoxLayout(this);
  
  // Battery section
  battery_label_ = new QLabel("电池电压:", this);
  battery_label_->setStyleSheet("font-weight: bold; color: #333;");
  battery_value_ = new QLabel("N/A", this);
  battery_value_->setStyleSheet("color: #2ecc71; font-size: 14px;");
  main_layout_->addWidget(battery_label_);
  main_layout_->addWidget(battery_value_);
  
  // Speed section
  speed_label_ = new QLabel("速度:", this);
  speed_label_->setStyleSheet("font-weight: bold; color: #333; margin-top: 10px;");
  speed_value_ = new QLabel("N/A", this);
  speed_value_->setStyleSheet("font-size: 12px;");
  main_layout_->addWidget(speed_label_);
  main_layout_->addWidget(speed_value_);
  
  angular_speed_label_ = new QLabel("角速度:", this);
  angular_speed_label_->setStyleSheet("font-weight: bold; color: #333;");
  angular_speed_value_ = new QLabel("N/A", this);
  angular_speed_value_->setStyleSheet("font-size: 12px;");
  main_layout_->addWidget(angular_speed_label_);
  main_layout_->addWidget(angular_speed_value_);
  
  // Position section
  position_label_ = new QLabel("位置:", this);
  position_label_->setStyleSheet("font-weight: bold; color: #333; margin-top: 10px;");
  position_value_ = new QLabel("N/A", this);
  position_value_->setStyleSheet("font-size: 12px;");
  main_layout_->addWidget(position_label_);
  main_layout_->addWidget(position_value_);
  
  // IMU section
  imu_accel_label_ = new QLabel("IMU 加速度:", this);
  imu_accel_label_->setStyleSheet("font-weight: bold; color: #333; margin-top: 10px;");
  imu_accel_value_ = new QLabel("N/A", this);
  imu_accel_value_->setStyleSheet("font-size: 11px;");
  main_layout_->addWidget(imu_accel_label_);
  main_layout_->addWidget(imu_accel_value_);
  
  imu_gyro_label_ = new QLabel("IMU 角速度:", this);
  imu_gyro_label_->setStyleSheet("font-weight: bold; color: #333;");
  imu_gyro_value_ = new QLabel("N/A", this);
  imu_gyro_value_->setStyleSheet("font-size: 11px;");
  main_layout_->addWidget(imu_gyro_label_);
  main_layout_->addWidget(imu_gyro_value_);
  
  imu_quat_label_ = new QLabel("IMU 四元数:", this);
  imu_quat_label_->setStyleSheet("font-weight: bold; color: #333;");
  imu_quat_value_ = new QLabel("N/A", this);
  imu_quat_value_->setStyleSheet("font-size: 11px;");
  main_layout_->addWidget(imu_quat_label_);
  main_layout_->addWidget(imu_quat_value_);
  
  // Topics tree
  QLabel *topics_header = new QLabel("话题列表:", this);
  topics_header->setStyleSheet("font-weight: bold; color: #333; margin-top: 10px;");
  main_layout_->addWidget(topics_header);
  
  topics_tree_ = new QTreeWidget(this);
  topics_tree_->setHeaderLabel("话题名称");
  topics_tree_->setMaximumHeight(100);
  topics_tree_->setRootIsDecorated(false);
  main_layout_->addWidget(topics_tree_);
  
  // Nodes tree
  QLabel *nodes_header = new QLabel("节点列表:", this);
  nodes_header->setStyleSheet("font-weight: bold; color: #333; margin-top: 10px;");
  main_layout_->addWidget(nodes_header);
  
  nodes_tree_ = new QTreeWidget(this);
  nodes_tree_->setHeaderLabel("节点名称");
  nodes_tree_->setMaximumHeight(100);
  nodes_tree_->setRootIsDecorated(false);
  main_layout_->addWidget(nodes_tree_);
  
  // Log section
  QLabel *log_header = new QLabel("日志:", this);
  log_header->setStyleSheet("font-weight: bold; color: #333; margin-top: 10px;");
  main_layout_->addWidget(log_header);
  
  QHBoxLayout *log_buttons = new QHBoxLayout();
  QPushButton *clear_log_btn = new QPushButton("清除日志", this);
  connect(clear_log_btn, &QPushButton::clicked, this, &SystemMonitorWidget::clearLog);
  log_buttons->addWidget(clear_log_btn);
  log_buttons->addStretch();
  main_layout_->addLayout(log_buttons);
  
  log_text_ = new QTextEdit(this);
  log_text_->setReadOnly(true);
  log_text_->setMaximumHeight(100);
  log_text_->setStyleSheet("background-color: #1e1e1e; color: #ffffff; font-family: monospace; font-size: 10px;");
  main_layout_->addWidget(log_text_);
  
  main_layout_->addStretch();
}

void SystemMonitorWidget::updateBatteryVoltage(float voltage)
{
  battery_value_->setText(QString(" %1 V").arg(voltage, 0, 'f', 2));
  
  // Change color based on voltage
  if (voltage < 11.0) {
    battery_value_->setStyleSheet("color: #e74c3c; font-size: 14px;");
  } else if (voltage < 12.0) {
    battery_value_->setStyleSheet("color: #f39c12; font-size: 14px;");
  } else {
    battery_value_->setStyleSheet("color: #2ecc71; font-size: 14px;");
  }
}

void SystemMonitorWidget::updateOdometry(double vx, double vy, double vz, double x, double y, double yaw)
{
  speed_value_->setText(QString(" Vx: %1 m/s, Vy: %2 m/s").arg(vx, 0, 'f', 3).arg(vy, 0, 'f', 3));
  angular_speed_value_->setText(QString(" Vz: %1 rad/s").arg(vz, 0, 'f', 3));
  position_value_->setText(QString(" X: %1 m, Y: %2 m, Yaw: %3°").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw * 180.0 / 3.14159, 0, 'f', 2));
}

void SystemMonitorWidget::updateImuData(double ax, double ay, double az, double wx, double wy, double wz, double qx, double qy, double qz, double qw)
{
  imu_accel_value_->setText(QString(" Ax: %1, Ay: %2, Az: %3 m/s²").arg(ax, 0, 'f', 2).arg(ay, 0, 'f', 2).arg(az, 0, 'f', 2));
  imu_gyro_value_->setText(QString(" Wx: %1, Wy: %2, Wz: %3 rad/s").arg(wx, 0, 'f', 3).arg(wy, 0, 'f', 3).arg(wz, 0, 'f', 3));
  imu_quat_value_->setText(QString(" Q: [%1, %2, %3, %4]").arg(qx, 0, 'f', 3).arg(qy, 0, 'f', 3).arg(qz, 0, 'f', 3).arg(qw, 0, 'f', 3));
}

void SystemMonitorWidget::updateTopicsList(const QStringList &topics)
{
  topics_tree_->clear();
  
  for (const QString &topic : topics) {
    QTreeWidgetItem *item = new QTreeWidgetItem(topics_tree_);
    item->setText(0, topic);
  }
}

void SystemMonitorWidget::updateNodesList(const QStringList &nodes)
{
  nodes_tree_->clear();
  
  for (const QString &node : nodes) {
    QTreeWidgetItem *item = new QTreeWidgetItem(nodes_tree_);
    item->setText(0, node);
  }
}

void SystemMonitorWidget::addLogMessage(const QString &level, const QString &message)
{
  QString color;
  if (level == "ERROR") {
    color = "#e74c3c";
  } else if (level == "WARN") {
    color = "#f39c12";
  } else {
    color = "#3498db";
  }
  
  QString formatted_msg = QString("<span style='color: %1;'>[%2]</span> <span style='color: #ffffff;'>%3</span>")
                          .arg(color, level, message);
  
  log_text_->append(formatted_msg);
  
  // Auto-scroll to bottom
  QTextCursor cursor = log_text_->textCursor();
  cursor.movePosition(QTextCursor::End);
  log_text_->setTextCursor(cursor);
}

void SystemMonitorWidget::clearLog()
{
  log_text_->clear();
}

