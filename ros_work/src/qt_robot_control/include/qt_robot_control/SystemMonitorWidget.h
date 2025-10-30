#ifndef QT_ROBOT_CONTROL_SYSTEM_MONITOR_WIDGET_H
#define QT_ROBOT_CONTROL_SYSTEM_MONITOR_WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QTreeWidget>
#include <QTimer>

class SystemMonitorWidget : public QWidget
{
  Q_OBJECT

public:
  explicit SystemMonitorWidget(QWidget *parent = nullptr);

public slots:
  void updateBatteryVoltage(float voltage);
  void updateOdometry(double vx, double vy, double vz, double x, double y, double yaw);
  void updateImuData(double ax, double ay, double az, double wx, double wy, double wz, double qx, double qy, double qz, double qw);
  void updateTopicsList(const QStringList &topics);
  void updateNodesList(const QStringList &nodes);
  void addLogMessage(const QString &level, const QString &message);

private:
  QVBoxLayout *main_layout_;
  
  // Battery
  QLabel *battery_label_;
  QLabel *battery_value_;
  
  // Speed
  QLabel *speed_label_;
  QLabel *speed_value_;
  QLabel *angular_speed_label_;
  QLabel *angular_speed_value_;
  
  // Position
  QLabel *position_label_;
  QLabel *position_value_;
  
  // IMU
  QLabel *imu_accel_label_;
  QLabel *imu_accel_value_;
  QLabel *imu_gyro_label_;
  QLabel *imu_gyro_value_;
  QLabel *imu_quat_label_;
  QLabel *imu_quat_value_;
  
  // Topics
  QTreeWidget *topics_tree_;
  
  // Nodes
  QTreeWidget *nodes_tree_;
  
  // Log
  QTextEdit *log_text_;
  
  void setupUI();
  void clearLog();
};

#endif // QT_ROBOT_CONTROL_SYSTEM_MONITOR_WIDGET_H


