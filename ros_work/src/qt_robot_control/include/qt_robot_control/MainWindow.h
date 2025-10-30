#ifndef QT_ROBOT_CONTROL_MAIN_WINDOW_H
#define QT_ROBOT_CONTROL_MAIN_WINDOW_H

#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QScrollArea>
#include <QComboBox>
#include <QLabel>
#include <QTimer>
#include <QProcess>
#include <QTextEdit>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QFileDialog>
#include <memory>
#include <QList>

// Forward declarations
class ROSNode;
class RosImageSubscriber;
class SystemMonitorWidget;
class RVizWidget;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

private slots:
  // Function package buttons
  void onStartBaseClicked();
  void onStartLidarClicked();
  void onStartCameraClicked();
  void onStartGeminiClicked();
  void onStartAllSensorsClicked();
  void onStartKeyboardControlClicked();
  void onStartGpioControlClicked();
  
  // Image topic selection
  void onRgbTopicChanged(int index);
  void onDepthTopicChanged(int index);
  
  // Timers
  void onUpdateTimer();
  
  // Process management
  void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus, const QString &process_name);

private:
  void setupUI();
  void setupFunctionButtons();
  void setupCameraDisplay();
  void setupLayout();
  void startROSNode();
  void updateFunctionButtonStatus();
  
  // Process management
  void startProcess(const QString &command, const QString &process_name);
  void stopProcess(const QString &process_name);
  bool isProcessRunning(const QString &process_name);
  void onProcessError(QProcess::ProcessError error, const QString &process_name);
  
  // Logging
  void saveLog(const QString &level, const QString &message);
  QString getLogFileName();
  
  // Core components
  ROSNode *ros_node_;
  RosImageSubscriber *rgb_subscriber_;
  RosImageSubscriber *depth_subscriber_;
  SystemMonitorWidget *system_monitor_;
  RVizWidget *rviz_widget_;
  
  // Process list
  struct ProcessInfo {
    QString name;
    QProcess *process;
    QPushButton *button;
  };
  QList<ProcessInfo> processes_;
  
  // UI components - Function buttons panel
  QWidget *function_panel_;
  QVBoxLayout *function_layout_;
  QPushButton *btn_base_;
  QPushButton *btn_lidar_;
  QPushButton *btn_camera_;
  QPushButton *btn_gemini_;
  QPushButton *btn_all_sensors_;
  QPushButton *btn_keyboard_;
  QPushButton *btn_gpio_;
  
  // UI components - RViz area
  QWidget *rviz_container_;
  
  // UI components - Camera display
  QWidget *camera_panel_;
  QVBoxLayout *camera_layout_;
  QComboBox *rgb_topic_combo_;
  QComboBox *depth_topic_combo_;
  QLabel *rgb_image_label_;
  QLabel *depth_image_label_;
  
  // Main layout
  QWidget *central_widget_;
  QHBoxLayout *main_layout_;
  
  // Timers
  QTimer *update_timer_;
  
  // Logging
  QString log_buffer_;
  QFile *log_file_;
  QTextStream *log_stream_;
  QString current_log_file_;
};

#endif // QT_ROBOT_CONTROL_MAIN_WINDOW_H


