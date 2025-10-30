#ifndef QT_ROBOT_CONTROL_RVIZ_WIDGET_H
#define QT_ROBOT_CONTROL_RVIZ_WIDGET_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QProcess>

class RVizWidget : public QWidget
{
  Q_OBJECT

public:
  explicit RVizWidget(QWidget *parent = nullptr);
  ~RVizWidget();

  void startRViz();
  void stopRViz();
  bool isRunning();

private slots:
  void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);

private:
  QProcess *rviz_process_;
  QVBoxLayout *layout_;
  QLabel *status_label_;
  
  void setupUI();
};

#endif // QT_ROBOT_CONTROL_RVIZ_WIDGET_H


