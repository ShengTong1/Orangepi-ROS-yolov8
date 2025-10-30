#include "qt_robot_control/RVizWidget.h"
#include <QProcess>
#include <QVBoxLayout>
#include <QLabel>

RVizWidget::RVizWidget(QWidget *parent)
  : QWidget(parent)
{
  rviz_process_ = new QProcess(this);
  setupUI();
  
  connect(rviz_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &RVizWidget::onProcessFinished);
}

RVizWidget::~RVizWidget()
{
  stopRViz();
}

void RVizWidget::setupUI()
{
  layout_ = new QVBoxLayout(this);
  layout_->setContentsMargins(0, 0, 0, 0);
  
  status_label_ = new QLabel("RViz未运行", this);
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setStyleSheet("background-color: #ecf0f1; border: 1px solid #bdc3c7; padding: 20px;");
  layout_->addWidget(status_label_);
}

void RVizWidget::startRViz()
{
  if (rviz_process_->state() == QProcess::Running) {
    return;
  }
  
  // Start RViz in a separate window
  rviz_process_->start("rviz2", QStringList());
  
  status_label_->setText("RViz启动中...");
  status_label_->setStyleSheet("background-color: #3498db; color: white; border: 1px solid #2980b9; padding: 20px;");
}

void RVizWidget::stopRViz()
{
  if (rviz_process_->state() == QProcess::Running) {
    rviz_process_->terminate();
    
    if (!rviz_process_->waitForFinished(3000)) {
      rviz_process_->kill();
    }
  }
  
  status_label_->setText("RViz已停止");
  status_label_->setStyleSheet("background-color: #ecf0f1; border: 1px solid #bdc3c7; padding: 20px;");
}

bool RVizWidget::isRunning()
{
  return rviz_process_->state() == QProcess::Running;
}

void RVizWidget::onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
  Q_UNUSED(exitCode)
  Q_UNUSED(exitStatus)
  
  status_label_->setText("RViz已停止");
  status_label_->setStyleSheet("background-color: #ecf0f1; border: 1px solid #bdc3c7; padding: 20px;");
}


