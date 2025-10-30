#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
YOLOv8 目标检测 - 图形界面版本
美观的 PyQt5 界面
"""

import sys
import os
import numpy as np
import cv2
import yaml
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                              QHBoxLayout, QPushButton, QLabel, QFileDialog, 
                              QDoubleSpinBox, QSpinBox, QGroupBox, QMessageBox,
                              QSplitter, QTextEdit, QStatusBar, QProgressBar)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap, QImage, QFont
from mindx.sdk import base
from det_utils import *


class DetectionThread(QThread):
    """检测线程，避免界面卡顿"""
    finished = pyqtSignal(list, np.ndarray)  # result, result_image
    error = pyqtSignal(str)
    
    def __init__(self, img, model_path, conf_thres, iou_thres, class_names):
        super().__init__()
        self.img = img
        self.model_path = model_path
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.class_names = class_names
        
    def run(self):
        try:
            # 前处理
            img_after, scale, dh, dw = resize_image(self.img, (640, 640), True)
            letterbox_info = (scale, dw, dh)
            
            # 将图像处理成输入的格式
            data = img2input(img_after)
            
            # 模型推理
            model = base.model(modelPath=self.model_path, deviceId=0)
            output = model.infer([data])[0]
            
            # 后处理
            output.to_host()
            output = np.array(output)
            
            # 标准化输出
            pred = std_output(output)
            
            # 置信度过滤+nms
            result = nms(pred, self.conf_thres, self.iou_thres)
            
            # 坐标变换
            result = cod_trf(result, self.img, img_after, letterbox_info)
            
            # 绘制结果
            result_image = draw(result, self.img.copy(), self.class_names)
            
            self.finished.emit(result, result_image)
            
        except Exception as e:
            self.error.emit(str(e))


class YOLOv8GUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("YOLOv8 植物病害检测系统 - 昇腾版本")
        self.setGeometry(100, 100, 1400, 900)
        
        # 变量
        self.original_image = None
        self.result_image = None
        self.model_path = 'yolov8.om'
        self.device_id = 0
        self.class_names = []
        self.detection_results = []
        
        # 加载类别
        self.load_classes()
        
        # 初始化 mxVision
        try:
            base.mx_init()
            print("mxVision 初始化成功")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"mxVision 初始化失败: {str(e)}")
        
        # 初始化界面
        self.init_ui()
        
    def init_ui(self):
        """初始化界面"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # 创建分割器
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        # 左侧控制面板
        control_panel = self.create_control_panel()
        splitter.addWidget(control_panel)
        
        # 右侧图像显示区
        image_panel = self.create_image_panel()
        splitter.addWidget(image_panel)
        
        # 设置分割器比例
        splitter.setSizes([350, 1050])
        
        # 创建状态栏
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("就绪")
        
        # 应用样式
        self.apply_style()
        
    def create_control_panel(self):
        """创建控制面板"""
        panel = QWidget()
        panel.setMaximumWidth(350)
        layout = QVBoxLayout()
        panel.setLayout(layout)
        
        # 标题
        title = QLabel("🌿 YOLOv8 检测控制")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # 文件选择区域
        file_group = QGroupBox("📁 图像选择")
        file_layout = QVBoxLayout()
        
        self.image_path_label = QLabel("未选择图像")
        self.image_path_label.setWordWrap(True)
        self.image_path_label.setStyleSheet("padding: 5px;")
        file_layout.addWidget(self.image_path_label)
        
        self.select_image_btn = QPushButton("选择图像...")
        self.select_image_btn.clicked.connect(self.select_image)
        file_layout.addWidget(self.select_image_btn)
        
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        # 参数配置区域
        param_group = QGroupBox("⚙️ 检测参数")
        param_layout = QVBoxLayout()
        
        # 置信度阈值
        conf_layout = QHBoxLayout()
        conf_layout.addWidget(QLabel("置信度阈值:"))
        self.conf_thres_spin = QDoubleSpinBox()
        self.conf_thres_spin.setRange(0.0, 1.0)
        self.conf_thres_spin.setSingleStep(0.01)
        self.conf_thres_spin.setValue(0.25)
        self.conf_thres_spin.setDecimals(2)
        conf_layout.addWidget(self.conf_thres_spin)
        param_layout.addLayout(conf_layout)
        
        # IOU 阈值
        iou_layout = QHBoxLayout()
        iou_layout.addWidget(QLabel("IOU 阈值:"))
        self.iou_thres_spin = QDoubleSpinBox()
        self.iou_thres_spin.setRange(0.0, 1.0)
        self.iou_thres_spin.setSingleStep(0.01)
        self.iou_thres_spin.setValue(0.45)
        self.iou_thres_spin.setDecimals(2)
        iou_layout.addWidget(self.iou_thres_spin)
        param_layout.addLayout(iou_layout)
        
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        
        # 检测按钮
        self.detect_btn = QPushButton("🚀 开始检测")
        self.detect_btn.setMinimumHeight(50)
        self.detect_btn.clicked.connect(self.start_detection)
        self.detect_btn.setEnabled(False)
        layout.addWidget(self.detect_btn)
        
        # 保存结果按钮
        self.save_btn = QPushButton("💾 保存结果")
        self.save_btn.setMinimumHeight(40)
        self.save_btn.clicked.connect(self.save_result)
        self.save_btn.setEnabled(False)
        layout.addWidget(self.save_btn)
        
        # 结果显示区域
        result_group = QGroupBox("📊 检测结果")
        result_layout = QVBoxLayout()
        
        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setMaximumHeight(150)
        result_layout.addWidget(self.result_text)
        
        result_group.setLayout(result_layout)
        layout.addWidget(result_group)
        
        # 进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)
        
        layout.addStretch()
        
        return panel
        
    def create_image_panel(self):
        """创建图像显示面板"""
        panel = QWidget()
        layout = QVBoxLayout()
        panel.setLayout(layout)
        
        # 原图标签
        orig_label_title = QLabel("📸 原始图像")
        orig_label_title.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(orig_label_title)
        
        self.original_image_label = QLabel()
        self.original_image_label.setMinimumHeight(400)
        self.original_image_label.setAlignment(Qt.AlignCenter)
        self.original_image_label.setStyleSheet("border: 2px solid #4CAF50; background-color: #f0f0f0;")
        self.original_image_label.setText("等待加载图像...")
        layout.addWidget(self.original_image_label)
        
        # 结果图标签
        result_label_title = QLabel("🎯 检测结果")
        result_label_title.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(result_label_title)
        
        self.result_image_label = QLabel()
        self.result_image_label.setMinimumHeight(400)
        self.result_image_label.setAlignment(Qt.AlignCenter)
        self.result_image_label.setStyleSheet("border: 2px solid #2196F3; background-color: #f0f0f0;")
        self.result_image_label.setText("等待检测...")
        layout.addWidget(self.result_image_label)
        
        return panel
        
    def apply_style(self):
        """应用美观的样式"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #ffffff;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #cccccc;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
                background-color: #f9f9f9;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 8px;
                border-radius: 5px;
                font-size: 13px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:disabled {
                background-color: #cccccc;
            }
            QDoubleSpinBox, QSpinBox {
                border: 1px solid #ddd;
                border-radius: 3px;
                padding: 5px;
            }
            QTextEdit {
                border: 1px solid #ddd;
                border-radius: 3px;
                background-color: white;
            }
        """)
        
    def load_classes(self):
        """加载类别名称"""
        try:
            with open('data.yaml', 'r', encoding='utf-8') as f:
                yaml_data = yaml.safe_load(f)
                self.class_names = yaml_data.get('names', [])
                print(f"加载了 {len(self.class_names)} 个类别")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"加载类别文件失败: {str(e)}")
            
    def select_image(self):
        """选择图像文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择图像", "", "Image Files (*.jpg *.jpeg *.png *.bmp)"
        )
        
        if file_path:
            self.image_path = file_path
            self.image_path_label.setText(os.path.basename(file_path))
            self.load_original_image()
            self.detect_btn.setEnabled(True)
            self.statusBar.showMessage(f"已加载图像: {os.path.basename(file_path)}")
            
    def load_original_image(self):
        """加载原始图像"""
        try:
            self.original_image = cv2.imread(self.image_path)
            if self.original_image is None:
                QMessageBox.critical(self, "错误", "无法读取图像文件！")
                return
                
            # 调整图像尺寸以适应显示
            display_img = self.resize_image_for_display(self.original_image.copy())
            qt_img = self.cv2_to_qimage(display_img)
            self.original_image_label.setPixmap(QPixmap.fromImage(qt_img))
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"加载图像失败: {str(e)}")
            
    def start_detection(self):
        """开始检测"""
        if self.original_image is None:
            QMessageBox.warning(self, "警告", "请先选择图像！")
            return
            
        if not os.path.exists(self.model_path):
            QMessageBox.critical(self, "错误", f"模型文件不存在: {self.model_path}")
            return
            
        # 禁用按钮
        self.detect_btn.setEnabled(False)
        self.detect_btn.setText("检测中...")
        self.progress_bar.setVisible(True)
        self.result_image_label.setText("正在检测...")
        self.statusBar.showMessage("正在运行检测...")
        
        # 获取参数
        conf_thres = self.conf_thres_spin.value()
        iou_thres = self.iou_thres_spin.value()
        
        # 创建并启动检测线程
        self.detection_thread = DetectionThread(
            self.original_image, self.model_path, conf_thres, 
            iou_thres, self.class_names
        )
        self.detection_thread.finished.connect(self.detection_finished)
        self.detection_thread.error.connect(self.detection_error)
        self.detection_thread.start()
        
    def detection_finished(self, result, result_image):
        """检测完成"""
        self.detection_results = result
        self.result_image = result_image
        
        # 显示结果图像
        display_img = self.resize_image_for_display(result_image.copy())
        qt_img = self.cv2_to_qimage(display_img)
        self.result_image_label.setPixmap(QPixmap.fromImage(qt_img))
        
        # 显示检测信息
        self.update_result_text(result)
        
        # 恢复按钮状态
        self.detect_btn.setEnabled(True)
        self.detect_btn.setText("🚀 开始检测")
        self.save_btn.setEnabled(True)
        self.progress_bar.setVisible(False)
        self.statusBar.showMessage(f"检测完成！发现 {len(result)} 个目标")
        
    def detection_error(self, error_msg):
        """检测出错"""
        QMessageBox.critical(self, "检测错误", f"检测失败:\n{error_msg}")
        
        # 恢复按钮状态
        self.detect_btn.setEnabled(True)
        self.detect_btn.setText("🚀 开始检测")
        self.progress_bar.setVisible(False)
        self.statusBar.showMessage("检测失败")
        
    def update_result_text(self, result):
        """更新结果显示文本"""
        if len(result) == 0:
            self.result_text.setText("未检测到目标")
            return
            
        text = f"检测到 {len(result)} 个目标:\n\n"
        for i, r in enumerate(result, 1):
            if len(r) >= 6:
                cls_id = int(r[5])
                conf = r[4]
                if cls_id < len(self.class_names):
                    text += f"{i}. {self.class_names[cls_id]} (置信度: {conf:.2f})\n"
                    
        self.result_text.setText(text)
        
    def save_result(self):
        """保存检测结果"""
        if self.result_image is None:
            QMessageBox.warning(self, "警告", "请先完成检测！")
            return
            
        file_path, _ = QFileDialog.getSaveFileName(
            self, "保存结果", "yolov8_result.jpg", "JPEG Files (*.jpg); *.png Files (*.png)"
        )
        
        if file_path:
            try:
                cv2.imwrite(file_path, self.result_image)
                QMessageBox.information(self, "成功", f"结果已保存到: {file_path}")
                self.statusBar.showMessage(f"已保存: {file_path}")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"保存失败: {str(e)}")
                
    def resize_image_for_display(self, image):
        """调整图像大小以适应显示"""
        max_height = 380
        h, w = image.shape[:2]
        
        if h > max_height:
            scale = max_height / h
            new_w = int(w * scale)
            new_h = max_height
            image = cv2.resize(image, (new_w, new_h))
            
        return image
        
    def cv2_to_qimage(self, cv_image):
        """将 OpenCV 图像转换为 QImage"""
        h, w, ch = cv_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(cv_image.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        return qt_image
        
    def closeEvent(self, event):
        """关闭事件"""
        try:
            base.mx_deinit()
            print("mxVision 已清理")
        except:
            pass
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = YOLOv8GUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()





