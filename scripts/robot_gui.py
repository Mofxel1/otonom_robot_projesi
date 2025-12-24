#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import yaml
import cv2
import numpy as np
import rospy
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QPushButton, 
                             QVBoxLayout, QHBoxLayout, QWidget, QMessageBox, 
                             QLineEdit, QFormLayout, QFrame)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer

# --- MODÜLLER ---
from smart_navigator import SmartNavigator
from map_processor import MapProcessor

class NavigationThread(QThread):
    finished = pyqtSignal(bool)
    def __init__(self, navigator, x, y):
        super().__init__()
        self.navigator = navigator
        self.x = x
        self.y = y
    def run(self):
        success = self.navigator.go_to(self.x, self.y)
        self.finished.emit(success)

class RobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        rospy.init_node('robot_gui_node', anonymous=True)
        
        self.setWindowTitle("Otonom Robot Kontrol Paneli")
        self.setGeometry(100, 100, 1200, 800)
        
        self.map_folder = os.path.expanduser("~/catkin_ws/src/otonom_robot/maps")
        self.yaml_path = os.path.join(self.map_folder, "otonom_harita.yaml")
        self.pgm_path = os.path.join(self.map_folder, "otonom_harita.pgm")
        
        self.resolution = 0.05
        self.origin = [0.0, 0.0, 0.0]
        self.map_image = None        
        self.scale_factor = 1.0
        
        self.mode = "NAV" 
        self.zone_points = []
        
        self.navigator = SmartNavigator()
        self.processor = MapProcessor(self.map_folder)
        
        self.map_pub = rospy.Publisher('/map_navigation', OccupancyGrid, queue_size=1, latch=False)
        
        self.initUI()
        self.load_and_publish_map()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_map_to_ros)
        self.timer.start(2000) 

    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # --- SOL PANEL ---
        left_widget = QWidget()
        left_widget.setFixedWidth(280)
        control_panel = QVBoxLayout()
        left_widget.setLayout(control_panel)
        
        self.lbl_status = QLabel("Durum: Hazır")
        self.lbl_status.setStyleSheet("font-weight: bold; color: blue; font-size: 14px;")
        control_panel.addWidget(self.lbl_status)
        
        # Ayırıcı
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        control_panel.addWidget(line)

        # MANUEL KOORDİNAT GİRİŞİ
        form_layout = QFormLayout()
        self.txt_x = QLineEdit("0.0")
        self.txt_y = QLineEdit("0.0")
        self.txt_th = QLineEdit("0.0")
        
        form_layout.addRow("X (Metre):", self.txt_x)
        form_layout.addRow("Y (Metre):", self.txt_y)
        form_layout.addRow("Açı (Derece):", self.txt_th)
        control_panel.addLayout(form_layout)
        
        btn_go_manual = QPushButton("GİRİLEN NOKTAYA GİT")
        btn_go_manual.setStyleSheet("background-color: #2196F3; color: white;")
        btn_go_manual.clicked.connect(self.go_to_manual_point)
        control_panel.addWidget(btn_go_manual)

        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        control_panel.addWidget(line2)
        
        btn_nav = QPushButton("📍 MOD: TIKLA & GİT")
        btn_nav.clicked.connect(lambda: self.set_mode("NAV"))
        control_panel.addWidget(btn_nav)
        
        btn_forbid = QPushButton("🚫 MOD: YASAKLI BÖLGE")
        btn_forbid.clicked.connect(lambda: self.set_mode("FORBID"))
        control_panel.addWidget(btn_forbid)
        
        control_panel.addStretch()
        self.lbl_info = QLabel("Mod: NAVİGASYON")
        control_panel.addWidget(self.lbl_info)
        
        main_layout.addWidget(left_widget)
        
        # --- SAĞ PANEL (HARİTA) ---
        self.map_label = QLabel("Harita Bekleniyor...")
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setStyleSheet("border: 2px solid gray; background: #202020;")
        self.map_label.mousePressEvent = self.map_clicked 
        main_layout.addWidget(self.map_label, 1)

    def load_and_publish_map(self):
        if not os.path.exists(self.yaml_path) or not os.path.exists(self.pgm_path):
            self.lbl_status.setText("Hata: Harita dosyaları bulunamadı!")
            return

        with open(self.yaml_path, 'r') as file:
            data = yaml.safe_load(file)
            self.resolution = data['resolution']
            self.origin = data['origin'] 

        self.map_image = cv2.imread(self.pgm_path)
        self.update_map_display()
        self.publish_map_to_ros()
        self.lbl_status.setText("Harita Yüklendi.")

    def update_map_display(self):
        if self.map_image is None: return
        display_img = self.map_image.copy()
        
        # --- GRID (IZGARA) ÇİZİMİ ---
        pixels_per_meter = int(1.0 / self.resolution)
        h, w, _ = display_img.shape
        overlay = display_img.copy()
        
        # Dikey çizgiler
        for x in range(0, w, pixels_per_meter):
            cv2.line(overlay, (x, 0), (x, h), (0, 255, 0), 1)
        # Yatay çizgiler
        for y in range(0, h, pixels_per_meter):
            cv2.line(overlay, (0, y), (w, y), (0, 255, 0), 1)
            
        cv2.addWeighted(overlay, 0.3, display_img, 0.7, 0, display_img)

        # Bölge Noktaları
        for pt in self.zone_points:
            cv2.circle(display_img, pt, 4, (0, 0, 255), -1)
        if len(self.zone_points) > 1:
            pts = np.array(self.zone_points, np.int32)
            cv2.polylines(display_img, [pts], False, (0, 0, 255), 2)

        height, width, channel = display_img.shape
        bytesPerLine = 3 * width
        qImg = QImage(display_img.data, width, height, bytesPerLine, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(qImg)
        self.map_label.setPixmap(pixmap.scaled(self.map_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        
        if self.map_label.pixmap():
            self.scale_factor = width / self.map_label.pixmap().width()

    def publish_map_to_ros(self):
        if self.map_image is None or rospy.is_shutdown(): return
        gray_img = cv2.cvtColor(self.map_image, cv2.COLOR_BGR2GRAY)
        gray_img = cv2.flip(gray_img, 0)
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        h, w = gray_img.shape
        msg.info.resolution = self.resolution
        msg.info.width = w
        msg.info.height = h
        msg.info.origin.position.x = self.origin[0]
        msg.info.origin.position.y = self.origin[1]
        msg.info.origin.orientation.w = 1.0
        
        # Basit sıkıştırma (0, 100, -1)
        flat_data = gray_img.flatten()
        occupancy_data = [100 if px == 0 else 0 for px in flat_data]
        msg.data = occupancy_data
        self.map_pub.publish(msg)

    def set_mode(self, mode):
        self.mode = mode
        self.zone_points = []
        self.update_map_display()
        if mode == "NAV": self.lbl_info.setText("Mod: TIKLA & GİT")
        elif mode == "FORBID": self.lbl_info.setText("Mod: YASAKLI BÖLGE")

    def map_clicked(self, event):
        if self.map_image is None: return
        pixmap = self.map_label.pixmap()
        if not pixmap: return
        
        x_off = (self.map_label.width() - pixmap.width()) / 2
        y_off = (self.map_label.height() - pixmap.height()) / 2
        click_x = event.pos().x() - x_off
        click_y = event.pos().y() - y_off
        
        if 0 <= click_x < pixmap.width() and 0 <= click_y < pixmap.height():
            real_x = int(click_x * self.scale_factor)
            real_y = int(click_y * self.scale_factor)
            
            # Koordinatları Hesapla ve Kutucuklara Yaz
            h = self.map_image.shape[0]
            wx = self.origin[0] + (real_x * self.resolution)
            wy = self.origin[1] + ((h - real_y) * self.resolution)
            
            self.txt_x.setText(f"{wx:.2f}")
            self.txt_y.setText(f"{wy:.2f}")
            
            if self.mode == "NAV":
                self.start_navigation(wx, wy)
            elif self.mode == "FORBID":
                self.zone_points.append((real_x, real_y))
                self.update_map_display()
                if len(self.zone_points) == 4:
                    self.finish_zone()

    def go_to_manual_point(self):
        try:
            x = float(self.txt_x.text())
            y = float(self.txt_y.text())
            self.start_navigation(x, y)
        except ValueError:
            QMessageBox.warning(self, "Hata", "Lütfen geçerli bir sayı girin!")

    def start_navigation(self, wx, wy):
        self.lbl_status.setText(f"Gidiliyor: {wx:.2f}, {wy:.2f}")
        self.nav_thread = NavigationThread(self.navigator, wx, wy)
        self.nav_thread.finished.connect(self.nav_finished)
        self.nav_thread.start()

    def nav_finished(self, success):
        self.lbl_status.setText("VARIŞ BAŞARILI! ✅" if success else "BAŞARISIZ! ❌")

    def finish_zone(self):
        world_points = []
        h = self.map_image.shape[0]
        for (px, py) in self.zone_points:
            wx = self.origin[0] + (px * self.resolution)
            wy = self.origin[1] + ((h - py) * self.resolution)
            world_points.append((wx, wy))

        reply = QMessageBox.question(self, 'Onay', 'Yasaklı Bölge Kaydedilsin mi?', QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.processor.load_map()
            self.processor.add_forbidden_zone(world_points)
            self.processor.save_map()
            self.load_and_publish_map()
            # Move base costmap temizle
            try:
                rospy.wait_for_service('/move_base/clear_costmaps', timeout=0.5)
                reset = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
                reset()
            except: pass
            self.lbl_status.setText("Yasaklı Bölge Eklendi.")
        
        self.zone_points = []
        self.update_map_display()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotGUI()
    window.show()
    sys.exit(app.exec_())
