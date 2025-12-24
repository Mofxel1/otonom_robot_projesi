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
                             QLineEdit, QFormLayout, QFrame) # <-- Yeni eklenenler
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
        self.setGeometry(100, 100, 1100, 700) # Pencereyi biraz genişlettik
        
        self.map_folder = os.path.expanduser("~/catkin_ws/src/otonom_robot/maps")
        self.yaml_path = os.path.join(self.map_folder, "otonom_harita.yaml")
        self.pgm_path = os.path.join(self.map_folder, "otonom_harita.pgm")
        
        self.resolution = 0.05
        self.origin = [0.0, 0.0, 0.0]
        self.map_image = None       
        self.scale_factor = 1.0
        
        self.mode = "NAV" 
        self.zone_points = []
        self.op_boundaries = None 
        
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
        left_widget.setFixedWidth(250) # Paneli biraz genişlettik
        control_panel = QVBoxLayout()
        left_widget.setLayout(control_panel)
        
        # Durum Göstergesi
        self.lbl_status = QLabel("Durum: Hazır")
        self.lbl_status.setStyleSheet("font-weight: bold; color: blue; font-size: 14px;")
        self.lbl_status.setWordWrap(True)
        control_panel.addWidget(self.lbl_status)
        
        # Ayırıcı Çizgi
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        control_panel.addWidget(line)

        # --- YENİ: MANUEL KOORDİNAT GİRİŞİ ---
        coord_layout = QFormLayout()
        
        self.txt_x = QLineEdit("0.0")
        self.txt_y = QLineEdit("0.0")
        self.txt_th = QLineEdit("0.0")
        
        coord_layout.addRow("X (Metre):", self.txt_x)
        coord_layout.addRow("Y (Metre):", self.txt_y)
        # coord_layout.addRow("Açı (Derece):", self.txt_th) # İstersen açabilirsin
        
        control_panel.addLayout(coord_layout)
        
        btn_go_manual = QPushButton("GİRİLEN NOKTAYA GİT")
        btn_go_manual.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
        btn_go_manual.clicked.connect(self.go_to_manual_point)
        control_panel.addWidget(btn_go_manual)
        # -------------------------------------

        # Ayırıcı Çizgi 2
        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        control_panel.addWidget(line2)

        # Mod Butonları
        btn_nav = QPushButton("📍 MOD: TIKLA & GİT")
        btn_nav.setStyleSheet("padding: 8px;")
        btn_nav.clicked.connect(lambda: self.set_mode("NAV"))
        control_panel.addWidget(btn_nav)
        
        btn_forbid = QPushButton("🚫 MOD: YASAKLI BÖLGE")
        btn_forbid.setStyleSheet("padding: 8px;")
        btn_forbid.clicked.connect(lambda: self.set_mode("FORBID"))
        control_panel.addWidget(btn_forbid)
        
        btn_speed = QPushButton("⚠️ MOD: HIZ BÖLGESİ")
        btn_speed.setStyleSheet("padding: 8px;")
        btn_speed.clicked.connect(lambda: self.set_mode("SPEED"))
        control_panel.addWidget(btn_speed)
        
        control_panel.addStretch()
        self.lbl_info = QLabel("Mod: TIKLA & GİT")
        self.lbl_info.setStyleSheet("font-weight: bold; color: gray;")
        control_panel.addWidget(self.lbl_info)
        
        main_layout.addWidget(left_widget)
        
        # --- SAĞ PANEL (Harita) ---
        self.map_label = QLabel("Harita Yükleniyor...")
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setStyleSheet("border: 2px solid gray; background: #eee;")
        self.map_label.mousePressEvent = self.map_clicked 
        main_layout.addWidget(self.map_label)

    def load_and_publish_map(self):
        if not os.path.exists(self.yaml_path) or not os.path.exists(self.pgm_path):
            self.lbl_status.setText("Hata: Harita yok!")
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
        
        # SINIRLARI ÇİZ
        try:
            if rospy.has_param("/op_boundaries"):
                self.op_boundaries = rospy.get_param("/op_boundaries")
                h, w = self.map_image.shape[:2]
                
                px1 = int((self.op_boundaries[0] - self.origin[0]) / self.resolution)
                py1 = int(h - ((self.op_boundaries[1] - self.origin[1]) / self.resolution))
                px2 = int((self.op_boundaries[2] - self.origin[0]) / self.resolution)
                py2 = int(h - ((self.op_boundaries[3] - self.origin[1]) / self.resolution))
                
                cv2.rectangle(display_img, (px1, py1), (px2, py2), (0, 0, 255), 3)
        except Exception: pass
        
        # BÖLGELERİ ÇİZ
        for pt in self.zone_points:
            cv2.circle(display_img, pt, 3, (255, 0, 0), -1)
        if len(self.zone_points) > 1:
            pts = np.array(self.zone_points, np.int32)
            cv2.polylines(display_img, [pts], False, (255, 0, 0), 2)

        height, width = display_img.shape[:2]
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
        
        flat_data = gray_img.flatten()
        occupancy_data = []
        for px in flat_data:
            if px == 0: occupancy_data.append(100)
            elif px >= 250: occupancy_data.append(0)
            else: occupancy_data.append(0)
        
        msg.data = occupancy_data
        self.map_pub.publish(msg)

    def set_mode(self, mode):
        self.mode = mode
        self.zone_points = []
        self.update_map_display()
        
        if mode == "NAV": self.lbl_info.setText("Mod: TIKLA & GİT")
        elif mode == "FORBID": self.lbl_info.setText("Mod: YASAKLI BÖLGE")
        elif mode == "SPEED": self.lbl_info.setText("Mod: HIZ BÖLGESİ")

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
            
            # --- TIKLANAN YERİ KUTUCUKLARA YAZ ---
            h = self.map_image.shape[0]
            wx = self.origin[0] + (real_x * self.resolution)
            wy = self.origin[1] + ((h - real_y) * self.resolution)
            
            self.txt_x.setText(f"{wx:.2f}")
            self.txt_y.setText(f"{wy:.2f}")
            # -------------------------------------

            if self.mode == "NAV":
                self.start_navigation(wx, wy)
                
            elif self.mode in ["FORBID", "SPEED"]:
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
        # SINIR KONTROLÜ
        if self.op_boundaries:
            min_x, min_y, max_x, max_y = self.op_boundaries
            if not (min_x <= wx <= max_x and min_y <= wy <= max_y):
                QMessageBox.warning(self, "Güvenlik Uyarısı", 
                    "Seçilen nokta OPERASYON SINIRLARI DIŞINDA!\nRobot güvenli bölgeden çıkamaz.")
                self.lbl_status.setText("Hata: Sınır dışı hedef!")
                return

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

        if self.mode == "FORBID":
            reply = QMessageBox.question(self, 'Onay', 'Yasaklı Bölge Kaydedilsin mi?', QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.processor.load_map()
                self.processor.add_forbidden_zone(world_points)
                self.processor.save_map()
                self.load_and_publish_map() 
                try:
                    rospy.wait_for_service('/move_base/clear_costmaps', timeout=0.5)
                    reset = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
                    reset()
                except: pass
                self.lbl_status.setText("Yasaklı Bölge Aktif!")

        elif self.mode == "SPEED":
            reply = QMessageBox.question(self, 'Onay', 'Yavaş Bölge Eklensin mi?', QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.navigator.add_speed_zone(world_points)
                self.lbl_status.setText("Hız Bölgesi Aktif (RAM).")

        self.zone_points = []
        self.update_map_display()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotGUI()
    window.show()
    sys.exit(app.exec_())
