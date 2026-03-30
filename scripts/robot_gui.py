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
        
        self.setWindowTitle("Otonom Robot Kontrol Paneli - Profesyonel Sürüm")
        self.setGeometry(100, 100, 1100, 700) 
        
        self.map_folder = os.path.expanduser("~/catkin_ws/src/otonom_robot/maps")
        self.yaml_path = os.path.join(self.map_folder, "otonom_harita.yaml")
        self.pgm_path = os.path.join(self.map_folder, "otonom_harita.pgm")
        
        self.resolution = 0.05
        self.origin = [0.0, 0.0, 0.0]
        self.map_image = None        
        self.scale_factor = 1.0
        
        self.mode = "NAV" 
        self.current_draw_points = []
        self.op_boundaries = None 
        
        self.navigator = SmartNavigator()
        self.processor = MapProcessor(self.map_folder)
        
        # YENİ TOPIC: gui_zones
        self.map_pub = rospy.Publisher('/gui_zones', OccupancyGrid, queue_size=1, latch=False)
        
        self.initUI()
        self.load_map_data()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_map_to_ros)
        self.timer.start(2000) 

    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        left_widget = QWidget()
        left_widget.setFixedWidth(250) 
        control_panel = QVBoxLayout()
        left_widget.setLayout(control_panel)
        
        self.lbl_status = QLabel("Durum: Hazır")
        self.lbl_status.setStyleSheet("font-weight: bold; color: blue; font-size: 14px;")
        self.lbl_status.setWordWrap(True)
        control_panel.addWidget(self.lbl_status)
        
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        control_panel.addWidget(line)

        coord_layout = QFormLayout()
        self.txt_x = QLineEdit("0.0")
        self.txt_y = QLineEdit("0.0")
        coord_layout.addRow("X (Metre):", self.txt_x)
        coord_layout.addRow("Y (Metre):", self.txt_y)
        control_panel.addLayout(coord_layout)
        
        btn_go_manual = QPushButton("GİRİLEN NOKTAYA GİT")
        btn_go_manual.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
        btn_go_manual.clicked.connect(self.go_to_manual_point)
        control_panel.addWidget(btn_go_manual)

        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        control_panel.addWidget(line2)

        btn_nav = QPushButton("📍 MOD: TIKLA & GİT")
        btn_nav.setStyleSheet("padding: 8px;")
        btn_nav.clicked.connect(lambda: self.set_mode("NAV"))
        control_panel.addWidget(btn_nav)
        
        btn_forbid = QPushButton("🚫 MOD: YASAKLI BÖLGE")
        btn_forbid.setStyleSheet("padding: 8px; background-color: #ffcccc;")
        btn_forbid.clicked.connect(lambda: self.set_mode("FORBID"))
        control_panel.addWidget(btn_forbid)
        
        btn_speed = QPushButton("⚠️ MOD: HIZ BÖLGESİ")
        btn_speed.setStyleSheet("padding: 8px; background-color: #ffffcc;")
        btn_speed.clicked.connect(lambda: self.set_mode("SPEED"))
        control_panel.addWidget(btn_speed)
        
        # Temizle Butonu Eklendi
        btn_clear = QPushButton("🗑️ BÖLGELERİ TEMİZLE")
        btn_clear.setStyleSheet("padding: 8px; color: red;")
        btn_clear.clicked.connect(self.clear_zones)
        control_panel.addWidget(btn_clear)
        
        control_panel.addStretch()
        self.lbl_info = QLabel("Mod: TIKLA & GİT")
        self.lbl_info.setStyleSheet("font-weight: bold; color: gray;")
        control_panel.addWidget(self.lbl_info)
        
        main_layout.addWidget(left_widget)
        
        self.map_label = QLabel("Harita Yükleniyor...")
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setStyleSheet("border: 2px solid gray; background: #eee;")
        self.map_label.mousePressEvent = self.map_clicked 
        main_layout.addWidget(self.map_label)

    def load_map_data(self):
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

    def w2p(self, wx, wy):
        """World (Metre) koordinatını Pixel'e çevirir"""
        if self.map_image is None: return 0, 0
        px = int((wx - self.origin[0]) / self.resolution)
        py = int(self.map_image.shape[0] - ((wy - self.origin[1]) / self.resolution))
        return px, py

    def update_map_display(self):
        if self.map_image is None: return
        
        # RViz yönüne çevir
        display_img = cv2.flip(self.map_image.copy(), 0)
        
        # Çizilmekte olan noktalar (Mavi)
        for pt in self.current_draw_points:
            cv2.circle(display_img, pt, 4, (255, 0, 0), -1)
            
        # Kayıtlı Yasaklı Bölgeler (Kırmızı - İçi dolu saydam veya sadece kalın çizgi)
        for zone in self.processor.forbidden_zones:
            pts = np.array([self.w2p(x, y) for (x, y) in zone], np.int32)
            cv2.polylines(display_img, [pts], True, (0, 0, 255), 3)

        # Kayıtlı Hız Bölgeleri (Sarı/Turuncu)
        for zone in self.processor.speed_zones:
            pts = np.array([self.w2p(x, y) for (x, y) in zone], np.int32)
            cv2.polylines(display_img, [pts], True, (0, 165, 255), 3)

        height, width = display_img.shape[:2]
        bytesPerLine = 3 * width
        qImg = QImage(display_img.data, width, height, bytesPerLine, QImage.Format_RGB888).rgbSwapped()
        
        pixmap = QPixmap.fromImage(qImg)
        self.map_label.setPixmap(pixmap.scaled(self.map_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        
        if self.map_label.pixmap():
            self.scale_factor = width / self.map_label.pixmap().width()

    def publish_map_to_ros(self):
        """GUI üzerinden ROS'a Çok Renkli (Layered) Maske Gönderir"""
        if self.map_image is None or rospy.is_shutdown(): return

        h, w = self.map_image.shape[:2]
        # Bembeyaz (Maliyetsiz 0) bir şeffaf harita oluştur
        mask_layer = np.zeros((h, w), dtype=np.uint8)

        # Hız Bölgelerini Griye (50) Boya
        for zone in self.processor.speed_zones:
            pts = np.array([self.w2p(x, y) for (x, y) in zone], np.int32)
            cv2.fillPoly(mask_layer, [pts], 50)

        # Yasaklı Bölgeleri Siyaha (100) Boya (Üstüne yazarak ezer)
        for zone in self.processor.forbidden_zones:
            pts = np.array([self.w2p(x, y) for (x, y) in zone], np.int32)
            cv2.fillPoly(mask_layer, [pts], 100)

        # RViz eksenine uydurmak için resmi ters çevir
        mask_layer = cv2.flip(mask_layer, 0)

        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.info.resolution = self.resolution
        msg.info.width = w
        msg.info.height = h
        msg.info.origin.position.x = self.origin[0]
        msg.info.origin.position.y = self.origin[1]
        msg.info.origin.orientation.w = 1.0
        
        msg.data = mask_layer.flatten().tolist()
        self.map_pub.publish(msg)

    def set_mode(self, mode):
        self.mode = mode
        self.current_draw_points = []
        self.update_map_display()
        
        if mode == "NAV": self.lbl_info.setText("Mod: TIKLA & GİT")
        elif mode == "FORBID": self.lbl_info.setText("Mod: YASAKLI BÖLGE ÇİZİLİYOR...")
        elif mode == "SPEED": self.lbl_info.setText("Mod: HIZ BÖLGESİ ÇİZİLİYOR...")

    def map_clicked(self, event):
        if self.map_image is None: return
        pixmap = self.map_label.pixmap()
        if not pixmap: return
        
        x_off = (self.map_label.width() - pixmap.width()) / 2.0
        y_off = (self.map_label.height() - pixmap.height()) / 2.0
        
        click_x = event.pos().x() - x_off
        click_y = event.pos().y() - y_off
        
        if 0 <= click_x < pixmap.width() and 0 <= click_y < pixmap.height():
            real_x = click_x * self.scale_factor
            real_y = click_y * self.scale_factor
            
            h = self.map_image.shape[0]
            wx = self.origin[0] + (real_x * self.resolution)
            wy = self.origin[1] + ((h - real_y) * self.resolution)
            
            self.txt_x.setText(f"{wx:.2f}")
            self.txt_y.setText(f"{wy:.2f}")

            if self.mode == "NAV":
                self.start_navigation(wx, wy)
                
            elif self.mode in ["FORBID", "SPEED"]:
                self.current_draw_points.append((int(real_x), int(real_y))) 
                self.update_map_display()
                if len(self.current_draw_points) == 4:
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
        for (px, py) in self.current_draw_points:
            wx = self.origin[0] + (px * self.resolution)
            wy = self.origin[1] + ((h - py) * self.resolution)
            world_points.append((wx, wy))

        if self.mode == "FORBID":
            self.processor.add_forbidden_zone(world_points)
            self.lbl_status.setText("Yasaklı Bölge Kaydedildi!")
            self.trigger_costmap_update()

        elif self.mode == "SPEED":
            self.processor.add_speed_zone(world_points)
            # Eğer SmartNavigator içinde anlık okuyacak bir sistemin varsa onu da besleyebilirsin:
            self.navigator.add_speed_zone(world_points) 
            self.lbl_status.setText("Hız Bölgesi Kaydedildi!")
            self.trigger_costmap_update()

        self.current_draw_points = []
        self.update_map_display()
        self.publish_map_to_ros() # Anında ROS'a gönder

    def clear_zones(self):
        reply = QMessageBox.question(self, 'Onay', 'Tüm bölgeler silinsin mi?', QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.processor.clear_all_zones()
            self.update_map_display()
            self.publish_map_to_ros()
            self.trigger_costmap_update()
            self.lbl_status.setText("Bölgeler temizlendi.")
            
    def trigger_costmap_update(self):
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=0.5)
            reset = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            reset()
        except: pass

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotGUI()
    window.show()
    sys.exit(app.exec_())
