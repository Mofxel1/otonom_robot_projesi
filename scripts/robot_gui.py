#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import yaml
import math
import cv2
import numpy as np
import rospy
import tf2_ros
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

        self.setWindowTitle("Otonom Robot Kontrol Paneli")
        self.setGeometry(100, 100, 1100, 700)

        self.map_folder = os.path.expanduser("~/catkin_ws/src/otonom_robot/maps")
        self.yaml_path  = os.path.join(self.map_folder, "otonom_harita.yaml")
        self.pgm_path   = os.path.join(self.map_folder, "otonom_harita.pgm")

        self.resolution = 0.05
        self.origin     = [0.0, 0.0, 0.0]
        self.map_image  = None
        self.scale_factor = 1.0

        self.mode                = "NAV"
        self.current_draw_points = []

        # Robot konumu ve yönü (tf'ten)
        self.robot_x   = None
        self.robot_y   = None
        self.robot_yaw = None

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.navigator = SmartNavigator()
        self.processor = MapProcessor(self.map_folder)

        self.map_pub = rospy.Publisher('/gui_zones', OccupancyGrid, queue_size=1, latch=True)

        self.initUI()
        self.load_map_data()

        self.pub_timer = QTimer()
        self.pub_timer.timeout.connect(self.publish_map_to_ros)
        self.pub_timer.start(1000)

        self.robot_timer = QTimer()
        self.robot_timer.timeout.connect(self._update_robot_pose)
        self.robot_timer.start(200)

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _update_robot_pose(self):
        """TF'ten robot konumunu ve yönünü al, display'i güncelle"""
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rospy.Time(0), rospy.Duration(0.1))
            self.robot_x = trans.transform.translation.x
            self.robot_y = trans.transform.translation.y
            # Quaternion → yaw
            q = trans.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny, cosy)
            self.update_map_display()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Koordinat dönüşümleri
    # ------------------------------------------------------------------

    def _res(self):
        return self.resolution

    def _origin(self):
        return (self.origin[0], self.origin[1])

    def _map_hw(self):
        if self.map_image is not None:
            return self.map_image.shape[:2]
        return 100, 100

    def display2world(self, disp_x, disp_y):
        """Display piksel → dünya (metre) — flip YOK versiyonu"""
        H, W   = self._map_hw()
        res    = self._res()
        ox, oy = self._origin()
        wx = ox + disp_x * res
        wy = oy + (H - 1 - disp_y) * res
        return wx, wy

    def world2display(self, wx, wy):
        """Dünya (metre) → display piksel — flip YOK versiyonu"""
        H, W   = self._map_hw()
        res    = self._res()
        ox, oy = self._origin()
        disp_x = int((wx - ox) / res)
        disp_y = int(H - 1 - (wy - oy) / res)
        return disp_x, disp_y

    def world2ros_pixel(self, wx, wy):
        """Dünya (metre) → ROS OccupancyGrid piksel (row0=ALT)"""
        res    = self._res()
        ox, oy = self._origin()
        ros_px = int((wx - ox) / res)
        ros_py = int((wy - oy) / res)
        return ros_px, ros_py

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

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

        line = QFrame(); line.setFrameShape(QFrame.HLine)
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

        line2 = QFrame(); line2.setFrameShape(QFrame.HLine)
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

        btn_clear = QPushButton("🗑️ BÖLGELERİ TEMİZLE")
        btn_clear.setStyleSheet("padding: 8px; color: red;")
        btn_clear.clicked.connect(self.clear_zones)
        control_panel.addWidget(btn_clear)

        control_panel.addStretch()

        # Robot konum bilgisi etiketi
        self.lbl_robot_pos = QLabel("Robot: —")
        self.lbl_robot_pos.setStyleSheet("color: darkgreen; font-size: 11px;")
        control_panel.addWidget(self.lbl_robot_pos)

        self.lbl_info = QLabel("Mod: TIKLA & GİT")
        self.lbl_info.setStyleSheet("font-weight: bold; color: gray;")
        control_panel.addWidget(self.lbl_info)

        main_layout.addWidget(left_widget)

        self.map_label = QLabel("Harita Yükleniyor...")
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setStyleSheet("border: 2px solid gray; background: #eee;")
        self.map_label.setScaledContents(False)
        self.map_label.setFixedSize(800, 580)
        self.map_label.mousePressEvent = self.map_clicked
        main_layout.addWidget(self.map_label)

    # ------------------------------------------------------------------
    # Harita yükleme
    # ------------------------------------------------------------------

    def load_map_data(self):
        if not os.path.exists(self.yaml_path) or not os.path.exists(self.pgm_path):
            self.lbl_status.setText("Hata: Harita yok!")
            return

        with open(self.yaml_path, 'r') as f:
            data = yaml.safe_load(f)
            self.resolution = data['resolution']
            self.origin     = data['origin']

        self.map_image = cv2.imread(self.pgm_path)
        self.update_map_display()
        self.publish_map_to_ros()
        self.lbl_status.setText("Harita Yüklendi.")

    # ------------------------------------------------------------------
    # Görüntü güncelleme
    # ------------------------------------------------------------------

    def update_map_display(self):
        if self.map_image is None:
            return

        # Flip YOK — orijinal pgm yönü kullanılır
        display_img = self.map_image.copy()

        # Çizilmekte olan noktalar (Mavi)
        for pt in self.current_draw_points:
            cv2.circle(display_img, pt, 4, (255, 0, 0), -1)

        # Yasaklı bölgeler (Kırmızı yarı saydam)
        for zone in self.processor.forbidden_zones:
            pts = np.array([self.world2display(x, y) for (x, y) in zone], np.int32)
            overlay = display_img.copy()
            cv2.fillPoly(overlay, [pts], (80, 80, 255))
            cv2.addWeighted(overlay, 0.4, display_img, 0.6, 0, display_img)
            cv2.polylines(display_img, [pts], True, (0, 0, 255), 2)

        # Hız bölgeleri (Turuncu yarı saydam)
        for zone in self.processor.speed_zones:
            pts = np.array([self.world2display(x, y) for (x, y) in zone], np.int32)
            overlay = display_img.copy()
            cv2.fillPoly(overlay, [pts], (80, 200, 255))
            cv2.addWeighted(overlay, 0.4, display_img, 0.6, 0, display_img)
            cv2.polylines(display_img, [pts], True, (0, 165, 255), 2)

        # Robot gösterimi
        if self.robot_x is not None and self.robot_y is not None:
            H, W = self._map_hw()
            rx, ry = self.world2display(self.robot_x, self.robot_y)

            if 0 <= rx < W and 0 <= ry < H:
                # Dış halka (siyah)
                cv2.circle(display_img, (rx, ry), 9, (0, 0, 0), -1)
                # İç daire (yeşil)
                cv2.circle(display_img, (rx, ry), 7, (0, 220, 0), -1)

                # Yön oku
                if self.robot_yaw is not None:
                    arrow_len = 15
                    # display'de Y ekseni ters (büyük y = aşağı), yaw da ters
                    ex = int(rx + arrow_len * math.cos(self.robot_yaw))
                    ey = int(ry - arrow_len * math.sin(self.robot_yaw))
                    cv2.arrowedLine(display_img, (rx, ry), (ex, ey),
                                    (0, 0, 180), 2, tipLength=0.4)

            # Sol panelde konum bilgisi
            yaw_deg = math.degrees(self.robot_yaw) if self.robot_yaw is not None else 0
            self.lbl_robot_pos.setText(
                f"Robot: ({self.robot_x:.2f}, {self.robot_y:.2f})\n"
                f"Yön: {yaw_deg:.1f}°")

        H, W  = display_img.shape[:2]
        qImg  = QImage(display_img.data, W, H, 3 * W, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(qImg)
        # Sabit boyut: map_label fixed 800x580, her seferinde aynı scale
        DISPLAY_W = 800
        DISPLAY_H = 580
        scaled = pixmap.scaled(DISPLAY_W, DISPLAY_H, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.map_label.setPixmap(scaled)
        # scale_factor sadece ilk hesaplamada set edilir
        if self.scale_factor == 1.0 and scaled.width() > 0:
            self.scale_factor = W / scaled.width()

    # ------------------------------------------------------------------
    # ROS publish
    # ------------------------------------------------------------------

    def publish_map_to_ros(self):
        if rospy.is_shutdown() or self.map_image is None:
            return

        H, W   = self.map_image.shape[:2]
        res    = self.resolution
        ox     = self.origin[0]
        oy     = self.origin[1]

        # pgm: 0=siyah(engel), 255=beyaz(serbest)
        # ROS: 100=engel, 0=serbest, -1=bilinmeyen
        gray      = cv2.cvtColor(self.map_image, cv2.COLOR_BGR2GRAY)
        base_data = np.where(gray < 50, 100,
                    np.where(gray > 200, 0, -1)).astype(np.int8)
        # OpenCV row0=üst, ROS row0=alt → flip
        base_data = np.flipud(base_data)

        zone_mask = np.zeros((H, W), dtype=np.uint8)

        for zone in self.processor.speed_zones:
            pts = np.array([self.world2ros_pixel(x, y) for (x, y) in zone], np.int32)
            cv2.fillPoly(zone_mask, [pts], 50)

        for zone in self.processor.forbidden_zones:
            pts = np.array([self.world2ros_pixel(x, y) for (x, y) in zone], np.int32)
            cv2.fillPoly(zone_mask, [pts], 100)

        rospy.loginfo_throttle(5, f"[GUI] Yasaklı piksel: {np.sum(zone_mask == 100)}, "
                                  f"Hız piksel: {np.sum(zone_mask == 50)}")

        merged = base_data.copy()
        merged[zone_mask == 50]  = 50
        merged[zone_mask == 100] = 100

        msg = OccupancyGrid()
        msg.header.stamp              = rospy.Time.now()
        msg.header.frame_id           = "map"
        msg.info.resolution           = res
        msg.info.width                = W
        msg.info.height               = H
        msg.info.origin.position.x    = ox
        msg.info.origin.position.y    = oy
        msg.info.origin.orientation.w = 1.0
        msg.data = merged.flatten().tolist()
        self.map_pub.publish(msg)

    # ------------------------------------------------------------------
    # Mod ve tıklama
    # ------------------------------------------------------------------

    def set_mode(self, mode):
        self.mode = mode
        self.current_draw_points = []
        self.update_map_display()
        labels = {"NAV":    "Mod: TIKLA & GİT",
                  "FORBID": "Mod: YASAKLI BÖLGE ÇİZİLİYOR...",
                  "SPEED":  "Mod: HIZ BÖLGESİ ÇİZİLİYOR..."}
        self.lbl_info.setText(labels.get(mode, ""))

    def map_clicked(self, event):
        if self.map_image is None:
            return
        pixmap = self.map_label.pixmap()
        if not pixmap:
            return

        x_off = (self.map_label.width()  - pixmap.width())  / 2.0
        y_off = (self.map_label.height() - pixmap.height()) / 2.0

        click_x = event.pos().x() - x_off
        click_y = event.pos().y() - y_off

        if not (0 <= click_x < pixmap.width() and 0 <= click_y < pixmap.height()):
            return

        disp_x = int(click_x * self.scale_factor)
        disp_y = int(click_y * self.scale_factor)
        wx, wy = self.display2world(disp_x, disp_y)

        self.txt_x.setText(f"{wx:.2f}")
        self.txt_y.setText(f"{wy:.2f}")

        if self.mode == "NAV":
            self.start_navigation(wx, wy)
        elif self.mode in ["FORBID", "SPEED"]:
            self.current_draw_points.append((disp_x, disp_y))
            self.update_map_display()
            if len(self.current_draw_points) == 4:
                self.finish_zone()

    # ------------------------------------------------------------------
    # Navigasyon
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Zone yönetimi
    # ------------------------------------------------------------------

    def finish_zone(self):
        world_points = []
        for (disp_x, disp_y) in self.current_draw_points:
            wx, wy = self.display2world(disp_x, disp_y)
            world_points.append([float(wx), float(wy)])

        rospy.loginfo(f"[GUI] Zone noktaları: {world_points}")

        if self.mode == "FORBID":
            self.processor.add_forbidden_zone(world_points)
            self.lbl_status.setText("Yasaklı Bölge Kaydedildi!")
            self.trigger_costmap_update()
        elif self.mode == "SPEED":
            self.processor.add_speed_zone(world_points)
            self.navigator.add_speed_zone(world_points)
            self.lbl_status.setText("Hız Bölgesi Kaydedildi!")
            self.trigger_costmap_update()

        self.current_draw_points = []
        self.update_map_display()
        self.publish_map_to_ros()

    def clear_zones(self):
        reply = QMessageBox.question(self, 'Onay', 'Tüm bölgeler silinsin mi?',
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.processor.clear_all_zones()
            self.update_map_display()
            self.publish_map_to_ros()
            self.trigger_costmap_update()
            self.lbl_status.setText("Bölgeler temizlendi.")

    def trigger_costmap_update(self):
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=0.5)
            rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
        except:
            pass


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotGUI()
    window.show()
    sys.exit(app.exec_())
