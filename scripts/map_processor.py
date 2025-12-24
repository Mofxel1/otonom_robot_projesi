#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
import os
import cv2
import yaml
import numpy as np

# --- 1. HARİTA KAYDEDİCİ (Mission Control Kullanır) ---
class MapSaver:
    def __init__(self):
        pass

    def save_now(self):
        """
        Haritayı ~/catkin_ws/src/otonom_robot/maps klasörüne kaydeder.
        """
        rospy.loginfo("--- HARİTA KAYIT İŞLEMİ (MapSaver) ---")
        
        folder_path = os.path.expanduser("~/catkin_ws/src/otonom_robot/maps")
        if not os.path.exists(folder_path):
            try:
                os.makedirs(folder_path)
            except OSError as e:
                rospy.logerr(f"Klasör hatası: {e}")
                return None

        full_path = os.path.join(folder_path, "otonom_harita")
        map_saver_cmd = "/opt/ros/noetic/lib/map_server/map_saver"
        
        try:
            subprocess.call([map_saver_cmd, "-f", full_path])
            rospy.loginfo(f"Harita kaydedildi: {full_path}.pgm")
            return folder_path
        except Exception as e:
            rospy.logerr(f"Kaydetme hatası: {e}")
            return None

# --- 2. HARİTA DÜZENLEYİCİ (GUI Kullanır) ---
class MapProcessor:
    def __init__(self, map_folder):
        self.map_folder = map_folder
        self.yaml_path = os.path.join(map_folder, "otonom_harita.yaml")
        self.pgm_path = os.path.join(map_folder, "otonom_harita.pgm")
        self.map_img = None
        self.resolution = 0.05
        self.origin = [0.0, 0.0, 0.0]

    def load_map(self):
        if not os.path.exists(self.yaml_path) or not os.path.exists(self.pgm_path):
            return False

        try:
            with open(self.yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                self.resolution = data['resolution']
                self.origin = data['origin']

            self.map_img = cv2.imread(self.pgm_path, cv2.IMREAD_GRAYSCALE)
            return True
        except Exception:
            return False

    def world_to_pixel(self, wx, wy):
        if self.map_img is None: return (0, 0)
        origin_x = self.origin[0]
        origin_y = self.origin[1]
        height = self.map_img.shape[0]
        px = int((wx - origin_x) / self.resolution)
        py = int(height - ((wy - origin_y) / self.resolution))
        return (px, py)

    def add_forbidden_zone(self, points):
        if self.map_img is None: self.load_map()
        if self.map_img is None: return

        pixel_points = []
        for p in points:
            px, py = self.world_to_pixel(p[0], p[1])
            pixel_points.append((px, py))
        
        pts = np.array(pixel_points, np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.fillPoly(self.map_img, [pts], 0) # Siyaha boya

    def save_map(self):
        if self.map_img is not None:
            cv2.imwrite(self.pgm_path, self.map_img)
