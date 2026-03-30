#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
import os
import yaml

# --- 1. HARİTA KAYDEDİCİ ---
class MapSaver:
    def __init__(self):
        pass

    def save_now(self):
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

# --- 2. BÖLGE (ZONE) YÖNETİCİSİ ---
class MapProcessor:
    def __init__(self, map_folder):
        self.map_folder = map_folder
        self.zones_file = os.path.join(map_folder, "zones.yaml")
        self.forbidden_zones = []
        self.speed_zones = []
        self.load_zones()

    def load_zones(self):
        """Kaydedilmiş yasaklı ve hız bölgelerini yükler"""
        if os.path.exists(self.zones_file):
            try:
                with open(self.zones_file, 'r') as f:
                    data = yaml.safe_load(f) or {}
                    self.forbidden_zones = data.get('forbidden', [])
                    self.speed_zones = data.get('speed', [])
            except Exception as e:
                rospy.logerr(f"Zone dosyası okunamadı: {e}")

    def save_zones(self):
        """Bölgeleri YAML dosyasına kaydeder"""
        data = {
            'forbidden': self.forbidden_zones,
            'speed': self.speed_zones
        }
        with open(self.zones_file, 'w') as f:
            yaml.dump(data, f)

    def add_forbidden_zone(self, points):
        """Gerçek dünya (world) koordinatlarını ekler"""
        self.forbidden_zones.append(points)
        self.save_zones()

    def add_speed_zone(self, points):
        self.speed_zones.append(points)
        self.save_zones()
        
    def clear_all_zones(self):
        self.forbidden_zones = []
        self.speed_zones = []
        self.save_zones()
