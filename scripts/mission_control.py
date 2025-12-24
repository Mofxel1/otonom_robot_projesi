#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import math
import sys
import os
import subprocess
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty 

# --- YENİ ZIGZAG MODÜLÜNÜ ÇAĞIRIYORUZ ---
from zigzag_navigator import ZigzagMission

class MissionControl:
    def __init__(self):
        rospy.init_node('mission_control_node')
        
        self.explorer_pub = rospy.Publisher('/explorer_enabled', Bool, queue_size=1, latch=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.state = 'INITIALIZING'
        self.wall_follow_start_pos = None
        self.exploration_started_flag = False
        
        # Sınırları takip etmek için değişkenler
        self.min_x, self.max_x = sys.float_info.max, -sys.float_info.max
        self.min_y, self.max_y = sys.float_info.max, -sys.float_info.max
        
        rospy.Subscriber("/boundary_explorer_node/state", String, self.explorer_state_callback)
        
        rospy.loginfo("GÖREV KONTROL MERKEZİ BAŞLATILDI.")
        self.run()

    def explorer_state_callback(self, msg):
        # Duvar takibi başladığında konumu kaydet
        if msg.data == 'duvari_takip_et' and self.wall_follow_start_pos is None:
            pos = self.get_robot_pose()
            if pos:
                self.wall_follow_start_pos = pos
                self.update_boundaries(pos.x, pos.y)
                self.exploration_started_flag = True
                rospy.loginfo("Duvar Takibi Başladı, sınırlar kaydediliyor...")

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
            return trans.transform.translation
        except Exception:
            return None

    def update_boundaries(self, x, y):
        self.min_x = min(self.min_x, x)
        self.max_x = max(self.max_x, x)
        self.min_y = min(self.min_y, y)
        self.max_y = max(self.max_y, y)

    def save_map_automatically(self):
        rospy.loginfo("--- HARİTA KAYDEDİLİYOR ---")
        folder_path = os.path.expanduser("~/catkin_ws/src/otonom_robot/maps")
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        full_path = os.path.join(folder_path, "otonom_harita")
        try:
            subprocess.call(["/opt/ros/noetic/lib/map_server/map_saver", "-f", full_path])
            rospy.loginfo(f"Harita şuraya kaydedildi: {full_path}")
        except Exception as e:
            rospy.logerr(f"Harita kaydedilemedi: {e}")

    def refresh_costmaps(self):
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=2.0)
            reset = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            reset()
        except: pass

    def execute_zigzag_mission(self):
        """
        Eski mantıkla, bulunan sınırlardan köşe listesi oluşturur
        ve ZigzagMission'a gönderir.
        """
        rospy.loginfo("--- ZIGZAG GÖREVİ TETİKLENİYOR ---")
        
        # Bulduğumuz min/max değerlerinden sanal bir "Oda" oluşturuyoruz (4 Köşe)
        # Eğer boundary_explorer gerçek köşe listesi verseydi onu kullanırdık.
        # Şimdilik "Bounding Box" yöntemi en güvenlisidir.
        
        corners = [
            (self.min_x, self.min_y), # Sol Alt
            (self.max_x, self.min_y), # Sağ Alt
            (self.max_x, self.max_y), # Sağ Üst
            (self.min_x, self.max_y)  # Sol Üst
        ]
        
        # Alan kontrolü
        if (self.max_x - self.min_x) < 0.5 or (self.max_y - self.min_y) < 0.5:
            rospy.logwarn("Taranacak alan çok küçük! Zigzag iptal ediliyor.")
            return

        # Yeni sınıfı çağır ve görevi başlat
        ziggy = ZigzagMission()
        ziggy.execute_mission(corners)

    def run(self):
        # ----------------------------------------------
        # 1. FAZ: DUVAR TAKİBİ (EXPLORATION)
        # ----------------------------------------------
        rospy.sleep(2.0)
        rospy.loginfo("FAZ 1: DUVAR TAKİBİ BAŞLATILIYOR...")
        self.explorer_pub.publish(Bool(True))
        
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            current_pos = self.get_robot_pose()
            if self.wall_follow_start_pos and current_pos:
                # Sınırları sürekli güncelle
                self.update_boundaries(current_pos.x, current_pos.y)
                
                # Başlangıç noktasına geri döndük mü?
                dist = math.sqrt((current_pos.x - self.wall_follow_start_pos.x)**2 + 
                                 (current_pos.y - self.wall_follow_start_pos.y)**2)
                
                # Biraz uzaklaştıktan sonra tekrar yaklaşırsak tur tamamlanmıştır
                if self.exploration_started_flag and dist > 2.0: 
                    self.exploration_started_flag = False # Uzaklaştık
                
                if not self.exploration_started_flag and dist < 1.0 and self.wall_follow_start_pos:
                    rospy.loginfo("Tur tamamlandı! Duvar takibi bitiyor.")
                    self.explorer_pub.publish(Bool(False)) # Explorer'ı sustur
                    rospy.sleep(1.0)
                    self.refresh_costmaps() # Move Base temizlensin
                    break 
            rate.sleep()

        # ----------------------------------------------
        # 2. FAZ: ZIGZAG TARAMA (MOVE BASE İLE)
        # ----------------------------------------------
        rospy.loginfo("FAZ 2: ZIGZAG TARAMA BAŞLATILIYOR...")
        self.execute_zigzag_mission()
        
        # ----------------------------------------------
        # 3. FAZ: KAYIT VE GUI
        # ----------------------------------------------
        rospy.loginfo("FAZ 3: HARİTA KAYDI VE GUI...")
        self.save_map_automatically()
        self.refresh_costmaps()
        
        rospy.loginfo("\n===========================================")
        rospy.loginfo(" BÜTÜN GÖREVLER TAMAMLANDI. GUI AÇILIYOR... ")
        rospy.loginfo("===========================================\n")
        
        gui_path = os.path.expanduser("~/catkin_ws/src/otonom_robot/scripts/robot_gui.py")
        if os.path.exists(gui_path):
            env = os.environ.copy()
            subprocess.call(["python3", gui_path], env=env)
        else:
            rospy.logerr(f"GUI bulunamadı: {gui_path}")
            
        rospy.loginfo("Sistem Kapatılıyor.")

if __name__ == '__main__':
    try: MissionControl()
    except rospy.ROSInterruptException: pass
