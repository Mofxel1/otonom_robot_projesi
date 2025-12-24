#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class ZigzagMission:
    def __init__(self):
        # MissionControl tarafından çağrılacağı için burada init_node yok.
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("ZigzagNavigator: Move Base sunucusu bekleniyor...")
        # Mission Control zaten beklediği için burada takılmasın, 
        # ama ilk bağlantıyı kontrol etmek iyidir.
        if not self.client.wait_for_server(timeout=rospy.Duration(5.0)):
             rospy.logwarn("ZigzagNavigator: Move Base sunucusu HENÜZ hazır değil (bekleniyor...)")
        else:
             rospy.loginfo("ZigzagNavigator: Move Base hazır.")

        self.scan_width = 0.5  # Zigzag aralığı (Metre)

    def calculate_zigzag_path(self, corners):
        """
        Eski mantık: Köşe noktalarına (Polygon) göre içeriyi tarayacak noktaları hesaplar.
        """
        if len(corners) < 3:
            rospy.logwarn("ZigzagNavigator: Yeterli köşe yok (Min 3). Sadece merkeze gidilecek.")
            return []

        # 1. Sınırları (Bounding Box) Bul
        min_x = min(p[0] for p in corners)
        max_x = max(p[0] for p in corners)
        min_y = min(p[1] for p in corners)
        max_y = max(p[1] for p in corners)

        rospy.loginfo(f"Tarama Alanı: X[{min_x:.2f} - {max_x:.2f}], Y[{min_y:.2f} - {max_y:.2f}]")

        waypoints = []
        # Duvarlara çok yapışmamak için kenarlardan biraz (margin) içeriden başla
        margin = 0.3 
        current_x = min_x + margin
        direction = 1  # 1: Yukarı, -1: Aşağı

        while current_x <= (max_x - margin):
            # Y eksenindeki hedefler
            if direction == 1:
                y_start = min_y + margin
                y_end = max_y - margin
            else:
                y_start = max_y - margin
                y_end = min_y + margin

            # --- Ray Casting Kontrolü ---
            # Nokta gerçekten odanın içinde mi?
            
            # 1. Sütun Başlangıcı
            if self.is_inside_polygon(current_x, y_start, corners):
                waypoints.append((current_x, y_start))
            
            # 2. Sütun Bitişi
            if self.is_inside_polygon(current_x, y_end, corners):
                waypoints.append((current_x, y_end))

            # Yana kay (X ekseninde ilerle)
            current_x += self.scan_width
            
            # 3. Geçiş Noktası
            if current_x <= (max_x - margin):
                if self.is_inside_polygon(current_x, y_end, corners):
                    waypoints.append((current_x, y_end))
            
            # Yönü değiştir
            direction *= -1

        return waypoints

    def is_inside_polygon(self, x, y, poly):
        """Bir noktanın çokgenin içinde olup olmadığını kontrol eder."""
        n = len(poly)
        inside = False
        p1x, p1y = poly[0]
        for i in range(n + 1):
            p2x, p2y = poly[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def move_to_goal(self, x, y):
        """Move Base kullanarak akıllı sürüş."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0 # Düz duruş

        rospy.loginfo(f"Hedefe Gidiliyor -> X: {x:.2f}, Y: {y:.2f}")
        
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        
        if not wait:
            return False
        return self.client.get_state() == 3 # 3 = SUCCEEDED

    def execute_mission(self, corners):
        rospy.loginfo("--- ZIGZAG TARAMA (MoveBase) BAŞLIYOR ---")
        
        path = self.calculate_zigzag_path(corners)
        
        if not path:
            rospy.logwarn("Rota oluşturulamadı! Alan çok küçük olabilir.")
            return

        rospy.loginfo(f"Toplam {len(path)} hedef nokta belirlendi.")

        for i, (x, y) in enumerate(path):
            if rospy.is_shutdown(): break
            
            # Noktaya git
            success = self.move_to_goal(x, y)
            
            if not success:
                rospy.logwarn(f"Nokta {i+1} atlanıyor (Engel var veya ulaşılamaz).")

        rospy.loginfo("--- ZIGZAG GÖREVİ TAMAMLANDI ---")
