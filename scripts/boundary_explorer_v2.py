#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import math

class BoundaryExplorerV2:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.state_pub = rospy.Publisher('~state', String, queue_size=1)
        
        # /scan dinliyoruz
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        
        self.is_active = False 
        self.control_sub = rospy.Subscriber('/explorer_enabled', Bool, self.control_callback)
        
        rospy.loginfo("Sinir Kesfi (HASSAS MERKEZLEME MODU: HEDEF %50) Baslatildi.")
        
        self.state = 'baslangic_tarama'
        
        # --- AYARLAR ---
        self.forward_speed = 0.15
        self.turning_speed = 0.4
        
        self.desired_distance_wall = 0.75 
        self.collision_distance = 0.80    
        self.panic_distance = 0.50        
        
        self.robot_blind_radius = 0.30
        
        # PID
        self.kp = 0.6   
        self.kd = 4.0   
        self.prev_error = 0.0
        self.last_time = rospy.get_time()

    def control_callback(self, msg):
        if msg.data and not self.is_active:
            rospy.loginfo("Sınır Keşfi Aktif Edildi!")
            self.state = 'baslangic_tarama' 
            self.last_time = rospy.get_time()
        elif not msg.data and self.is_active:
            rospy.loginfo("Sınır Keşfi Pasif Edildi!")
            self.stop_robot()
        self.is_active = msg.data    

    def get_sector_min(self, ranges, start_percent, end_percent):
        total_points = len(ranges)
        start_idx = int(total_points * (start_percent / 100.0))
        end_idx = int(total_points * (end_percent / 100.0))
        
        if start_idx > end_idx: 
             slice_data = ranges[start_idx:] + ranges[:end_idx]
        else:
             slice_data = ranges[start_idx:end_idx]
             
        valid_data = [r for r in slice_data if r > self.robot_blind_radius and not math.isinf(r)]
        
        if not valid_data:
            return 99.9
        return min(valid_data)

    def scan_callback(self, msg):
        if not self.is_active:
            return

        ranges = list(msg.ranges)
        total_points = len(ranges)
        
        # --- YÖN HARİTASI (0=ARKA, 180=ON -> %50=ON) ---
        dist_front = self.get_sector_min(ranges, 45, 55)
        
        # Sağ Takip İçin (%20-%30 arası = Sağ)
        dist_right = self.get_sector_min(ranges, 20, 30)
        precise_right_dist = dist_right

        previous_state = self.state

        # --- DURUM MAKİNESİ ---

        # 1. Durum: BAŞLANGIÇ (Hassas Merkezleme)
        if self.state == 'baslangic_tarama':
            
            # 1. Tüm geçerli verileri al ve en yakınını bul
            # Sadece 3.0 metre içindeki engellere odaklan (Uzaktakilere kilitlenme)
            valid_indices = [i for i, r in enumerate(ranges) if self.robot_blind_radius < r < 3.0 and not math.isinf(r)]
            
            if not valid_indices:
                rospy.loginfo_throttle(1, "Yakinlarda duvar yok, araniyor...")
                self.rotate_robot(1)
                return

            # En yakın noktanın indeksini ve mesafesini bul
            # valid_indices içindeki en küçük mesafeye sahip indeksi bul
            min_idx = min(valid_indices, key=lambda i: ranges[i])
            min_dist = ranges[min_idx]
            
            # 2. İndeksi Yüzdeye Çevir (%0 - %100)
            current_percent = (min_idx / total_points) * 100.0
            
            # 3. HEDEF: %50 (Lidar ters olduğu için %50 = Ön)
            target_percent = 50.0
            tolerance = 2.0 # +/- %2 hata payı (Çok hassas)

            error = current_percent - target_percent
            
            rospy.loginfo_throttle(0.2, f"Hizalama -> Engel: {min_dist:.2f}m | Konum: %{current_percent:.1f} | Hedef: %50")

            # 4. HİZALAMA MANTIĞI
            if abs(error) < tolerance:
                # Hedef tam merkezde!
                rospy.loginfo("HEDEF KILITLENDI (%50). Yaklasiliyor...")
                self.stop_robot()
                self.state = 'duvar_bul'
            
            elif error > 0: 
                # Engel %50'den büyükte (Solda) -> Merkeze çekmek için SOLA dön
                # (Robot Sola dönünce, engel Lidar dizisinde sağa/aşağı kayar)
                self.rotate_robot(1) 
            
            else: 
                # Engel %50'den küçükte (Sağda) -> Merkeze çekmek için SAĞA dön
                self.rotate_robot(-1)

        
        # 2. Durum: DUVARA YAKLAŞ
        elif self.state == 'duvar_bul':
            if dist_front > self.collision_distance:
                self.move_forward()
            else:
                rospy.loginfo(f"Duvara geldim ({dist_front:.2f}m).")
                self.stop_robot()
                self.state = 'sola_don'


        # 3. Durum: SOLA DÖN
        elif self.state == 'sola_don':
            if dist_front > 1.2 and precise_right_dist < 1.5:
                self.state = 'duvari_takip_et'
                self.prev_error = 0
                self.last_time = rospy.get_time()
            else:
                self.rotate_robot(1) 


        # 4. Durum: DUVAR TAKİP
        elif self.state == 'duvari_takip_et':
            # Acil Durumlar
            if dist_front < self.collision_distance:
                self.rotate_robot(1) 
                self.prev_error = 0
                return
            
            if precise_right_dist < self.panic_distance:
                 t = Twist()
                 t.linear.x = 0.05
                 t.angular.z = 0.4 
                 self.cmd_vel_pub.publish(t)
                 return

            # PID
            current_time = rospy.get_time()
            dt = current_time - self.last_time
            if dt <= 0: dt = 0.1

            error = self.desired_distance_wall - precise_right_dist
            derivative = (error - self.prev_error) / dt
            
            turn_cmd = (self.kp * error) + (self.kd * derivative)
            self.prev_error = error
            self.last_time = current_time
            
            turn_cmd = max(min(turn_cmd, 0.3), -0.3)
            
            twist = Twist()
            twist.linear.x = self.forward_speed
            twist.angular.z = turn_cmd
            self.cmd_vel_pub.publish(twist)

        if self.state != previous_state:
            self.state_pub.publish(self.state)

    def move_forward(self):
        t = Twist()
        t.linear.x = self.forward_speed
        self.cmd_vel_pub.publish(t)

    def rotate_robot(self, direction):
        t = Twist()
        t.linear.x = 0.0 
        t.angular.z = self.turning_speed * direction
        self.cmd_vel_pub.publish(t)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('boundary_explorer_node_v2', anonymous=True)
        explorer = BoundaryExplorerV2()
        explorer.run()
    except rospy.ROSInterruptException:
        pass
