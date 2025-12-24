#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class BoundaryExplorer:
    def __init__(self):
        # Düğümü başlat
        rospy.init_node('boundary_explorer_node', anonymous=True)
        
        # Hız komutlarını yayınlamak için publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Düzeltilmiş Lidar verilerini dinlemek için subscriber
        # Önceki adımda oluşturduğumuz /scan_fixed topic'ini dinliyoruz!
        self.scan_sub = rospy.Subscriber('/scan_fixed', LaserScan, self.scan_callback, queue_size=1)
        
        # Robotun durumunu tutacak değişken
        # Olası durumlar: 'duvar_bul', 'sola_don', 'duvari_takip_et'
        self.state = 'duvar_bul'
        
        # Parametreler (bunları kendi simülasyonunuza göre ayarlayabilirsiniz)
        self.forward_speed = 0.15          # İleri yönlü hız
        self.turning_speed = 0.4           # Dönüş hızı
        self.desired_distance_wall = 0.4   # Duvardan istenen uzaklık (metre)
        self.collision_distance = 0.3      # Çarpışma olarak kabul edilecek ön mesafe
        
        # Kontrolcü için oransal kazanç (Proportional gain)
        # Hatanın (istenilen mesafe - mevcut mesafe) ne kadar hızlı düzeltileceğini belirler.
        self.kp = 1.5

        self.rate = rospy.Rate(10) # 10 Hz
        rospy.loginfo("Sınır Keşfi Düğümü Başlatıldı.")

    def scan_callback(self, msg):
        """
        Lidar verisi her geldiğinde bu fonksiyon çalışır ve durumu yönetir.
        """
        # Lidar verisinden ilgili yönlerdeki mesafeleri alıyoruz
        # 360 derecelik bir Lidar için:
        # 0. index: tam ön
        # 90. index: tam sol
        # 270. index: tam sağ
        front_dist = msg.ranges[0]
        right_dist = msg.ranges[270]
        front_right_dist = msg.ranges[315] # Sağ-ön çapraz

        # Sonsuz değerleri (engel yok) daha yönetilebilir bir sayı ile değiştir
        if math.isinf(right_dist):
            right_dist = msg.range_max

        # --- DURUM MAKİNESİ ---
        
        # 1. Durum: DUVAR_BUL
        if self.state == 'duvar_bul':
            rospy.loginfo("Durum: Duvar Bulunuyor...")
            if front_dist > self.collision_distance:
                self.move_forward()
            else:
                self.stop_robot()
                self.state = 'sola_don'

        # 2. Durum: SOLA_DON
        elif self.state == 'sola_don':
            rospy.loginfo("Durum: Sola Dönülüyor...")
            # Önü boşalana kadar sola dön.
            if front_dist > self.collision_distance * 1.2:
                self.stop_robot()
                self.state = 'duvari_takip_et'
            else:
                self.turn_left()

        # 3. Durum: DUVARI_TAKIP_ET
        elif self.state == 'duvari_takip_et':
            rospy.loginfo(f"Durum: Duvar Takip Ediliyor... Sağ Mesafe: {right_dist:.2f}m")
            
            # Öncelik 1: Çarpışmayı önle (İç köşe veya engel)
            if front_dist < self.collision_distance:
                self.turn_left()
                rospy.logwarn("Engel algılandı! Sola dönülüyor.")
                return

            # Hata hesaplaması (Oransal Kontrolcü)
            error = self.desired_distance_wall - right_dist
            
            # Dönüş hızını hataya göre ayarla
            turn_cmd = self.kp * error
            
            # Dönüş hızını makul limitler içinde tut
            turn_cmd = max(min(turn_cmd, self.turning_speed), -self.turning_speed)
            
            # Hız komutunu oluştur
            twist = Twist()
            twist.linear.x = self.forward_speed
            twist.angular.z = turn_cmd
            
            self.cmd_vel_pub.publish(twist)

    def move_forward(self):
        twist = Twist()
        twist.linear.x = self.forward_speed
        self.cmd_vel_pub.publish(twist)

    def turn_left(self):
        twist = Twist()
        twist.angular.z = self.turning_speed
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def run(self):
        # Düğüm kapatılana kadar bekle
        rospy.spin()

if __name__ == '__main__':
    try:
        explorer = BoundaryExplorer()
        explorer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Düğüm kapatıldı.")