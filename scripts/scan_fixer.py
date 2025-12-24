#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan

# Global publisher
pub = None

def scan_callback(scan_msg):
    """
    /scan topic'inden gelen ham veriyi alır.
    Robotun kendi parçalarını (çıtalar) siler ve /scan_fixed olarak yayınlar.
    """
    fixed_scan = scan_msg
    
    # Tuple olan ranges verisini düzenlenebilir listeye çevir
    current_ranges = list(fixed_scan.ranges)
    
    # --- AYAR: KÖR NOKTA MESAFESİ (Metre) ---
    # Robotun yarıçapı + biraz güvenlik payı.
    # Kobuki için 0.30m (30cm) idealdir, çıtaları yok eder.
    min_valid_distance = 0.30 
    # ----------------------------------------

    for i in range(len(current_ranges)):
        dist = current_ranges[i]

        # FİLTRELEME MANTIĞI:
        # Eğer mesafe 30 cm'den kısaysa (ve 0 değilse), bu robotun kendisidir.
        # Bu değeri 'inf' (Sonsuz) yaparak "Burada engel yok, boşluk var" diyoruz.
        if dist < min_valid_distance and dist > 0.01:
            current_ranges[i] = float('inf')

        # (İsteğe bağlı) Çok uzak mesafeleri de temizlemek istersen:
        # elif dist > 10.0:
        #     current_ranges[i] = float('inf')

    # Düzenlenmiş listeyi mesaja geri yükle
    fixed_scan.ranges = current_ranges
    
    if pub is not None:
        pub.publish(fixed_scan)

def main():
    global pub
    rospy.init_node('scan_fixer_node', anonymous=True)
    
    # Temizlenmiş veriyi yayınla
    pub = rospy.Publisher('/scan_fixed', LaserScan, queue_size=10)
    
    # Ham veriyi dinle
    rospy.Subscriber('/scan', LaserScan, scan_callback, queue_size=1)
    
    rospy.loginfo(f"Scan Fixer Calisiyor: {0.30}m altindaki engeller (citalar) siliniyor.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
