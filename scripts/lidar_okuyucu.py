#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
import math

def get_average_distance(ranges, index, window=3):
    """Belirli bir indeksin etrafındaki verilerin ortalamasını alır."""
    valid_values = []
    start = max(0, index - window)
    end = min(len(ranges), index + window)
    
    for i in range(start, end):
        val = ranges[i]
        if not math.isinf(val) and not math.isnan(val) and val > 0.0:
            valid_values.append(val)
            
    if len(valid_values) == 0:
        return float('inf')
    
    return sum(valid_values) / len(valid_values)

def scan_callback(msg):
    count = len(msg.ranges)
    ranges = list(msg.ranges)
    
    # --- 4 ANA YÖN (İndeks Bazlı) ---
    # Hangi indeksin hangi yöne denk geldiğini test etmek için
    idx_0   = 0
    idx_90  = int(count * 0.25)
    idx_180 = int(count * 0.50)
    idx_270 = int(count * 0.75)
    
    dist_0   = get_average_distance(ranges, idx_0)
    dist_90  = get_average_distance(ranges, idx_90)
    dist_180 = get_average_distance(ranges, idx_180)
    dist_270 = get_average_distance(ranges, idx_270)

    # --- EN YAKIN ENGEL NEREDE? ---
    # Bu kısım yönü bulmanı sağlar.
    # Elini robotun önüne koyduğunda buradaki 'Min Index' ve 'Min Derece'
    # neyi gösteriyorsa, robotun önü orasıdır.
    valid_ranges = [r for r in ranges if not math.isinf(r) and r > 0.1]
    if valid_ranges:
        min_dist = min(valid_ranges)
        min_index = ranges.index(min_dist)
        # İndeksi dereceye çevir (Yaklaşık)
        min_degree = (min_index / count) * 360.0
    else:
        min_dist = 99.9
        min_index = 0
        min_degree = 0.0

    def fmt(val):
        if math.isinf(val): return "--- "
        return f"{val:.2f}m"

    # --- EKRANA BAS ---
    # Terminali temizlemeden sürekli satır günceller
    output = (
        f"| 0 DERECE (Baslangic): {fmt(dist_0)} | "
        f"90 DERECE (%25): {fmt(dist_90)} | "
        f"180 DERECE (%50): {fmt(dist_180)} | "
        f"270 DERECE (%75): {fmt(dist_270)} | "
        f"--> EN YAKIN: {fmt(min_dist)} @ {min_degree:.0f} Derece (Index: {min_index})"
    )
    print(output)

def main():
    rospy.init_node('lidar_yon_testi', anonymous=True)
    
    # Test için ham veriyi dinlemek daha iyidir
    topic_name = '/scan_fixed' 
    
    rospy.Subscriber(topic_name, LaserScan, scan_callback)
    rospy.loginfo(f"Lidar YON TESTI Baslatildi. {topic_name} dinleniyor...")
    rospy.loginfo("TEST: Elini robotun ONUNE koy ve 'EN YAKIN' kisminda hangi derecenin yazdigina bak.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
