#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
import tf  
import dynamic_reconfigure.client
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan

class SmartNavigator:
    def __init__(self):
        # Lidar Verisi
        self.current_tunnel_distance = 99.9
        
        # Hız Bölgeleri [(min_x, max_x, min_y, max_y), ...]
        self.speed_zones = []
        self.current_speed_mode = "NORMAL"
        
        # Hız Ayarları
        self.NORMAL_SPEED = 0.2
        self.SLOW_SPEED = 0.15 
        
        self.tf_listener = tf.TransformListener()
        
        # Dynamic Reconfigure (Hız değiştirmek için)
        self.reconf_client = None
        try:
            self.reconf_client = dynamic_reconfigure.client.Client("/move_base/TrajectoryPlannerROS", timeout=2.0)
        except:
            rospy.logwarn("[SmartNav] Hız kontrolcüsüne bağlanılamadı (Simülasyon veya farklı planner).")

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        rospy.loginfo("[SmartNav] Move Base bekleniyor...")
        self.client.wait_for_server()
        rospy.loginfo("[SmartNav] HAZIR.")

    def add_speed_zone(self, points):
        """Hız bölgesi ekler (Kare/Dikdörtgen sınırları)."""
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zone = (min(xs), max(xs), min(ys), max(ys))
        self.speed_zones.append(zone)
        rospy.loginfo(f"[SmartNav] Hız Bölgesi Eklendi: X[{min(xs):.1f}-{max(xs):.1f}]")

    def check_speed_zones(self, robot_x, robot_y):
        """Robot bölgedeyse hızı düşürür."""
        if not self.reconf_client: return

        in_zone = False
        for (x1, x2, y1, y2) in self.speed_zones:
            if x1 <= robot_x <= x2 and y1 <= robot_y <= y2:
                in_zone = True
                break
        
        if in_zone and self.current_speed_mode == "NORMAL":
            self.reconf_client.update_configuration({"max_vel_x": self.SLOW_SPEED})
            self.current_speed_mode = "SLOW"
            rospy.loginfo(f">>> YAVAŞ BÖLGE! Hız {self.SLOW_SPEED} m/s")
            
        elif not in_zone and self.current_speed_mode == "SLOW":
            self.reconf_client.update_configuration({"max_vel_x": self.NORMAL_SPEED})
            self.current_speed_mode = "NORMAL"
            rospy.loginfo(f"<<< NORMAL BÖLGE. Hız {self.NORMAL_SPEED} m/s")

    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        total = len(ranges)
        idx_start = int(total * 0.47)
        idx_end   = int(total * 0.53)
        front_slice = ranges[idx_start : idx_end]
        valid = [r for r in front_slice if r > 0.1 and not math.isinf(r)]
        if valid:
            self.current_tunnel_distance = min(valid)
        else:
            self.current_tunnel_distance = 99.9

    def go_to(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        # --- DİNAMİK YÖN HESAPLAMASI (Robotun hedefe dönerek gitmesi) ---
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            current_x = trans[0]
            current_y = trans[1]
            theta = math.atan2(y - current_y, x - current_x)
            quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
            
            goal.target_pose.pose.orientation.x = quaternion[0]
            goal.target_pose.pose.orientation.y = quaternion[1]
            goal.target_pose.pose.orientation.z = quaternion[2]
            goal.target_pose.pose.orientation.w = quaternion[3]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"[SmartNav] Gidiliyor -> {x},{y}")
        self.client.send_goal(goal)
        
        waiting = False
        bypass = False 
        timer = 0

        while not rospy.is_shutdown():
            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                return True
            if state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
                if bypass: return False

            # --- 1. HIZ BÖLGESİ KONTROLÜ ---
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                current_x = trans[0]
                current_y = trans[1]
                self.check_speed_zones(current_x, current_y)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass 
            
            # --- 2. ENGEL KONTROLÜ (İPTAL EDİLDİ) ---
            # if not bypass:
            #     if self.current_tunnel_distance < 1.2:
            #         if not waiting:
            #             self.client.cancel_goal()
            #             waiting = True
            #             timer = rospy.get_time()
            #             rospy.logwarn("ENGEL VAR! Bekleniyor...")
            #         elif (rospy.get_time() - timer) > 5.0:
            #             rospy.logwarn("DOLANMA MODU.")
            #             bypass = True; waiting = False
            #             self.client.send_goal(goal)
            #     elif waiting and self.current_tunnel_distance > 1.4:
            #         waiting = False
            #         self.client.send_goal(goal)
            
            rospy.sleep(0.1)
        return False
