#!/usr/bin/env python
import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from math import radians, copysign, sqrt, pow, pi
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

class Bug2():

    def __init__(self):
        rospy.init_node("bug2", anonymous=False)

        self.range_ahead = None
        # self.range_min = 2
        # self.range_max = 0
        self.range_right = None
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=20)
        self.rate = 20
        self.r = rospy.Rate(self.rate)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()        
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/odom'
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame,'/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

        rospy.on_shutdown(self.shutdown)
                
        obj_dst = 0.8

        hit_point = set() 

        print "start bug2"

        while True: 
            (position, rotation) = self.get_odom()

            if self.isgoal(position.x):
                print "GOAL REACHED"
                break

            while True:
                self.translate(0.2)

                # (position, rotation) = self.get_odom()
                # self.rotate_rad(-1 * rotation)

                if not math.isnan(self.range_ahead) and (self.range_ahead < obj_dst):
                    break
            
            print "found an obstacle"  
            
            (position, rotation) = self.get_odom()
            hit_point.add(position)
            
            # turn left 
            self.rotate(30)
            while not math.isnan(self.range_right):
                if self.range_right > 1:
                    break
                self.rotate(15)
            
            # follow boundary 
            print "start following boundary"
            while True:
                self.translate(0.15)

                # turn until it see the object
                while math.isnan(self.range_right) or self.range_right > 0.8:
                    print "turn right"
                    self.rotate(-10)

                while self.range_right < 0.8:
                    print "turn left"
                    self.rotate(17)
                
                self.translate(0.15)


                (position, rotation) = self.get_odom()
                if self.mline(position.x, position.y):
                    print "m-line reached"
                    # turn back
                    self.rotate(-1 * rotation)
                    break
                if self.check_hitpoint(position, hit_point):
                    print "impossible"
                    return
        print "done"
    
    def check_hitpoint(self, position, hit_point):
        cur_x = position.x
        cur_y = position.y
        for point in hit_point:
            if abs(point.x - cur_x) > 0.1:
                continue
            if abs(point.y - cur_y) > 0.1:
                continue
            return True
        return False

    def translate(self, dist):
        goal_distance = dist
        linear_speed = 0.1
        if dist < 0:
            linear_speed *= -1
        linear_duration = goal_distance / linear_speed
        
        move_cmd = Twist()        
        move_cmd.linear.x = linear_speed        
        ticks = int(linear_duration * self.rate)
        
        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
        
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(0.5)

    def rotate_rad(self, rad):
        goal_angle = rad
        angular_speed = 0.5
        if rad < 0:
            angular_speed *= -1
        angular_duration = goal_angle / angular_speed
                
        move_cmd = Twist()            
        move_cmd.angular.z = angular_speed

        ticks = int(angular_duration * self.rate)
        for t in range(ticks):           
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
                
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(0.5)  

    def rotate(self, deg):
        goal_angle = deg * pi / 180.0
        angular_speed = 0.5
        if deg < 0:
            angular_speed *= -1
        angular_duration = goal_angle / angular_speed
                
        move_cmd = Twist()            
        move_cmd.angular.z = angular_speed

        ticks = int(angular_duration * self.rate)
        #ticks = abs(ticks) 
        for t in range(ticks):           
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
                
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(0.5)    

    def isgoal(self, x):
        if (9.9 < x < 10.1) and abs(y) <= 0.1:
            return True
        return False

    def mline(self, x, y):
        if abs(y) <= 0.08 and (0.1 <= x <= 10.1):
            return True
        return False

    def scan_callback(self, msg):
        # self.all_nan = np.all(np.isnan(msg.ranges))
        self.range_right = msg.ranges[0]
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]
        # if self.all_nan:
        #     self.range_max = float('nan')
        #     self.range_min = float('nan')
        # else:
        #     self.range_max = np.nanmax(msg.ranges)
        #     self.range_min = np.nanmin(msg.ranges)

    # def find_mline(self, start, target):
    #     slope = (target[1] - start[1]) / (target[0] - start[0])
    #     intercept = target[1] - target[0] * slope
    #     return (slope, intercept)

    # def is_on_mline(self, position):
    #     diff = position.x * self.mline[0] + self.mline[1] - position.y 
    #     if ( abs(diff) < self.dst_tol):
    #         return True
    #     return False

    def get_odom(self):
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def shutdown(self): 
        rospy.loginfo("Stopping the robot...") 
        self.cmd_vel.publish(Twist()) 
        rospy.sleep(1) 

if __name__ == '__main__': 
    try: 
        Bug2() 
    except: 
        rospy.loginfo("Bug2 node terminated.")
    exit()
