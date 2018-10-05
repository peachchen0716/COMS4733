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
        self.range_min = 2
        self.range_max = 0
        self.range_right = None
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.cmd_vel = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=5)
        self.rate = 10
        self.r = rospy.Rate(self.rate)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/odom'
        self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
        self.base_frame = '/base_footprint'
        
        rospy.on_shutdown(self.shutdown)
        # Get the starting position values     
        (position, rotation) = self.get_odom()
                        
        x_start = position.x
        y_start = position.y
        self.mline = self.find_mline([x_start, y_start], [10.0, 0.0])
        self.dst_tol = 1e-7
        self.angle_tol = 1e-3
        angle = math.atan2(0.0 - y_start, 10.0 - x_start)

        # turn towards m-line 
        obj_dst = 0.8

        hit_point = set() 

        self.rotate(-90)
        (position, rotation) = self.get_odom()
        print rotation
        self.turn_back(rotation)
        print "start bug2"
        while False: 
            (position, rotation) = self.get_odom()
            reach_goal = (position.x == 10.0 and position.y == 0.0) 

            while not reach_goal:
                if not math.isnan(self.range_min) and (self.range_min < obj_dst):
                    break
                self.translate(0.5)
                # rospy.sleep(1)  
            
            print "found an obstacle"             
            (position, rotation) = self.get_odom()
            hit_point.add(position)
            
            # turn left 
            while True:
                self.rotate(10.0)
                # rospy.sleep(1)
                
                if self.all_nan:
                    break
            
            # follow boundary 
            print "start following boundary"
            while True:
                self.translate(0.3)
                # turn until it see the object
                while math.isnan(self.range_min):
                    print "turn right"
                    self.rotate(-10.0)
                    if self.range_min < obj_dst:
                        break
                while not math.isnan(self.range_min):
                    print "turn left"
                    self.rotate(10.0)
                    if self.range_min > obj_dst:
                        break

                (position, rotation) = self.get_odom()
                if self.is_on_mline(position) == True:
                    print "m-line reached"
                    break
                if position in hit_point:
                    print "impossible"
            break
        print "done"
    
    def translate(self, dist):
        goal_distance = dist
        linear_speed = 0.5
        if dist < 0:
            linear_speed *= -1
        linear_duration = goal_distance / linear_speed
        
        move_cmd = Twist()        
        move_cmd.linear.x = linear_speed        
        ticks = int(linear_duration * self.rate)
        
        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
        
        # Stop the robot before the rotation
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)
        
        # Stop the robot
        self.cmd_vel.publish(Twist())

    def rotate(self, deg):
        goal_angle = deg * pi / 180.0
        angular_speed = 0.5
        if deg < 0:
            angular_speed *= -1
        angular_duration = goal_angle / angular_speed
                
        move_cmd = Twist()            
        move_cmd.angular.z = angular_speed

        # Rotate for a time to go 180 degrees
        ticks = int(angular_duration * self.rate)
        #ticks = abs(ticks) 
        for t in range(ticks):           
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
                
        # Stop the robot before the next leg
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)    
          
        # Stop the robot
        self.cmd_vel.publish(Twist())

    def rotate_rad(self, rad):
        angular_speed = 0.3
        if rad < 0:
            angular_speed *= -1
        angular_duration = rad / angular_speed
                
        move_cmd = Twist()            
        move_cmd.angular.z = angular_speed

        # Rotate for a time to go 180 degrees
        ticks = int(angular_duration * self.rate)
        #ticks = abs(ticks) 
        for t in range(ticks):           
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
                
        # Stop the robot before the next leg
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)    
          
        # Stop the robot
        self.cmd_vel.publish(Twist())

    def scan_callback(self, msg):
        self.all_nan = np.all(np.isnan(msg.ranges))
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]
        if self.all_nan:
            self.range_max = float('nan')
            self.range_min = float('nan')
        else:
            self.range_max = np.nanmax(msg.ranges)
            self.range_min = np.nanmin(msg.ranges)
        self.range_right = msg.ranges[len(msg.ranges) - 1]

    def find_mline(self, start, target):
        slope = (target[1] - start[1]) / (target[0] - start[0])
        intercept = target[1] - target[0] * slope
        return (slope, intercept)

    def is_on_mline(self, position):
        diff = position.x * self.mline[0] + self.mline[1] - position.y 
        if ( abs(diff) < self.dst_tol):
            return True
        return False

    def turn_back(self, rad):
        self.rotate_rad(-1 * rad)

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def shutdown(self): 
        # Always stop the robot when shutting down the node.  
        rospy.loginfo("Stopping the robot...") 
        self.cmd_vel.publish(Twist()) 
        rospy.sleep(1) 

if __name__ == '__main__': 
    try: 
        Bug2() 
    except: 
        rospy.loginfo("Bug2 node terminated.")
    exit()
