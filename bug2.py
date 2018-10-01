#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from math import radians, copysign, sqrt, pow, pi
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

class Bug2():

    def __init__(self):
        
        rospy.init_node("bug2", anonymous=False)

        self.g_range_ahead = 1 
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        rate = 10
        self.r = rospy.Rate(rate)
        linear_speed = 0.2

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
            
        #use conditional variable
        reach_goal = False
        #reach_goal = True
        
        twist = Twist()
        twist.linear.x = 1

        for i in range(10):
            self.cmd_vel.publish(twist)
            self.r.sleep()
        twist = Twist()
        self.cmd_vel.publish(twist)
        
        while not reach_goal:
            if not encounter_object:
                self.cmd_vel.publish(twist)             
                encounter_object = (g_range_ahead < 0.8)
            else:
                self.cmd_vel.publish(twist)
                encounter_object = (g_range_right > 0) 
            twist = Twist()
            if encounter_object:
                twist.angular.z = 1
            else:
                twist.linear.x = 1

        print "done"

    def scan_callback(self, msg):
        global g_range_ahead
        global g_range_right
        g_range_ahead = min(msg.ranges) 
        g_range_right = msg.ranges(leng(msg.ranges) - 1)

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
