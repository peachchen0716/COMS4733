#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from math import sqrt, pow, pi

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.twist = Twist()
    self.rate = 20
    self.r = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)

  def shutdown(self):
    rospy.loginfo("Stopping the robot...")
    rospy.sleep(1)

  def image_callback(self, msg):

    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 10,  10,  10])
    upper_yellow = numpy.array([255, 255, 250])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    lower_green = numpy.array([60, 90, 90])
    upper_green = numpy.array([100, 255, 200])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    lower_red = numpy.array([ 0,  70,  80])
    upper_red = numpy.array([10, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    lower_blue = numpy.array([ 110,  50,  20])
    upper_blue = numpy.array([130, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0

    mask_green[0:search_top, 0:w] = 0
    mask_green[search_bot:h, 0:w] = 0

    mask_red[0:search_top, 0:w] = 0
    mask_red[search_bot:h, 0:w] = 0

    mask_blue[0:search_top, 0:w] = 0
    mask_blue[search_bot:h, 0:w] = 0

    M_green = cv2.moments(mask_green)
    M_red = cv2.moments(mask_red)
    M_blue = cv2.moments(mask_blue)

    M = cv2.moments(mask)


    if M_green['m00'] > 0:
      print "M green: ", M_green['m00']
      self.twist.linear.x = 0.7
      self.twist.angular.z = 0.5
      self.cmd_vel_pub.publish(self.twist)

    elif M_red['m00'] > 0:
      print "M red: ", M_red['m00']
      self.cmd_vel_pub.publish(self.twist)
      print "find goal"
      rospy.signal_shutdown("find goal") 

    elif M_blue['m00'] > 0:
      print "M blue: ", M_blue['m00']
      self.twist.linear.x = 0.7
      self.twist.angular.z = -0.5
      self.cmd_vel_pub.publish(self.twist)

    elif M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)

    cv2.imshow("window", image)
    cv2.waitKey(3)

  def translate(self, dist):
    goal_distance = dist
    linear_speed = 0.7
    if dist < 0:
      linear_speed *= -1
    linear_duration = goal_distance/linear_speed


    move_cmd = Twist()
    move_cmd.linear.x = linear_speed
    ticks = int(linear_duration * self.rate)
    for t in range(ticks):
      self.cmd_vel_pub.publish(move_cmd)
      self.r.sleep()

    move_cmd = Twist()
    self.cmd_vel_pub.publish(move_cmd)
    rospy.sleep(0.4)

  def rotate(self, deg):
    goal_angle = deg * pi / 180.0
    # goal_angle = deg
    angular_speed = 0.5
    if deg < 0:
      angular_speed *= -1
    angular_duration = goal_angle / angular_speed

    move_cmd = Twist()
    move_cmd.angular.z = angular_speed

    ticks = int(angular_duration * self.rate)
    for t in range(ticks):
      self.cmd_vel_pub.publish(move_cmd)
      self.r.sleep()

    move_cmd = Twist()
    self.cmd_vel_pub.publish(move_cmd)

    rospy.sleep(1)

    self.cmd_vel_pub.publish(Twist())

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL


