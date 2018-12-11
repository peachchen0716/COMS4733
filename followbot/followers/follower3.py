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

    lower_red = numpy.array([ 0,  70,  80])
    upper_red = numpy.array([10, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20

    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0

    mask_red[0:search_top, 0:w] = 0
    mask_red[search_bot:h, 0:w] = 0

    M_red = cv2.moments(mask_red)

    M = cv2.moments(mask)

    if M_red['m00'] > 0:
      
      contours, hcontour = cv2.findContours(numpy.copy(mask_red),1,2)
      approx = []
      count = 0
      for cnt in contours:
          approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
          count += approx.shape[0]
      if count > 10 and len(contours):
        print "found ", count, " vertices with ", len(contours), "contours"
        rospy.signal_shutdown("find goal") 


      # if len(approx) == 3:
      #     p1 = approx[0][0]
      #     p2 = approx[1][0]
      #     p3 = approx[2][0]

      #     print p1, p2, p3

      min_x = min(approx[:, 0][:, 0])
      max_x = max(approx[:, 0][:, 0])

      min_y = min(approx[:, 0][:, 1])
      max_y = max(approx[:, 0][:, 1])

      if max_x - min_x > 100:
        center = (max_x + min_x) / 2
        # print "center of red mask: ", center
        left_copy = numpy.copy(mask_red)
        right_copy = numpy.copy(mask_red)
        left_copy[0:h, center:w] = 0
        right_copy[0:h, 0:center] = 0

        if numpy.count_nonzero(left_copy) < numpy.count_nonzero(right_copy):
          self.twist.linear.x = 0.7
          self.twist.angular.z = 0.5
          self.cmd_vel_pub.publish(self.twist)
        else:
          self.twist.linear.x = 0.7
          self.twist.angular.z = -0.5
          self.cmd_vel_pub.publish(self.twist)
        return

    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)

    cv2.imshow("window", mask_red)
    cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
