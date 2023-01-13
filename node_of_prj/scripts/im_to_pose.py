#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MySubscriber(object):

    def __init__(self):
        rospy.init_node('im_subscriber')
        self.subscription = rospy.Subscriber('/camera/image_raw', Image, self.listener_callback)
        self.publisher = rospy.Publisher('/ball_pose', Point, queue_size=10)
        self.bridge = CvBridge()
        self.index = 0

    def listener_callback(self, msg):
        # Convert the sensor_msgs/Image message to a OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        bgr = cv2.cvtColor(cv_image,cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lower_value = np.array([00,80,0])
        higher_value = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_value, higher_value)
        mask = cv2.blur(mask,(6,6))                        
        mask = cv2.erode(mask, None, iterations=2)         
        mask = cv2.dilate(mask, None, iterations=2)        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        x = 1.0
        y = 1.0
        radius = 1.0 # 100-35
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 10:
                center_coordinates, radius = cv2.minEnclosingCircle(cnt)
                cfx = float(320)
                cfy = float(240)
                x = float(center_coordinates[0])
                y = float(center_coordinates[1])
                x = x - cfx
                y = -1*(y - cfy)
                radius = radius
        print(x,y)
        # Publish the center point as a geometry_msgs/Point message
        point_msg = Point()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = radius
        self.publisher.publish(point_msg)
        self.index = self.index + 1

if __name__ == '__main__':
    try:
        my_subscriber = MySubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass