#!/usr/bin/env python


import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Int64
from trucky_custom_msgs.msg import ActuatorsState
from ackermann_msgs.msg import AckermannDriveStamped

bridge = CvBridge()


class ReverseAckerman:
    def __init__(self):

        
        rospy.init_node('color_detection_node')

        self.bridge = CvBridge()
        self.cv_image = None
        self.ackerman_msg = AckermannDriveStamped()

        self.redlow1 = np.array([0, 100, 20], np.uint8)
        self.redhigh1 = np.array([8, 255, 255], np.uint8)
        self.redlow2 = np.array([175, 100, 20], np.uint8)
        self.redhigh2 = np.array([179, 255, 255], np.uint8)
        self.greenhigh = np.array([86,255,255], np.uint8)
        self.greenlow = np.array([36,0,0], np.uint8)

        #rospy.init_node('reverseAckerman')
        self.pub = rospy.Publisher("/actuator_state", ActuatorsState, queue_size=10)
        self.pub_ackerman = rospy.Publisher("/ackerman_cmd", AckermannDriveStamped, queue_size=10)
        image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.traffic_light_pub = rospy.Publisher('/traffic_light_state', String, queue_size=10)
        self.rate = rospy.Rate(50)  # Rate of 50 Hz
        self.image_pub_red = rospy.Publisher('/processed_image', Image, queue_size=10)
        self.image_pub_green = rospy.Publisher('/processed_image_green', Image, queue_size=10)

        

    def image_callback(self, msg):
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #red mask creation
        mask_red1 = cv2.inRange(frameHSV, self.redlow1, self.redhigh1)
        mask_red2 = cv2.inRange(frameHSV, self.redlow2, self.redhigh2)
        mask_red = cv2.add(mask_red1, mask_red2)
        self.mask_red_view = cv2.bitwise_and(frame, frame, mask=mask_red)

        mask_green = cv2.inRange(frameHSV, self.greenlow, self.greenhigh)
        self.mask_green_view = cv2.bitwise_and(frame, frame, mask=mask_green)

        contours_red, _ =cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)       
        contours_green, _ =cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	mask_green = cv2.inRange(frameHSV, self.greenlow, self.greenhigh)
        self.mask_green_view = cv2.bitwise_and(frame, frame, mask=mask_green)       

        if contours_red:
            print("red")
            self.traffic_light_pub.publish("red")
        elif contours_green and not contours_red:
            self.traffic_light_pub.publish("green")
        else:
            self.traffic_light_pub.publish("nothing")
            print("nothing")


        #processed_image_red =self.bridge.cv2_to_imgmsg(self.mask_red_view,"bgr8")
        #self.image_pub_red.publish(processed_image_red)
        #processed_image_green =self.bridge.cv2_to_imgmsg(self.mask_green_view,"bgr8")
        #self.image_pub_green.publish(processed_image_green)
        #rospy.sleep(1)
        



        
        
        
        # print("si llego")
        #cv2.imshow('Original_Image', frame)
        #cv2.imshow('Mask_Red_Show', mask_red)
        #cv2.imshow('Mask_Red_View', mask_red_view)
        #green mask creation
        mask_green = cv2.inRange(frameHSV, self.greenlow, self.greenhigh)
        self.mask_green_view = cv2.bitwise_and(frame, frame, mask=mask_green)
        #cv2.imshow('Mask_Green_Show', mask_green)
        #cv2.imshow('Mask_Green_View', mask_green_view)
        # actuator_state = ActuatorsState()
        # actuator_state.motor_pwm_high_time = int((msg.drive.speed / 18) * (1700 - 1520) + 1520)
        # actuator_state.servo_pwm_high_time = int((msg.drive.steering_angle / 180) * (2000 - 1000) + 1000)
        # actuator_state.output_mode = ""
        # self.pub.publish(actuator_state)

    def run(self):
        while not rospy.is_shutdown():
            if(self.cv_image is not None and self.cv_image.any()):
                self.rate.sleep()
            




if __name__ == '__main__':
    try:
        ackerman = ReverseAckerman()
        ackerman.run()
    except rospy.ROSInterruptException:
        pass
    
    






