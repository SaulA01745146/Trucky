#!/usr/bin/env python


import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int64
from trucky_custom_msgs.msg import ActuatorsState
from ackermann_msgs.msg import AckermannDriveStamped

bridge = CvBridge()


class ReverseAckerman:
    def __init__(self):

        
        rospy.init_node('main_node')

        self.bridge = CvBridge()
        self.cv_image = None
        self.ackerman_msg = AckermannDriveStamped()

        self.rate = rospy.Rate(50)  # Rate of 50 Hz
        self.msg_speed = 0.0
        self.msg_angle = 0.0
        self.msg_traffic_light = ""
        #rospy.init_node('reverseAckerman')
        self.traffic_light_sub = rospy.Subscriber('/traffic_light_state',String, self.traffic_light_callback)
        self.actual_speed_sub = rospy.Subscriber('/actual_speed', Float64,self.speed_callback)
        self.actual_angle_sub = rospy.Subscriber('/actual_angle', Float64, self.angle_callback)
        
    def angle_callback(self,msg):
        self.msg_speed = msg.data
        
    def speed_callback(self,msg):
        self.msg_speed = msg.data
        

    def traffic_light_callback(self, msg):
        self.msg_traffic_light = msg.data
        




    def run(self):
        while not rospy.is_shutdown():
            if(self.cv_image is not None and self.cv_image.any()):
                if(self.msg_traffic_light == "green and red"):
                    print("se detiene y gira las ruedas")
                elif(self.msg_traffic_light =="nothing"):
                    print("usa la velocidad y angulo de las teclas mensaje")
                elif(self.msg_traffic_light == "green"):
                    print("gira y mantiene la velocidad del mensaje")
                elif(self.msg_traffic_light == "red"):
                    print("se detiene y el angulo deja el del mensaje")
                else:
                    print("error")
                self.rate.sleep()
            




if __name__ == '__main__':
    try:
        ackerman = ReverseAckerman()
        ackerman.run()
    except rospy.ROSInterruptException:
        pass
    
    






