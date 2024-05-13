#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class VideoCamera:
    def __init__(self):
        rospy.init_node('video_recorder', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.out = None
        self.frame_count = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.out is None:
            height, width, _ = cv_image.shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            output_path = os.path.join(os.getcwd(), 'output2.mp4')
            self.out = cv2.VideoWriter(output_path, fourcc, 20.0, (width, height))

        self.out.write(cv_image)
        self.frame_count += 1

if __name__ == '__main__':
    try:
        video_camera = VideoCamera()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

