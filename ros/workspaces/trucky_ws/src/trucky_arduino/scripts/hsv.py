#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 
image = np.zeros((480, 640, 3), dtype=np.uint8)
height = 480
width = 640

def callback_image(data):
    global image
    
    try:
        image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    # create trackbars for color change
    cv2.createTrackbar('lowH','image',0,179,nothing)
    cv2.createTrackbar('highH','image',179,179,nothing)
    
    cv2.createTrackbar('lowS','image',0,255,nothing)
    cv2.createTrackbar('highS','image',255,255,nothing)
    
    cv2.createTrackbar('lowV','image',0,255,nothing)
    cv2.createTrackbar('highV','image',255,255,nothing)

def nothing(x):
    pass

# Open the camera
#cap = cv2.VideoCapture(0) 
#ret, frame = cap.read()
#frame = cv2.imread('hsv_g.jpg') 

def main():
    global image
    cv2.namedWindow('image')
 
    
    
    rospy.init_node("hsv")
    loop_rate = rospy.Rate(rospy.get_param("~node_rate", 100))

    rospy.Subscriber("camera/color/image_raw", Image, callback_image)
    pubimage = rospy.Publisher("/hsv", Image, queue_size=10)

    print("Semaforo Node is Running")

    try:
        while not rospy.is_shutdown():
            height, width, _ = image.shape

 
            ilowH = cv2.getTrackbarPos('lowH', 'image')
            ihighH = cv2.getTrackbarPos('highH', 'image')
            ilowS = cv2.getTrackbarPos('lowS', 'image')
            ihighS = cv2.getTrackbarPos('highS', 'image')
            ilowV = cv2.getTrackbarPos('lowV', 'image')
            ihighV = cv2.getTrackbarPos('highV', 'image')
            
            # convert color to hsv because it is easy to track colors in this color model
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower_hsv = np.array([ilowH, ilowS, ilowV])
            higher_hsv = np.array([ihighH, ihighS, ihighV])
            # Apply the cv2.inrange method to create a mask
            mask = cv2.inRange(hsv, lower_hsv, higher_hsv)
            # Apply the mask on the image to extract the original color
            image = cv2.bitwise_and(image, image, mask=mask)
            
            try:
                image_message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
                pubimage.publish(image_message)
            except CvBridgeError as e:
                print(e)

            loop_rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    bridge = CvBridge()
    main()

