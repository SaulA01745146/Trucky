#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge=CvBridge()

image=np.zeros((480,640,3),dtype=np.uint8)
height=480
width=640

def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

def callback_image(data):
        global image, height, width
        try:
     
            image = bridge.imgmsg_to_cv2(data, "bgr8")
            height=data.height
            width=data.width
        except CvBridgeError as e:
            print(e)
        #print((image))
        



# Inicializar la captura de video
#captura = cv2.VideoCapture(0)

kernel_v = np.array([[1, 0, -1],
                     [1, 0, -1],
                     [1, 0, -1]])

kernel_h = np.array([[1, 1, 1],
                     [0, 0, 0],
                     [-1, -1, -1]])

vlin = 0.5 
vang = 0.2  
target_distance = 200  # Distancia objetivo del punto medio


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("vision_lin2")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    image_sub = rospy.Subscriber("camera/color/image_raw",Image, callback_image)

    pub = rospy.Publisher("/actuators_cmd", Int64 , queue_size=100)

    print("Vision Node  is Running")

    try:
        while not rospy.is_shutdown():
            #if image is not None:
            roi_height=100
            roi_width=400
            x=int((width - roi_width) / 2)
            y = height - roi_height -30
            #print(image.shape)
            #print(x,y,roi_height,roi_width)
            roi = image[y:y+roi_height, x:x+roi_width]

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            edges_v = cv2.filter2D(blurred, -1, kernel_v)
            edges_h = cv2.filter2D(blurred, -1, kernel_h)

            edges = cv2.addWeighted(edges_v, 0.5, edges_h, 0.5, 0)

            #cv2.imshow("edges", edges)

            thMax = 255
            thMin = 77
            _, thresh = cv2.threshold(edges, thMin, thMax, cv2.THRESH_BINARY)

            lines = cv2.HoughLines(thresh, 1, np.pi/180, 77)

            if lines is not None:
                #print('veo linea')
                total_rho = 0
                total_theta = 0
                num_lines = len(lines)
                for line in lines:
                    rho, theta = line[0]
                    total_rho += rho
                    total_theta += theta
                avg_rho = total_rho / num_lines
                avg_theta = total_theta / num_lines

                a = np.cos(avg_theta)
                b = np.sin(avg_theta)
                x0 = a * avg_rho
                y0 = b * avg_rho
                x_mid = int(x0 + target_distance * (-b))
                #print('x_mid', x_mid)
                y_mid = int(y0 + target_distance * (a))
                #print('y_mid', y_mid)


                #cv2.line(roi, (x_mid - 10, y_mid), (x_mid + 10, y_mid), (0, 255, 0), 2)
                #cv2.line(roi, (x_mid, y_mid - 10), (x_mid, y_mid + 10), (0, 255, 0), 2)

                #Ajustar estos valores, de acorde a los umbrales de giro del robot
                print(x_mid)
                if x_mid < -180:
                    print("Muy a la izquierda")
                elif 160 <= x_mid < 190:
                    print("Izquierda")
                elif 190 <= x_mid < 210:
                    print("Centro")
                elif -150 <= x_mid < -180:
                    print("Derecha")
                else:
                    print("Muy a la derecha")


            #cv2.imshow("Camino", roi)

            #tecla = cv2.waitKey(100) & 0xFF
            #if tecla == 27: 
            #    break
            #else:
             #s   print("Waiting for image...")

        #captura.release()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node

