#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

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
        global image
        try:
            image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

def target_distance(lines):
    if lines is not None:
        #print('veo linea')
        total_rho = 0
        total_theta = 0
        num_lines = len(lines)
        for line in lines:
            rho, theta = line[0]
            total_rho += rho
            avg_rho = total_rho / num_lines
            return int(avg_rho)
    else:
        return 200 #Valor predeterminado si no se detectan las lineas.
    
def steering_angle(x_mid):
    center_threshold = 20 #THRESHOLDA PARA CENTRAR EL ROBOT
    steering_gain = 0.01 #Ganancia del angulo de giro 

    eg = 200 - x_mid #Centro de la imagen es de 200 pixeles (Modificar de lo contrario)
    if abs(eg) < center_threshold:
        return 1400 #Si el robot esta centrado, no debe existir angulo de giro
    sa = steering_gain * eg  #steering_angle (Controlador proporcional)
    sa_normalized = sa/center_threshold #Se normaliza de -1 a 1
    pwm_range = 600 #Rango de PWM del servomotor (1600-1000)
    center_pwm = 1400 
    steering = center_pwm + (sa_normalized*pwm_range/2)

    return int(max(min(steering, 1600), 1000)) #VALOR DENTRO DEL RANGO

def main():
    rospy.init_node("vision_control")
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    rospy.Subscriber("camera/color/image_raw",Image, callback_image)
    pub_steering_angle = rospy.Publisher("/steering_angle", Int64 , queue_size=10)

    print("Vision Node  is Running")

    try:
        while not rospy.is_shutdown():
            height, width, _ = image.shape
            roi_height = 100  # Altura de la ROI
            roi_width = 400   # Ancho de la ROI
            x = int((width - roi_width) / 2)  # Coordenada x para el centro de la ROI
            y = height - roi_height -30          # Coordenada y para el centro de la ROI

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

            td = target_distance(lines) #Target_distance

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
                #y0 = b * avg_rho
                x_mid = int(x0 + td * (-b)) 
                #print('x_mid', x_mid)
                #y_mid = int(y0 + td * (a))
                #print('y_mid', y_mid)
                pub_steering_angle.publish(Int64(wrap_to_Pi(steering_angle(x_mid)))) #Publica angulo de giro

                   
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node

if __name__=='__main__':

    kernel_v = np.array([[1, 0, -1],
                     [1, 0, -1],
                     [1, 0, -1]])

    kernel_h = np.array([[1, 1, 1],
                        [0, 0, 0],
                        [-1, -1, -1]])
    main()
    

