#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

image=np.zeros((480,640,3),dtype=np.uint8)
height=480
width=640

def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi), (2 * np.pi))
    if result < 0:
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
        total_rho = 0
        num_lines = len(lines)
        for line in lines:
            rho, _ = line[0]
            total_rho += rho
        avg_rho = total_rho / num_lines
        return int(avg_rho)
    else:
        return 200  # Valor predeterminado si no se detectan las lineas.

def steering_angle(x_mid):
    center_threshold = 20  # Umbral para centrar el robot
    steering_gain = 0.01   # Ganancia del angulo de giro

    eg = 700 - x_mid  # Centro de la imagen
    if abs(eg) < center_threshold:
        return 1400  # Robot centrado, no hay angulo de giro
    sa = steering_gain * eg  # Angulo de giro (Controlador proporcional)
    sa_normalized = sa / center_threshold  # Se normaliza de -1 a 1
    pwm_range = 600  # Rango de PWM del servomotor (1600-1000)
    center_pwm = 1400 
    steering = center_pwm + (sa_normalized * pwm_range / 2)

    # Valor dentro del rango [1000, 1600]
    return int(max(min(steering, 1600), 1000))

def main():
    global image
    rospy.init_node("vision_control")
    loop_rate = rospy.Rate(rospy.get_param("~node_rate", 100))

    rospy.Subscriber("camera/color/image_raw", Image, callback_image)
    pub_steering_angle = rospy.Publisher("/steering_angle", Int64, queue_size=10)
    pubimage = rospy.Publisher("/imagen", Image, queue_size=10)

    print("Vision Node is Running")

    try:
        while not rospy.is_shutdown():
            height, width, _ = image.shape
            roi_height = 200  # Altura de la ROI
            roi_width = 1400   # Ancho de la ROI
            x = int((width - roi_width) / 2)  # Coordenada x ROI
            y = height - roi_height - 75      # Coordenada y ROI

            roi = image[y:y+roi_height, x:x+roi_width]

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            edges_v = cv2.filter2D(blurred, -1, kernel_v)
            edges_h = cv2.filter2D(blurred, -1, kernel_h)

            edges = cv2.addWeighted(edges_v, 0.5, edges_h, 0.5, 0)
            #cv2.imshow("edges",edges)
            
            
            thMax = 255
            thMin = 10 #MODIFICAR SI NO DETECTA LINEA 
            _, thresh = cv2.threshold(edges, thMin, thMax, cv2.THRESH_BINARY)
            

            lines = cv2.HoughLines(thresh, 1, np.pi/180, 77)

            if lines is not None:
                line_positions = []
                for line in lines:
                    rho, theta = line[0]
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    x_mid = int(x0 + target_distance(lines) * (-b))
                    line_positions.append(x_mid)

                if len(line_positions) >= 2:
                    line_positions.sort(key=lambda pos: abs(pos - (roi_width // 2)))

                    closest_line_1 = line_positions[0]
                    closest_line_2 = line_positions[1]

                    center_between_lines = (closest_line_1 + closest_line_2) // 2
                    distance_to_center = center_between_lines - (roi_width // 2)
                    
                    print("distance_to_center", distance_to_center)

                    if distance_to_center >  0:
                        print("Mover a la derecha para mantener el centro")
                    elif distance_to_center < 0:
                        print("Mover a la izquierda para mantener el centro")
                    else:
                        print("Manteniendo el centro")
                        
                    print(steering_angle(center_between_lines))

                    pub_steering_angle.publish(Int64(steering_angle(center_between_lines)))

                elif len(line_positions) == 1:
                    closest_line = line_positions[0]
                    distance_to_line = abs(closest_line - (roi_width // 2))
                    distance_in_pixels = 5  # 1 cm corresponde a 5 pixeles

                    if distance_to_line > distance_in_pixels:
                        if closest_line < roi_width // 2:
                            print("Mover a la izquierda para mantener 1 cm")
                        else:
                            print("Mover a la derecha para mantener 1 cm")
                    else:
                        print("Manteniendo distancia de 1 cm")
                    

                    pub_steering_angle.publish(Int64(steering_angle(closest_line)))
            else:
                print("No se detectaron lineas")
                pub_steering_angle.publish(Int64(0))
            try:
                edges_msg=bridge.cv2_to_imgmsg(thresh, "mono8")
                pubimage.publish(edges_msg)
            except CvBridgeError as e:
                print(e)
            
                
            #cv2.imshow("camino",roi)
            

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    kernel_v = np.array([[1, 0, -1],
                         [1, 0, -1],
                         [1, 0, -1]])

    kernel_h = np.array([[1, 1, 1],
                         [0, 0, 0],
                         [-1, -1, -1]])
    bridge = CvBridge()
    main()

