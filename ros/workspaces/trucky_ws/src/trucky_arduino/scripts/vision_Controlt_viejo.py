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
        return 0  # Valor predeterminado si no se detectan las lineas.
    
def lanes_error(img, left_lines,right_lines):
    height, width, _ = img.shape
    m_pix = 3.7 / 650 #Metros por pixel
    middle_image = width/2
    y_base = height
    left_f = np.polyfit(left_lines[:, 1], left_lines[:, 0], 2)
    right_f = np.polyfit(right_lines[:, 1], right_lines[:, 0], 2)

    left_lb = left_f[0] * y_base**2 + left_f[1]* y_base + left_f[2]
    right_lb = right_f[0] * y_base**2 + right_f[1] * y_base + right_f[2]

    centerLane = (left_lb + right_lb) / 2
    error = (middle_image - centerLane) * m_pix * 100

    return int(error)

def steering_angle(error):
    center_threshold = 20  # Umbral para centrar el robot, cambiar si se encuentra desalineado
    steering_gain = 0.01   # Ganancia del angulo de giro

    if abs(error) < center_threshold:
        return 1400  # Robot centrado, no hay angulo de giro
    sa = steering_gain * error  # Angulo de giro (Controlador proporcional)
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
	    	
            '''
            roi_height = 200  # Altura de la ROI
            roi_width = 1400   # Ancho de la ROI
            x = int((width - roi_width) / 2)  # Coordenada x ROI
            y = height - roi_height - 75      # Coordenada y ROI

            roi = image[y:y+roi_height, x:x+roi_width]
            '''

            # ************** TRAPECIO ****************************
            #PUNTOS DEL TRAPECIO 
            vertices = np.array([[(200,height),(width -200, height),
                                  (width - 1400, height //2 + 70),
                                  (1400, height //2 +70)]], dtype = np.int32)
            
            #TRAPECIO
            mascara = np.zeros_like(image)
            cv2.fillPoly(mascara, vertices, (255, 255, 255))

            imMask = cv2.bitwise_and(image, mascara) # Mascara en la imagen 

            gray = cv2.cvtColor(imMask, cv2.COLOR_BGR2GRAY)
            medianblur = cv2.medianBlur(gray, 1)
            
            thMax = 255
            thMin = 110 #MODIFICAR SI NO DETECTA LINEA 
            _, thresh = cv2.threshold(medianblur, thMin, thMax, cv2.THRESH_BINARY)
            th2 = cv2.adaptiveThreshold(medianblur, thMax, cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,17,10)
            th3 = cv2.adaptiveThreshold(medianblur, thMax, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,25,7)

            #edges_v = cv2.filter2D(thresh, -1, kernel_v)
            #edges_h = cv2.filter2D(thresh, -1, kernel_h)

            kernel = np.ones((7,7), np.uint8)
            th4 =cv2.morphologyEx(th3, cv2.MORPH_CLOSE, kernel)
            edges = cv2.Canny(th4, 50, 150)
            #erode = cv2.bitwise_not(thresh, edges)
            #kernel = np.ones((5,5), np.uint8)
            #dil = cv2.dilate(thresh,kernel, iterations=1)
            #bina = cv2.bitwise_not(erode)
            #contours, _ = cv2.findContours(bina, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            
            lines = cv2.HoughLines(edges, 1, np.pi/180, 110)
            if lines is not None:
                left_lines = []
                right_lines = []
                for line in lines:
                    rho, theta = line[0]
                    x1, y1, x2, y2 = int(rho * np.cos(theta)), int(rho * np.sin(theta)), \
                                     int((rho - 1000 * np.sin(theta)) * np.cos(theta)), int((rho + 1000 * np.cos(theta)) * np.sin(theta))
                    if x1 < width / 2:
                        left_lines.append([x1, y1])
                        print("left", left_lines[0]) 
                    else:
                        right_lines.append([x1, y1])
                        print("right", right_lines[0]) 

                if len(left_lines) > 0 and len(right_lines) > 0:
                    left_lines = np.array(left_lines)
                    right_lines = np.array(right_lines)
                    error = lanes_error(image, left_lines, right_lines) #AQUI SE LLAMA A LA FUNCION LANE_ERROR
                    print("Error:",error)
                    print("steering_angle", steering_angle(error))
                    pub_steering_angle.publish(Int64(steering_angle(error)))
                    

                else:
                    print("NO HAY LINEAS, NI IZQUIERDA, NI DERECHA")
                    pub_steering_angle.publish(Int64(0))
            else:
                print("No hay lineas")
                pub_steering_angle.publish(Int64(0))
                '''
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
'''
            
            try:
                edges_msg=bridge.cv2_to_imgmsg(edges, "mono8")
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


