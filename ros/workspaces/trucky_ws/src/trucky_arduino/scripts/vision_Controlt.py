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

def ajuste_luz(gray):
    luz_mean = np.mean(gray)
    if luz_mean < 50:
        #print("<50")
        thMax = 255
        thMin = 90
    elif luz_mean < 100:
        print("<100")
        thMax = 255
        thMin = 90
    elif luz_mean < 150:
        print("<150")
        thMax = 255
        thMin = 80
    else:
        print("<e")
        thMax = 255
        thMin = 70
    return thMax, thMin

def ajuste_canny(gray):
    intensidad = np.mean(gray)
    if intensidad < 50:
        baja = 30
        alta = 90
    elif intensidad < 100:
        baja = 50
        alta = 150
    elif intensidad < 150:
        baja = 70
        alta = 200
    else:
        baja = 90
        alta = 250
    return baja, alta

def filtro_de_lineas(linesP, width):
    left_lines = []
    right_lines = []
    for line in linesP:
        x1, y1, x2, y2 = line[0]
        slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0
        if abs(slope) < 0.3:
            continue
        if slope < 0 and x1 < width / 2 and x2 < width / 2:
            left_lines.append([x1, y1, x2, y2])
        elif slope > 0 and x1 > width / 2 and x2 > width / 2:
            right_lines.append([x1, y1, x2, y2])
    return left_lines, right_lines

def dib_lineas(image, lines, color=(0, 255, 0), thickness=2):
    for line in lines:
        x1, y1, x2, y2 = line
        cv2.line(image, (x1, y1), (x2, y2), color, thickness)

def lanes_error(img, left_lines, right_lines):
    height, width, _ = img.shape
    m_pix = 3.7 / 650  # Metros por pixel
    middle_image = width / 2

    if len(left_lines) == 0 and len(right_lines) > 0:
        right_line = np.mean(right_lines, axis=0)
        error = middle_image - (right_line[0] + right_line[2]) / 2  # Error en pixeles
        target_distance = 30  # Distancia objetivo del borde derecho en cm
        error_cm = error * m_pix  # Convertir error a cm
        error_adjusted = error_cm - target_distance  # Ajustar error para mantener la distancia objetivo
        return int(error_adjusted)

    elif len(left_lines) > 0 and len(right_lines) == 0:
        left_line = np.mean(left_lines, axis=0)
        error = (left_line[0] + left_line[2]) / 2 - middle_image  # Error en pixeles
        target_distance = 30  # Distancia objetivo del borde izquierdo en cm
        error_cm = error * m_pix  # Convertir error a cm
        error_adjusted = target_distance - error_cm  # Ajustar error para mantener la distancia objetivo
        return int(error_adjusted)

    elif len(left_lines) > 0 and len(right_lines) > 0:
        left_lines = np.array(left_lines)
        right_lines = np.array(right_lines)
        left_f = np.polyfit(left_lines[:, 1], left_lines[:, 0], 2)
        right_f = np.polyfit(right_lines[:, 1], right_lines[:, 0], 2)

        left_lb = left_f[0] * height**2 + left_f[1] * height + left_f[2]
        right_lb = right_f[0] * height**2 + right_f[1] * height + right_f[2]

        center_lane = (left_lb + right_lb) / 2
        error = middle_image - center_lane
        print("center lane: ", center_lane, "middleimage: ",middle_image)

        return int(error)

    else:
        return 0

def steering_angle(error):
    center_threshold = 20  # Umbral para centrar el robot
    steering_gain = 0.1   # Ganancia del angulo de giro

    if abs(error) < center_threshold:
        return 1400  # Robot centrado
    sa = steering_gain * error  # angulo de giro (Controlador proporcional)
    sa_normalized = sa / center_threshold  # Normalizar de -1 a 1
    pwm_range = 600  # Rango de PWM del servomotor (1600-1000)
    center_pwm = 1400 
    steering = center_pwm + (sa_normalized * pwm_range / 2)

    # Valor dentro del rango [1000, 1600]
    return int(max(min(steering, 1600), 1000))

def callback_image(data):
    global image
    try:
        image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

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

            lower_offset = 600
            upper_offset = 50
            trap_height = height // 1.5
            vertices = np.array([[(width // 4 - upper_offset, height),
                                (3 * width // 4 + upper_offset, height),
                                (3 * width // 4 - 100 + lower_offset, trap_height),
                                (width // 4 + 100 - lower_offset, trap_height)]], dtype=np.int32)
            
            mascara = np.zeros_like(image)
            cv2.fillPoly(mascara, vertices, (255, 255, 255))
            imMask = cv2.bitwise_and(image, mascara)

            gray = cv2.cvtColor(imMask, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            gray = clahe.apply(gray)
            se = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))
            bg = cv2.morphologyEx(gray, cv2.MORPH_DILATE, se)
            gradient = cv2.morphologyEx(gray, cv2.MORPH_GRADIENT, bg)
            gray = cv2.divide(gray, gradient, scale=255)

            medianblur = cv2.GaussianBlur(gray, (5, 5), sigmaX=33, sigmaY=33)
            thMax, thMin = ajuste_luz(gray)
            _, thresh = cv2.threshold(medianblur, thMin, thMax, cv2.THRESH_BINARY)

            sobelx = cv2.Sobel(thresh, cv2.CV_64F, 1, 0, ksize=3)
            sobely = cv2.Sobel(thresh, cv2.CV_64F, 0, 1, ksize=3)
            sobel_comb = cv2.sqrt(sobelx**2 + sobely**2)
            sobel_comb = cv2.convertScaleAbs(sobel_comb)

            baja, alta = ajuste_canny(gray)
            edges = cv2.Canny(sobel_comb, baja, alta)
            kernel = np.ones((9, 9), np.uint8)
            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contorno = cv2.drawContours(np.zeros_like(edges), contours, -1, (255, 255, 255), 1)

            rho = 2
            theta = np.pi / 100
            threshold_lines = 50
            min_line_length = 50
            max_line_gap = 50

            linesP = cv2.HoughLinesP(contorno, rho, theta, threshold_lines, np.array([]), min_line_length, max_line_gap)
            
            if linesP is not None:
                left_lines, right_lines = filtro_de_lineas(linesP, width)
                if len(left_lines) > 0 or len(right_lines) > 0:
                    if len(left_lines) > 0:
                        dib_lineas(image, left_lines, color=(0, 0, 255), thickness=2)
                    if len(right_lines) > 0:
                        dib_lineas(image, right_lines, color=(0, 255, 0), thickness=2)
                    
                    error = lanes_error(image, left_lines, right_lines)
                    steering = steering_angle(error)
                    print("error: ",error)
                    print("Steering Angle:", steering)
                                        
                    if steering != 1445:
                        pub_steering_angle.publish(Int64(steering))
                    else:
                        pub_steering_angle.publish(Int64(0))

                    

                else:
                    print("NO HAY LINEAS, NI IZQUIERDA, NI DERECHA")
                    pub_steering_angle.publish(Int64(0))
            else:
                print("No hay lineas")
                pub_steering_angle.publish(Int64(0))

            try:
                edges_msg = bridge.cv2_to_imgmsg(image, "rgb8")
                pubimage.publish(edges_msg)
            except CvBridgeError as e:
                print(e)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    bridge = CvBridge()
    main()



