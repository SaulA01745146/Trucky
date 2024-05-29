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

# Inicializar el filtro de Kalman
kalman = cv2.KalmanFilter(4, 2)
kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1
kalman.errorCovPost = np.eye(4, dtype=np.float32)

def ajuste_luz(gray):
    luz_mean = np.mean(gray)
    if luz_mean < 50:
        thMax = 255
        thMin = 100
    elif luz_mean < 100:
        thMax = 255
        thMin = 90
    elif luz_mean < 150:
        thMax = 255
        thMin = 80
    else:
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
        if abs(slope) < 0.5:
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
        
def calcular_curvatura(lineas):
    if len(lineas) < 2:
        return 0
    
    x = np.array([linea[0] for linea in lineas] + [linea[2] for linea in lineas])
    y = np.array([linea[1] for linea in lineas] + [linea[3] for linea in lineas])
    
    fit = np.polyfit(y, x, 2)
    curvatura = (1 + (2 * fit[0] * y[-1] + fit[1]) ** 2) ** 1.5 / np.abs(2 * fit[0])
    
    return curvatura

def steering_angle(predicted_x, width, curvatura, single_side=None):
    center_threshold = 20  # Umbral para centrar el robot
    min_steering_gain = 0.5  # Ganancia minima del angulo de direccion
    max_steering_gain = 2  # Ganancia maxima del angulo de direccion

    middle_image = width / 2
    target_distance = 100  # Distancia objetivo desde el borde del carril en pixeles

    if single_side == 'left':
        print('predicted_x_sololeft', predicted_x)
        error = middle_image - predicted_x - target_distance

    elif single_side == 'right':
        print('predicted_x_soloright', predicted_x)
        error = middle_image - predicted_x + target_distance
        
    else:
        print('predicted_x', predicted_x)
        error = middle_image - predicted_x

    print("error", error)
    
    # Ajustar la ganancia del angulo de direccion en funcion de la curvatura
    steering_gain = min_steering_gain + (max_steering_gain - min_steering_gain) * min(curvatura / 1000, 1)

    if abs(error) < center_threshold:
        return 1400  # Robot centrado

    elif error < 0:  # Prediccion a la derecha
        sa = 1350 - steering_gain * abs(error)
        sa = max(sa, 1000)  # Asegurar que este dentro del rango [1000, 1400]
        sa=round(sa)
        print("derecha", sa)
    else:  # Prediccion a la izquierda
        sa = 1480 + steering_gain * abs(error)
        sa = min(sa, 1800)  # Asegurar que este dentro del rango [1400, 1800]
        sa=round(sa)
        print("izquierda", sa)

    return int(sa)

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
                single_side = None
                if len(left_lines) > 0 or len(right_lines) > 0:
                    if len(left_lines) > 0:
                        dib_lineas(image, left_lines, color=(0, 0, 255), thickness=2)
                    if len(right_lines) > 0:
                        dib_lineas(image, right_lines, color=(0, 255, 0), thickness=2)
                    
                    # Punto medio de la linea para el filtro de Kalman
                    if len(left_lines) > 0 and len(right_lines) > 0:
                        left_x1, left_y1, left_x2, left_y2 = left_lines[0]
                        right_x1, right_y1, right_x2, right_y2 = right_lines[0]
                        mid_x = (left_x2 + right_x1) / 2
                        mid_y = (left_y2 + right_y1) / 2
                    elif len(left_lines) > 0:
                        left_x1, left_y1, left_x2, left_y2 = left_lines[0]
                        mid_x = left_x2
                        mid_y = left_y2
                        single_side = 'left'
                    else:
                        right_x1, right_y1, right_x2, right_y2 = right_lines[0]
                        mid_x = right_x1
                        mid_y = right_y1
                        single_side = 'right'
                    
                    # Filtro de Kalman
                    kalman.correct(np.array([[np.float32(mid_x)], [np.float32(mid_y)]]))
                    predicted = kalman.predict()
                    predicted_x, predicted_y = predicted[0][0], predicted[1][0]
                    cv2.circle(image, (predicted_x, predicted_y), 10, (0, 255, 255), -1)

                    # Calcular curvatura
                    curvatura = calcular_curvatura(left_lines + right_lines)
                    
                    # Calcular angulo de direccion
                    sa = steering_angle(predicted_x, width, curvatura, single_side)
                    #print(sa)

                    if sa != 1445:
                        pub_steering_angle.publish(Int64(sa))
                        print("sa_pub: ", sa)
                    else:
                        pub_steering_angle.publish(Int64(0))

            # Publicar imagen procesada
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
