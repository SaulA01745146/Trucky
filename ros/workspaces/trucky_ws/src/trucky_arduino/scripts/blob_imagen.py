#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Cargar la imagen desde un archivo
#image_path = '/home/ambar/Desktop/Imagen2.png'  # Reemplaza con la ruta de tu imagen
#captured_frame = cv2.imread(image_path)

image = np.zeros((480, 640, 3), dtype=np.uint8)
height = 480
width = 640
flag = 0

def callback_image(data):
    global image
    try:
        image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

def main():
    global image
    rospy.init_node("blob_imagen")
    loop_rate = rospy.Rate(rospy.get_param("~node_rate", 100))

    rospy.Subscriber("camera/color/image_raw", Image, callback_image)
    pubimage = rospy.Publisher("/semaforoimage", Image, queue_size=10)
    pub_semaforo_scan = rospy.Publisher("/semaforo", String, queue_size=10)

    print("Semaforo Node is Running")

    try:
        while not rospy.is_shutdown():
            height, width, _ = image.shape
            if image is None:
                print("Error: al abrir el video.")
            else:
                image = image.copy()

                # Convertir a espacio de color HSV
                captured_frame_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                # Definir los valores de la mascara para el verde
                lower_green = np.array([50, 90, 70])
                upper_green = np.array([70, 255, 255])
                mask_green = cv2.inRange(captured_frame_hsv, lower_green, upper_green)
                mask_green = cv2.GaussianBlur(mask_green, (5, 5), 2, 2)
                circles_green = cv2.HoughCircles(mask_green, cv2.HOUGH_GRADIENT, 1, mask_green.shape[0] / 8, param1=100, param2=18, minRadius=10, maxRadius=60)

                # Detectar crculos rojos
                lower_red = np.array([160, 100, 150])
                upper_red = np.array([179, 255, 255])
                mask_red = cv2.inRange(captured_frame_hsv, lower_red, upper_red)
                mask_red = cv2.GaussianBlur(mask_red, (5, 5), 2, 2)
                circles_red = cv2.HoughCircles(mask_red, cv2.HOUGH_GRADIENT, 1, mask_red.shape[0] / 8, param1=100, param2=18, minRadius=10, maxRadius=60)

                # Detectar crculos amarillos
                lower_yellow = (20, 100, 100)
                upper_yellow = (30, 255, 255)
                mask_yellow = cv2.inRange(captured_frame_hsv, lower_yellow, upper_yellow)
                mask_yellow = cv2.GaussianBlur(mask_yellow, (5, 5), 2, 2)
                circles_yellow = cv2.HoughCircles(mask_yellow, cv2.HOUGH_GRADIENT, 1, mask_yellow.shape[0] / 8, param1=100, param2=18, minRadius=10, maxRadius=60)

                # Si hemos extrado un crculo verde, dibujar un contorno
                if circles_green is not None:
                    circles_green = np.round(circles_green[0, :]).astype("int")
                    for (x, y, r) in circles_green:
                        cv2.circle(image, center=(x, y), radius=r, color=(0, 255, 0), thickness=2)
                    print("Green circle detected")
                    pub_semaforo_scan.publish(String("verde"))

                # Si hemos extrado un crculo rojo, dibujar un contorno
                if circles_red is not None:
                    circles_red = np.round(circles_red[0, :]).astype("int")
                    for (x, y, r) in circles_red:
                        cv2.circle(image, center=(x, y), radius=r, color=(0, 0, 255), thickness=2)
                    print("Red circle detected")
                    pub_semaforo_scan.publish(String("rojo"))
                    

                # Si hemos extrado un crculo amarillo, dibujar un contorno
                if circles_yellow is not None:
                    circles_yellow = np.round(circles_yellow[0, :]).astype("int")
                    for (x, y, r) in circles_yellow:
                        cv2.circle(image, center=(x, y), radius=r, color=(0, 255, 255), thickness=2)
                    print("Yellow circle detected")

                # Mostrar el frame resultante
                #cv2.imshow('frame', output_frame)
                #cv2.waitKey(0)
                #cv2.destroyAllWindows()
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
