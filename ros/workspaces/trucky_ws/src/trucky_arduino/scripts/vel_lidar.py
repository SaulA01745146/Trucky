#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
from std_msgs.msg import String 
from trucky_custom_msgs.msg import ActuatorsState

laserScan = " "

def calculate_speed(steering_angle):
    PWM_no_line = 0  # PWM cuando no se detecta linea 
    PWM_min = 1550      # PWM minimo si hay linea 
    PWM_max = 1600      # PWM maximo si detecta linea
    
    if steering_angle == 0:
        return PWM_no_line  # No se detectaron lineas 
    else:
        # Calculo de PWM
              
        return (max(PWM_min, int((steering_angle / 1.5) * ((PWM_max - PWM_min) + PWM_min)/PWM_min)))
    
def callback_laserScan(data):
    global laserScan
    laserScan = data.data

def callback_steering_angle(data):
    global laserScan
    steering_angle = data.data
    motor_speed = calculate_speed(steering_angle)

    actuators_msg = ActuatorsState()
    if laserScan == "obstacle":
        actuators_msg.servo_pwm_high_time = 1400
        actuators_msg.motor_pwm_high_time = 0
        actuators_msg.output_mode = " "
    else:
        if steering_angle < 1350:
            motor_speed =1527
        elif steering_angle > 1465:
            motor_speed =1520
             
        actuators_msg.servo_pwm_high_time = steering_angle
        actuators_msg.motor_pwm_high_time = motor_speed
        actuators_msg.output_mode = " "

    #print(actuators_msg)
    pub_actuators.publish(actuators_msg)

def motor_speed_controller():
    rospy.init_node('vel_Control', anonymous=True)
    rospy.Subscriber("/steering_angle", Int64, callback_steering_angle)
    rospy.Subscriber("/lidar_scan", String, callback_laserScan)
    rospy.spin()

if __name__ == '__main__':
    try:
        pub_actuators = rospy.Publisher("/actuators_cmd", ActuatorsState, queue_size=10)
        motor_speed_controller()
        print("NODO DE VELOCIDAD CORRIENDO...")
    except rospy.ROSInterruptException:
        pass

