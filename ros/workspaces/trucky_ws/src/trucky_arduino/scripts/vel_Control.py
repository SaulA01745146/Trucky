#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from trucky_custom_msgs.msg import ActuatorsState

def calculate_speed(steering_angle):
    motor_high_time_min = 1400 # BLDC Motor Pairing PWM High Time (ms)
    motor_dead_zone_time = 1450 # BLDC Motor Dead Zone PWM High Time (ms)
    motor_high_time_max = 1600 # PWM para salir de la Dead Zone 

    # De ang de giro a PWM 
    if steering_angle == 0:
        return 1450 # Si el angulo de giro es cero, la velocidad esta en la Dead Zone
    elif steering_angle > 0:
        return int((steering_angle / 1.5) * (motor_high_time_max - motor_dead_zone_time) + motor_dead_zone_time)
    else:
        return int((steering_angle / 1.5) * (motor_high_time_min - motor_dead_zone_time) + motor_dead_zone_time)

def callback_steering_angle(data):
    steering_angle = data.data
    motor_speed = calculate_speed(steering_angle)

    actuators_msg = ActuatorsState()
    actuators_msg.servo_pwm_high_time = steering_angle
    actuators_msg.motor_pwm_high_time = motor_speed
    actuators_msg.output_mode = " "

    pub_actuators.publish(actuators_msg)  

def motor_speed_controller():
    rospy.init_node('motor_speed_controller', anonymous=True)
    rospy.Subscriber("/steering_angle", Int64, callback_steering_angle)
    rospy.spin()

if __name__ == '__main__':
    try:
        pub_actuators = rospy.Publisher("/actuators_cmd", Int64, queue_size=10)
        
        motor_speed_controller()
    except rospy.ROSInterruptException:
        pass
