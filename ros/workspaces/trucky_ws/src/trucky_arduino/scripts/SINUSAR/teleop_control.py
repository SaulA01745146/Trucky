#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int64
from trucky_custom_msgs.msg import ActuatorsState
from ackermann_msgs.msg import AckermannDriveStamped

class ReverseAckerman:
    def __init__(self):
        rospy.init_node('reverseAckerman')
        self.pub = rospy.Publisher("/actuators_cmd", ActuatorsState, queue_size=10)
        self.sub = rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, self.ackermann_callback)
        self.rate = rospy.Rate(10)  # Rate of 50 Hz

    def ackermann_callback(self, msg):
        actuator_state = ActuatorsState()
        actuator_state.motor_pwm_high_time = int((msg.drive.speed / 2.6) * (1700 - 1480) + 1480)
        print(actuator_state.motor_pwm_high_time)
        actuator_state.servo_pwm_high_time = int((msg.drive.steering_angle / 0.3) * (1850 - 1550) + 1250)
        print(actuator_state.servo_pwm_high_time)
        actuator_state.output_mode = ""
        self.pub.publish(actuator_state)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        reverse_ackerman = ReverseAckerman()
        reverse_ackerman.main()
    except rospy.ROSInterruptException:
        pass
                
