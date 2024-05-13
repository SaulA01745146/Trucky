#!/usr/bin/env python

import tf, rospy
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, Pose, Vector3
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import time


class PosePuzzlebot:
    
    def __init__(self):
        self.msg = Odometry()
        rospy.init_node('puzzlebot_bug0_alg') 

        #Variables needed for this algo
        self.goal = np.array([-3, 2])  # goal coordinates
        self.goal_list = ([-3,0.5],[-1, 1.95],[0,0])
        self.goal = self.goal_list[0]
        self.hit_obstacle = False
        self.hip = 0
        self.ca = 0
        self.goal_thresh = 0.20
        self.angular_speed = 0.5 # radians per second
        self.following_wall = False
        self.linear_speed = 0.2  # robot linear speed
        self.angular_speed = 0.5  # robot angular speed

        self.cmd_msg = AckermannDriveStamped()
        

        self.pub = rospy.Publisher('/ackerman_cmd', Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.scan = None
        
        self.rate = rospy.Rate(50)


    def scan_callback(self, msg):
        #Callback for laser scan
        self.scan = msg

    
    def distance(self, p1, p2):
        return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def move_to_goal(self):
        """First function for moving to a goal direction"""
        dist_in_front = self.scan.ranges[0]
        print(dist_in_front)
        print("im in move to goal")
        #If the first range in laser scan(dist in front)this force a rotation for 5 seconds and goes straight for 1 second 
        if dist_in_front < 0.29:
            self.cmd_msg.drive.speed = 0.0
            self.cmd_msg.drive.steering_angle = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.sleep(1)  # Allow some time to stop

    """Functions used to find the angle of the wall"""
    def find_wall_direction(self, scan):
        #--------------------------------------------------------------
        # Calculate the number of ranges in the laser scan message
        num_ranges = len(scan.ranges)
        # print("num ranges: ", num_ranges)

        # Calculate the angle range of the laser scanner in radians
        angle_range = scan.angle_max - scan.angle_min

        # Calculate the angle increment of the laser scanner in radians
        angle_increment = angle_range / (num_ranges - 1)

        # Convert the range values to radians
        angles = [scan.angle_min + i * angle_increment for i in range(num_ranges)]

        end_angle = angles[1000]
        start_angle = np.pi/2
        # print("angles: ", angles)
        # print("end angle:", end_angle)

        hip = self.get_distance_in_sector(scan, start_angle, end_angle)
        # print("hip: ", hip)
        self.hip = hip
        ca = scan.ranges[860]
        # print("ca: ", ca)
        # print("angel ca: ", angles[860])
        
        angulo_ab = np.degrees(abs(angles[1000]) - abs(angles[860]))
        angulo_ab = abs(angulo_ab)

        # print("angulo_ab:", angulo_ab)

        co = np.sqrt((ca**2 + hip**2) - (2*ca*hip*np.cos(np.radians(angulo_ab))))

        alpha = np.arcsin((ca-hip*np.cos(np.radians(angulo_ab)))/co)
        alpha = alpha - 0.23
        # print("alpha :", alpha)

        # alpha = np.degrees(alpha)
        # alpha = alpha

        return alpha

    """Functions used to find the angle of the wall"""
    def range_index(self,scan, angle):
        """Returns the index into the scan ranges that correspond to the angle given (in rad).
        If the angle is out of range, then simply the first (0) or last index is returned, no
        exception is raised.

        Arguments
        ---------
        scan : LaserScan
        angle : float
        Angle w.r.t the x-axis of the robot, which is straight ahead"""

        #--------------------------------------------------------------
        # Your code here
        # print(scan.ranges)

        # Calculate the number of ranges in the laser scan message
        num_ranges = len(scan.ranges)

        # Calculate the angle range of the laser scanner in radians
        angle_range = scan.angle_max - scan.angle_min

        # Calculate the angle increment of the laser scanner in radians
        angle_increment = angle_range / (num_ranges - 1)

        # Convert the range values to radians
        angles = [scan.angle_min + i * angle_increment for i in range(num_ranges)]

        angles = [round(x,6) for x in angles]
        
        angle = round(angle, 6)

        # print(angles)
        # print(angle)
        # print(num_ranges)
        tolerance = 0.01

        if angle == 0:
            return len(angles)/2
        if angle < scan.angle_min:
            return 0
        if angle > scan.angle_max:
            return len(angles)-1
        
        # if abs(angle - np.pi/2)< tolerance:
        #     return len(angles)

        for i in range(num_ranges):
            
            if abs(angle - angles[i])< tolerance:
                # print(i)
                return i
        
        return 0
   
    """Functions used to find the angle of the wall"""
    def get_distance_in_sector(self, scan, start_angle, end_angle):
        """Returns the distance in m in the given sector by taking the average of the
        range scans.
        
        Arguments
        ---------
        scan : LaserScan
        start_angle : float
            Start angle of sector, in radians, where 0 is straight ahead.
        end_angle : float
            End angle of sector, in radians, where 0 is straight ahead."""
        
        num_scans = len(scan.ranges)
        

        start_index =  self.range_index(scan, start_angle)
        end_index = self.range_index(scan, end_angle)
        new_list = list(scan.ranges)
        for i in range(start_index, end_index):
            if np.isinf(scan.ranges[i]):
                # print("found and inf")
                new_list[i] = 0
                
        return np.mean(scan.ranges[start_index:end_index])


    def calculate_distance(self):
        # Create vectors for the line connecting the two coordinates and the robot's position
        x1 = 0
        x2 = self.goal[0]
        y1 = 0
        y2 = self.goal[1]
        x_robot = self.msg.pose.pose.position.x
        y_robot = self.msg.pose.pose.position.y

        line_vector = np.array([x2 - x1, y2 - y1])
        robot_vector = np.array([x_robot - x1, y_robot - y1])

        # Calculate the perpendicular distance from the robot's position to the line
        distance = np.linalg.norm(np.cross(line_vector, robot_vector)) / np.linalg.norm(line_vector)

        return distance


    def main(self):

        while not rospy.is_shutdown():
            # print("i dont see a scan")
            if self.scan is not None:
                    # print("i see a scan")
                    self.move_to_goal()

            self.rate.sleep()

if __name__ == "__main__":
    # Inicializacion de clase y ejecucion de codigo
    sq = PosePuzzlebot()
    try:
        sq.main()
    except rospy.ROSInterruptException:
        None
