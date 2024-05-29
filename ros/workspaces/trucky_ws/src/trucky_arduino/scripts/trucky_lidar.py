#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

robot_view = {
    'front': 0.0,
    'front_right': 0.0,
    'right': 0.0,
    'back': 0.0,
    'left': 0.0,
    'front_left': 0.0
}

min_key = 'front'
min_value = 12

def scan_callback(data):
    global robot_view, min_value, min_key

    corrected_ranges = []

    for range_value in data.ranges:
        if math.isinf(range_value):
            corrected_ranges.append(15)
        elif math.isnan(range_value):
            corrected_ranges.append(0)
        else:
            corrected_ranges.append(range_value)

    data.ranges = corrected_ranges
    robot_view = {
        'front': min(min(data.ranges[0:40]), min(data.ranges[315:359])),
        'front_right': min(data.ranges[200:244]),
        'right': min(data.ranges[245:289]),
        'back': min(data.ranges[160:200]),
        'left': min(data.ranges[70:114]),
        'front_left': min(data.ranges[115:159]),
    }
    min_key = min(robot_view, key=robot_view.get)
    min_value = robot_view[min(robot_view, key=robot_view.get)]
    #print("min_key: ", min_key, "min_value: ", min_value)

def stop():
    print("Hay obstaculo bb")
    pub_lidar_scan.publish(String("obstacle"))

def check_obstacle(event):
    print("CHECK OBSTACLE")
    if robot_view['front'] <= 0.5:
        print("stop")
        stop()
    else:
        pub_lidar_scan.publish(String("go"))
        print("nada")

if __name__ == '__main__':
    rospy.init_node('lidar_trucky', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    
    pub_lidar_scan = rospy.Publisher("/lidar_scan", String, queue_size=10)
    rospy.Timer(rospy.Duration(1), check_obstacle)
  
    rospy.spin()
