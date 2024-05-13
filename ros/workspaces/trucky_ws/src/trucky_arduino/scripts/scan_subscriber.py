#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan

def scan_callback(data):

    print("Header frame_id: ",data.header.frame_id)
    print("angle_min: ", data.angle_min)
    print("angle_max: ",data.angle_max)
    print("angle_increment: ",data.angle_increment)
    print("time_increment: ", data.time_increment)
    print("scan_time: ", data.scan_time)
    print("range_min: ", data.range_min)
    print("range_max: ", data.range_max)
    #print(data.ranges)
    corrected_ranges = []

    for range_value in data.ranges:
        if math.isinf(range_value):
            corrected_ranges.append(16)
        elif math.isnan(range_value):
            corrected_ranges.append(0)
        else:
            corrected_ranges.append(range_value)

    data.ranges = corrected_ranges

    print("Tamano: ", len(data.ranges), "Min: ", min(data.ranges), "Max: ", max(data.ranges))
    print("Index min: ", data.ranges.index(min(data.ranges)))
    print("Index max: ", data.ranges.index(max(data.ranges)))
    #print(data.intensities)
    #pass

if __name__=='__main__':
    rospy.init_node('scan_node',anonymous=True)
    rospy.Subscriber("scan", LaserScan, scan_callback)
    rospy.spin()
