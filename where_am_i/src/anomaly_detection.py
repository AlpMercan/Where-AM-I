#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import numpy as np
from The_kidnapper_Algorithm import *
from geometry_msgs.msg import Twist
import subprocess

class AnomalyDetector:
    def __init__(self):
        rospy.init_node('imu_anomaly_detector')
        self.First_start=True
        self.imu_subscriber = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.acceleration_threshold = 10
        self.angular_velocity_threshold = 1 

    def imu_callback(self, data):
        
        linear_acceleration = np.sqrt(data.linear_acceleration.x**2 +
                                      data.linear_acceleration.y**2 +
                                      data.linear_acceleration.z**2)

        angular_velocity = np.sqrt(data.angular_velocity.x**2 +
                                   data.angular_velocity.y**2 +
                                   data.angular_velocity.z**2)

        
        if (linear_acceleration > self.acceleration_threshold or
            angular_velocity > self.angular_velocity_threshold or self.First_start==True):
            rospy.loginfo("Anomaly detected in IMU data! Activating kidnapper algorithm.")
            pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
            stop_movement = Twist()
            stop_movement.linear.x = 0
            stop_movement.angular.z = 0
            pub.publish(stop_movement)
            self.First_start=False
            self.activate_kidnapper_algorithm()
            
        else:
            rospy.loginfo("IMU data within normal range.")

    def activate_kidnapper_algorithm(self):
        rospy.loginfo("Activating kidnapper algorithm as a separate process.")
        subprocess.run(["python3", "/home/alp/catkin_ws/src/nerdeyim/turtlebot3/where_am_i/src/The_kidnapper_Algorithm.py"])  ####change it to where your python script is


def main():
    anomaly_detector = AnomalyDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
