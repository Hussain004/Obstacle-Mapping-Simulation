#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time
from math import cos, sin, radians
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt


class my_turtlebot3_map:


    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        rospy.init_node('turtlebot3_controller')
        self.minimum_distance = float(input("Enter Minimum distance to maintain: "))
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.call_back)
        rospy.Subscriber('/scan', LaserScan, self.scan_call_back)
        self.rate = rospy.Rate(25)
        self.scan_ranges = None


    def scan_call_back(self, msg):
        self.scan_ranges = msg.ranges


    def call_back(self, data):
        self.x = round(data.pose.pose.position.x, 4)
        self.y = round(data.pose.pose.position.y, 4)
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw_rad = euler_from_quaternion(orientation_list)
        self.theta = yaw_rad


    def move_minimum_distance(self):
        vel_msg = Twist()
        while self.scan_ranges[0] > self.minimum_distance:
            vel_msg.linear.x = 0.1
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()
        while self.theta < radians(90):
            vel_msg.angular.z = radians(20)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()


    def rotate_goal(self, goal):
        des_angle = np.argmin(self.scan_ranges)
        error = des_angle - goal
        vel_msg = Twist()
        while abs(error) > 3:
            vel_msg.angular.z = radians(error)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            des_angle = np.argmin(self.scan_ranges)
            error = des_angle - goal
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()


    def obj_coor(self, min_dis, ref_angle):
        ref_angle = radians(ref_angle)
        transformation_matrix = np.array([
            [cos(self.theta), -sin(self.theta), self.x],
            [sin(self.theta), cos(self.theta), self.y],
            [0, 0, 1]
        ])
        obs_robot_coordinates = np.array([min_dis * cos(ref_angle), min_dis * sin(ref_angle), 1]).reshape(3, 1)
        obs_world = np.matmul(transformation_matrix, obs_robot_coordinates)
        return obs_world[0], obs_world[1]


    def map_object(self):
        plt.ion()
        fig, ax = plt.subplots()
        self.rotate_goal(0)
        self.move_minimum_distance()
        vel_msg = Twist()

        while not rospy.is_shutdown():
            vel_msg.linear.x = 0.1
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

            if self.scan_ranges[270] < 10:
                obs_x_w, obs_y_w = self.obj_coor(self.scan_ranges[269], 270)
                ax.plot(obs_x_w, obs_y_w, '.r')
                plt.draw()
                plt.xlim(-3, 3)
                plt.ylim(-3, 3)
                plt.pause(0.00001)

            if abs(self.scan_ranges[269] - self.minimum_distance) > 0.1:
                self.rotate_goal(270)

        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()

        plt.savefig("map.png")
        plt.ioff()
        plt.show()
        print("Figure saved")


if __name__ == '__main__':
    try:
        robo = my_turtlebot3_map()
        time.sleep(1)
        robo.map_object()
    except rospy.ROSInterruptException:
        pass
