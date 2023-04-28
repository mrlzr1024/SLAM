#!/usr/bin/env python3


'''
    [icp]
    小车
    更换IMU、激光雷达
    激光+IMU 里程计
    控制
'''
from cmath import pi
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu 
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Twist 
import math
import numpy as np
import math
import cv2 as cv


# ros2 run turtlesim turtle_teleop_key


class NodeSubscribe02(Node):
    theta_0 = 0
    theta_1 = 0
    switch = 0
    xy_msg = np.zeros((720, 2))
    begin_xy_msg = np.zeros((720, 2))
    now_statue = [0,0,0,0]
    now_statue2 = [0,0]
    points=[[0.9,0.9]]
    Car_statue=[0,0]

    def __init__(self, name):
        super().__init__(name)
        self.switch = 0
        self.imu_yaw = 0
        self.get_logger().info("我是%s test03!" % name)
        # 创建订阅者
        self.submoney = self.create_subscription(Float64MultiArray, "car_odom", self.recv_car_callback, 10)
        self.pub_novel3 = self.create_publisher(Twist,"turtle1/cmd_vel", 10) 
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)  
        
    def recv_car_callback(self, OdomData):
        self.Car_statue[0] = OdomData.data[0]
        self.Car_statue[1] = OdomData.data[1]

    def timer_callback(self):
        msg = Twist()
        print(f"[0]={round(self.Car_statue[0],2)} [2]={round(self.Car_statue[1],2)}")
        if self.Car_statue[0] > 0.05:
            msg.linear.x = self.Car_statue[0]
        elif self.Car_statue[0] < -0.05:
            msg.linear.x = self.Car_statue[0]
            
        elif self.Car_statue[1] >0.05:
            msg.linear.z = -self.Car_statue[1]
        elif self.Car_statue[1] <-0.05:
            msg.linear.z = -self.Car_statue[1]
        self.pub_novel3.publish(msg) 
        






def main(args=None):
    rclpy.init(args=args)  
    node = NodeSubscribe02("scan3")  
    print("ok!!!")
    rclpy.spin(node)  
    
    rclpy.shutdown()  
    print("Completed")