#!/usr/bin/env python3


'''
### 警告：雷达话题已经改为 myscan !!!!!!!!

    [icp]
    小车
    更换IMU、激光雷达
    激光+IMU 里程计
    发布： 
        1.小车全局坐标
        2.小车相对坐标
    与test03.py 结合控制小车

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
    points=[[0.9,0.9]] # 设置的点集
    Car_statue=[0,0]

    def __init__(self, name):
        super().__init__(name)
        self.switch = 0
        self.imu_yaw = 0
        self.get_logger().info("我是%s test05!" % name)
        # 创建订阅者
        self.command_subscribe_ = self.create_subscription(
            LaserScan, "myscan", self.command_callback, qos_profile_sensor_data)
        # self.submoney = self.create_subscription(
        #     Imu, "imu/data_raw", self.recv_money_callback, qos_profile_sensor_data)
        self.submoney = self.create_subscription(
            Float32, "Test01", self.recv_money_callback, 10)

        self.pub_novel1 = self.create_publisher(Float64MultiArray,"my_odom", 10)
        self.pub_novel2 = self.create_publisher(Float64MultiArray,"car_odom", 10) 
        

    def recv_money_callback(self, imuData):
        """
        4. 编写订阅回调处理逻辑
        """
        if self.switch <=0:
            self.theta_0 = imuData.data*180/pi
            self.imu_yaw = imuData.data*180/pi
            self.switch+=1
        else :
            self.imu_yaw = imuData.data*180/pi


        
    def command_callback(self, msg):
        send_msg = Float64MultiArray(data = [0,0,0])
        car_msg= Float64MultiArray(data = [0,0])
        j=0
        x=[0 ,180,90,270]
        x[0] +=round(self.imu_yaw  - self.theta_0)
        x[1] = x[0]+180
        x[2] = x[0]+90
        x[3] = x[0]+270
        for i in range(4):
            if x[i] <0 :
                x[i]+=360
            elif x[i] >360:
                x[i] -=360
        for i in range(len(msg.ranges)):
            if msg.ranges[i]*msg.intensities[i] != 0:
                for io in range(4):
                    if round(j*180/pi) == x[io] :
                        self.now_statue[io] = msg.ranges[i]
                j+=msg.angle_increment
        self.now_statue2[0] = round(self.now_statue[0],2)
        self.now_statue2[1] = round(self.now_statue[3],2)
        send_msg.data[0] = self.now_statue2 [0]
        send_msg.data[1] = self.now_statue2 [1]
        send_msg.data[2] = round(self.imu_yaw  - self.theta_0)
        self.pub_novel1.publish(send_msg) 
        self.Car_odom(self.points[0])
        car_msg.data[0] = self.Car_statue[0]
        car_msg.data[1] = self.Car_statue[1]
        print(f"{self.now_statue2} x_y=[{round(car_msg.data[0],2)},{round(car_msg.data[1] ,2)}]")
        self.pub_novel2.publish(car_msg) 

    
    def Car_odom(self,point):
        theta = round(self.imu_yaw  - self.theta_0)/180*pi
        # R = np.array([
        #             [np.cos(theta),-np.sin(theta)],
        #             [np.sin(theta),np.cos(theta)]]
        #             )
        R = np.array([[math.cos(theta), math.sin(theta)],
                    [-math.sin(theta), math.cos(theta)]])  # 旋转矩阵
        T = np.array([self.now_statue2 [0],self.now_statue2 [1]])
        tmp = point - T
        tmp1 = np.matmul(R,tmp)
        self.Car_statue[0]  = tmp1 [0]
        self.Car_statue[1]  = tmp1 [1]
        
        return [round(tmp1[0],2),round(tmp1[1],2)]


def main(args=None):
    rclpy.init(args=args)  
    node = NodeSubscribe02("scan2")  
    print("ok!!!")
    rclpy.spin(node)  
    
    rclpy.shutdown()  
    print("Completed")