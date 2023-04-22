#!/usr/bin/env python3


'''
    [icp]
    小车
    激光+IMU 里程计
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, Float32
import numpy as np
import math
import matplotlib.pyplot as plt
from dronekit import connect, VehicleMode, Rangefinder
import cv2 as cv

# ros2 run turtlesim turtle_teleop_key
def get_xy(A, B, theta):
    '''
        A : 起始状态(坐标系原点)
        B : 终止状态
        theta : 旋转角度
    '''
    theta =-theta
    R = np.array([[math.cos(theta), math.sin(theta)],
                 [-math.sin(theta), math.cos(theta)]])  # 旋转矩阵
    A_X = A[0].mean()
    A_Y = A[1].mean()
    C = np.matmul(R, B)
    B_X = C[0].mean()
    B_Y = C[1].mean()
    # print(B)
    # == 画图 ======================= #
    picture = np.ones((500,500,3),'uint8')
    img = picture*255
    # 显示原状态
    for x,y in zip(A[0],A[1]):
        cv.circle(img, (300+int(50*x),300+int(50*y)), 1, (255,0,0), 2)
    cv.circle(img, (300+int(50*A_X),300+int(50*A_Y)), 1, (255,0,0), 2)
    
    # 显示现状态
    for x,y in zip(B[0],B[1]):
        cv.circle(img, (300+int(50*x),300+int(50*y)), 1, (0,255,0), 2)
    
    # 显示矫正后的现状态
    for x,y in zip(C[0],C[1]):
        cv.circle(img, (300+int(50*x),300+int(50*y)), 1, (0,0,255), 2)
    cv.circle(img, (300+int(50*B_X),300+int(50*B_Y)), 1, (0,0,255), 2)

    cv.imshow("img", img)
    if(cv.waitKey(1)&0xff) == 27:
        return
    # test = np.array([1,0])
    # test2=np.matmul(R,test)
    # print(test2)
    return (round(A_X-B_X, 2)*2, round(A_Y - B_Y, 2)*2, round(theta / np.pi*180, 3))


class NodeSubscribe02(Node):
    theta_0 = 0
    theta_1 = 0
    xy_msg = np.zeros((360, 2))
    begin_xy_msg = np.zeros((360, 2))
    # vehicle = connect("/dev/myusb0", baud=115200, wait_ready=True)

    def __init__(self, name):
        super().__init__(name)
        self.switch = 0
        self.imu_yaw = 0
        self.get_logger().info("我是%s!" % name)
        # 创建订阅者
        self.command_subscribe_ = self.create_subscription(
            LaserScan, "scan", self.command_callback, 100)
        self.submoney = self.create_subscription(
            Float32, "Test01", self.recv_money_callback, 10)

    def recv_money_callback(self, imuData):
        """
        4. 编写订阅回调处理逻辑
        """
        self.imu_yaw = imuData.data

        # self.get_logger().info('接收到imu的位姿信息:%s' % self.imu_roll)

    def transfor_to(self, a_msg):
        # print("=======+=======")
        for i in range(360):
            self.xy_msg[i][0] = round((a_msg[i]*np.cos(i/180.0*np.pi)), 2)
            self.xy_msg[i][1] = round((a_msg[i]*np.sin(i/180.0*np.pi)), 2)

    def command_callback(self, msg):
        # == 处理inif（无效）的数据 ========================== #
        for i in range(len(msg.ranges)):
            if np.isinf(msg.ranges[i]):
                msg.ranges[i] = 18
        # ============================= #
        # print(len(msg.ranges))
        if len(msg.ranges) < 3240:
            return
        self.transfor_to(msg.ranges[0::9])
        if self.switch <= 5: #校准
            self.begin_xy_msg = np.array(self.xy_msg, copy=True).transpose()
            # self.theta_0 = self.vehicle.attitude.yaw
            # self.theta_1 = self.vehicle.attitude.yaw
            self.theta_0 = self.imu_yaw
            self.theta_1 = self.imu_yaw
            self.switch += 1
            return
        # ============================= #
        # self.transfor_to(msg.ranges[0::9])
        B_0 = self.xy_msg.transpose() # 现在的数据
        A_0 = self.begin_xy_msg # 初始时刻的数据

        self.theta_1 = self.imu_yaw
        print(F"{get_xy(A_0, B_0, self.theta_0-self.theta_1)},{round(self.theta_0 / np.pi*180, 3)}")
        # self.begin_xy_msg = np.array(self.xy_msg, copy=True).transpose()
        


def main(args=None):
    rclpy.init(args=args)  # 初始化rclpy
    node = NodeSubscribe02("scan2")  # 新建一个节点
    print("ok!!!")
    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    # node.vehicle.close()
    rclpy.shutdown()  # 关闭rclpy
    print("Completed")
