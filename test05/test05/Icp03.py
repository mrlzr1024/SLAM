#!/usr/bin/env python3


'''
    [icp]
    激光+IMU 里程计
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import matplotlib.pyplot as plt
from dronekit import connect, VehicleMode, Rangefinder


def get_xy(A, B, theta):
    '''
        A : 起始状态(坐标系原点)
        B : 终止状态
        theta : 旋转角度
    '''
    A_X = A[0].mean()
    A_Y = A[1].mean()
    R = np.array([[math.cos(theta), math.sin(theta)],
                 [-math.sin(theta), math.cos(theta)]])  # 旋转矩阵
    B = np.matmul(R, B)
    B_X = B[0].mean()
    B_Y = B[1].mean()
    return (round(A_X-B_X, 2), round(A_Y - B_Y, 2), round(theta / np.pi*180, 3))


class NodeSubscribe02(Node):
    theta_0 = 0
    theta_1 = 0
    xy_msg = np.zeros((360, 2))
    begin_xy_msg = np.zeros((360, 2))
    # vehicle = connect("/dev/myusb0", baud=115200, wait_ready=True)

    def __init__(self, name):
        super().__init__(name)
        self.switch = 0
        self.imu_roll = 0
        self.imu_yaw = 0
        self.imu_pitch = 0
        self.get_logger().info("我是%s!" % name)
        # 创建订阅者
        self.command_subscribe_ = self.create_subscription(
            LaserScan, "scan", self.command_callback, 100)
        self.submoney = self.create_subscription(
            Float64MultiArray, "imu_data", self.recv_money_callback, 10)

    def recv_money_callback(self, imuData):
        """
        4. 编写订阅回调处理逻辑
        """
        self.imu_roll = imuData.data[0]
        self.imu_yaw = imuData.data[2]
        self.imu_pitch = imuData.data[1]
        # self.get_logger().info('接收到imu的位姿信息:%s' % self.imu_roll)

    def transfor_to(self, a_msg):
        # print("=======+=======")
        for i in range(360):
            self.xy_msg[i][0] = round((a_msg[i]*np.cos(i/180.0*np.pi)), 2)
            self.xy_msg[i][1] = round((a_msg[i]*np.sin(i/180.0*np.pi)), 2)

    def command_callback(self, msg):
        # ============================ #
        for i in range(len(msg.ranges)):
            if np.isinf(msg.ranges[i]):
                msg.ranges[i] = 18
        # ============================= #
        self.transfor_to(msg.ranges[0::9])
        if self.switch == 0:
            self.begin_xy_msg = np.array(self.xy_msg, copy=True).transpose()
            # self.theta_0 = self.vehicle.attitude.yaw
            # self.theta_1 = self.vehicle.attitude.yaw
            self.theta_0 = self.imu_yaw 
            self.theta_1 = self.imu_yaw 
            self.switch += 1
            return
        # ============================= #
        self.transfor_to(msg.ranges[0::9])
        B_0 = self.xy_msg.transpose()
        A_0 = self.begin_xy_msg

        print(get_xy(A_0, B_0, self.theta_0-self.theta_1))
        self.begin_xy_msg = np.array(self.xy_msg, copy=True).transpose()
        self.theta_1 = self.imu_yaw


def main(args=None):
    rclpy.init(args=args)  # 初始化rclpy
    node = NodeSubscribe02("scan2")  # 新建一个节点
    print("ok!!!")
    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    # node.vehicle.close()
    rclpy.shutdown()  # 关闭rclpy
    print("Completed")
