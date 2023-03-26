

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np







import numpy as np
import math
import matplotlib.pyplot as plt
theta_all=0
# 求出两个点之间的向量角度，向量方向由点1指向点2
def getThetaOfTwoPoints(x1, y1, x2, y2):
    return math.atan2(y2-y1, x2-x1)

# 求出两个点的距离
def getDistOfTwoPoints(x1, y1, x2, y2):
    return math.sqrt(math.pow(x2-x1, 2) + math.pow(y2-y1, 2))

# 在pt_set点集中找到距(p_x, p_y)最近点的id
def getClosestID(p_x, p_y, pt_set):
    id = 0
    min = 10000000
    for i in range(pt_set.shape[1]):
        dist = getDistOfTwoPoints(p_x, p_y, pt_set[0][i], pt_set[1][i])
        if dist < min:
            id = i
            min = dist
    return id

# 求出两个点集之间的平均点距
def DistOfTwoSet(set1, set2):
    loss = 0
    for i in range(set1.shape[1]):
        id = getClosestID(set1[0][i], set1[1][i], set2)
        dist = getDistOfTwoPoints(set1[0][i], set1[1][i], set2[0][id], set2[1][id])
        loss = loss + dist
    return loss/set1.shape[1]

# ICP核心代码
def ICP(sourcePoints, targetPoints):
    global theta_all
    A = targetPoints
    B = sourcePoints

    iteration_times = 0
    dist_now = 1
    dist_improve = 1
    # print("=====DistOfTwoSet========")
    dist_before = DistOfTwoSet(A, B)
    # print("=====while========")
    while iteration_times < 10 and dist_improve > 0.0001:
        x_mean_target = A[0].mean()
        y_mean_target = A[1].mean()
        x_mean_source = B[0].mean()
        y_mean_source = B[1].mean()
        # print(np.array([[x_mean_target], [y_mean_target]]))
        # print(A)
        A_ = A - np.array([[x_mean_target], [y_mean_target]])
        B_ = B - np.array([[x_mean_source], [y_mean_source]])
        if (A_==B_).all() :
            print("[A==B]",end="")
        w_up = 0
        w_down = 0
        # print("=====for========")
        for i in range(A_.shape[1]):
            j = getClosestID(A_[0][i], A_[1][i], B_)
            w_up_i = A_[0][i]*B_[1][j] - A_[1][i]*B_[0][j]
            w_down_i = A_[0][i]*B_[0][j] + A_[1][i]*B_[1][j]
            w_up = w_up + w_up_i
            w_down = w_down + w_down_i

        theta = math.atan2(w_up, w_down)
        x = x_mean_target - math.cos(theta)*x_mean_source - math.sin(theta)*y_mean_source
        y = y_mean_target + math.sin(theta)*x_mean_source - math.cos(theta)*y_mean_source
        R = np.array([[math.cos(theta), math.sin(theta)], [-math.sin(theta), math.cos(theta)]])

        B = np.matmul(R, B) + np.array([[x], [y]])

        iteration_times = iteration_times + 1
        dist_now = DistOfTwoSet(A, B)
        dist_improve = dist_now -dist_before 
        # print("迭代第"+str(iteration_times)+"次, 损失是"+str(dist_now)+",质心距离是"+str(dist_improve))


        theta_all+=theta
        print("=>   theta="+str(int(theta_all/np.pi*180)))
        dist_before = dist_now
    
    return B


class NodeSubscribe02(Node):
    # begin_xy_msg=np.zeros((360,2))
    # debug_xy_msg=np.zeros((360,1))
    xy_msg=np.zeros((360,2))
    def __init__(self,name):
        super().__init__(name)
        self.switch = 0
        self.get_logger().info("我是%s!" % name)
        # 创建订阅者
        # self.command_subscribe_ = self.create_subscription(String,"command",self.command_callback,10)
        self.command_subscribe_ = self.create_subscription(LaserScan,"scan",self.command_callback,100)

    def transfor_to(self,a_msg):
        # print("=======+=======")
        for i in range(360):
            self.xy_msg[i][0]=int(a_msg[i]*np.cos(i/180.0*np.pi))
            self.xy_msg[i][1]=int(a_msg[i]*np.sin(i/180.0*np.pi))
            
            print(self.xy_msg[i][0])

    def command_callback(self,msg):
        
        # ============================ #
        # print("=======+=======")
        for i in range(360):
            if np.isinf(msg.ranges[i]):
                msg.ranges[i]=18
                
        # ============================= #        
        self.transfor_to(msg.ranges)
        if self.switch == 0 :
            self.begin_xy_msg = np.array(self.xy_msg, copy=True)
            self.switch+=1
        ICP(self.begin_xy_msg.transpose(),self.xy_msg.transpose())
        # print("========结束=============")
        # exit()
        
    
    


def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = NodeSubscribe02("scan2")  # 新建一个节点
    print("ok!!!")
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
