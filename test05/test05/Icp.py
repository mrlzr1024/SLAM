

#!/usr/bin/env python3



'''
    [icp]
    纯激光里程计
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np







import numpy as np
import math
import matplotlib.pyplot as plt

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
theta_sum=0
x_sum=0
y_sum=0
# ICP核心代码
def ICP(sourcePoints, targetPoints):
    A = targetPoints
    B = sourcePoints
    C = B
    global theta_sum
    global x_sum
    global y_sum


    # theta_sum=0
    # x_sum=0
    # y_sum=0



    iteration_times = 0
    dist_now = 1
    dist_improve = 1
    dist_before = DistOfTwoSet(A, B)
    # print("===")
    while iteration_times < 30 and dist_improve > 0.00000001:
        x_mean_target = A[0].mean()
        y_mean_target = A[1].mean()
        x_mean_source = B[0].mean()
        y_mean_source = B[1].mean()

        A_ = A - np.array([[x_mean_target], [y_mean_target]])
        B_ = B - np.array([[x_mean_source], [y_mean_source]])

        w_up = 0
        w_down = 0
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
        C = np.matmul(R, C)
        iteration_times = iteration_times + 1
        dist_now = DistOfTwoSet(A, B)
        dist_improve = dist_before - dist_now
        print("迭代第"+str(iteration_times)+"次, 损失是"+str(dist_now)+",角度："+str(round(theta_sum/np.pi*180,3)))
        theta_sum+=theta
        
        
        dist_before = dist_now
    
    x = C[0].mean()
    y = C[1].mean()
    x_sum+=x_mean_target-x
    y_sum+=y_mean_target-y
    print(f"=== R={round(theta_sum/np.pi*180,3)}\t; X={round(x_sum,3)}\t; Y = {round(y_sum,3)}")
    return B


import cv2
import numpy as np
 






# picture = picture*255
 

def draw(A):
    # A = np.array([[7.6, 0], [4.3, 7.4], [-3.5, 6.2], [-6.6, 0], [-2.9, -5.0], [2.6, -4.5]])
    min_x = 0
    min_y = 0
    for i in A:
        if i[1]<min_y:
            min_y = i[1]
        if i[0]<min_x:
            min_x = i[0]
    print(f"x= {min_x} y={min_y}")
    for i in range(len(A)):
        A[i][0]+=-min_x
        A[i][0]*=10
        A[i][1]+=-min_y
        A[i][1]*=10

    max_x = 0
    max_y = 0
    for i in A:
        if i[1]>max_y:
            max_y = i[1]
        if i[0]>max_x:
            max_x = i[0]
    print(f"x= {max_x} y={max_y}")


    # print((int(A[0][0]),int(A[0][1])))


    img = np.ones((int(max_x+50),int(max_y+50),3),'uint8')*255

    for i in range(len(A)):
        cv2.circle(img, (int(A[i][0])+10,int(A[i][1])+10), 1, (0, 0, 255), 2)

    cv2.imshow('line',img)
    cv2.waitKey(0)

class NodeSubscribe02(Node):
    xy_msg=np.zeros((360,2))
    begin_xy_msg=np.zeros((360,2))
    def __init__(self,name):
        super().__init__(name)
        self.switch = 0
        self.get_logger().info("我是%s!" % name)
        # 创建订阅者
        self.command_subscribe_ = self.create_subscription(LaserScan,"scan",self.command_callback,100)

    def transfor_to(self,a_msg):
        # print("=======+=======")
        for i in range(360):
            self.xy_msg[i][0]=round((a_msg[i]*np.cos(i/180.0*np.pi)),2)
            self.xy_msg[i][1]=round((a_msg[i]*np.sin(i/180.0*np.pi)),2)
            
            # print(self.xy_msg[i][0])

    def command_callback(self,msg):
        # print(len(msg.ranges[0::9]))
        # ============================ #
        # print("=======+=======")
        for i in range(len(msg.ranges)):
            if np.isinf(msg.ranges[i]):
                msg.ranges[i]=18
                
        # ============================= #        
        self.transfor_to(msg.ranges[0::9])
        if self.switch == 0 :
            self.begin_xy_msg = np.array(self.xy_msg, copy=True).transpose()
            self.switch+=1
            return
        # ============================= #

        # draw(self.begin_xy_msg)
        # exit()

       
        # A = self.begin_xy_msg.transpose()


        # ===测试集======================== #
        # A = np.array([[5, 30], [-5.85, 30.5], [10.65, -0], [-11.2, -0.1]]).transpose()
        # theta_pre = math.pi / 3
        # R_pre = np.array([[math.cos(theta_pre), math.sin(theta_pre)], [-math.sin(theta_pre), math.cos(theta_pre)]])
        # C = A-30
        # B = np.matmul(R_pre, A) +114
        # print("======")
    
        # ICP(A, B)
        # exit()
        # ===测试集======================== #


        
        ICP(self.begin_xy_msg,self.xy_msg.transpose())
        self.begin_xy_msg = np.array(self.xy_msg, copy=True).transpose()
        # print("========结束=============")
        
        
    
    


def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = NodeSubscribe02("scan2")  # 新建一个节点
    print("ok!!!")
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
