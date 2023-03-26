from cmath import nan
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


sum_x=0
sum_y=0
sum_theta=0
# ICP核心代码
def ICP(sourcePoints, targetPoints):
    global sum_theta
    global sum_x
    global sum_y
    A = targetPoints
    B = sourcePoints

    iteration_times = 0
    dist_now = 1
    dist_improve = 1
    dist_before = DistOfTwoSet(A, B)
    C = B
    while iteration_times < 10 and dist_improve > 0.000001:
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
        print("迭代第"+str(iteration_times)+"次, 质心距离是"+str(dist_now)+",提升了"+str(dist_improve))
        print(f"{theta/np.pi*180}")
        sum_x+=x
        sum_y+=y
        sum_theta+=theta
        # print(theta_pre)
        dist_before = dist_now
    x_mean_source = C[0].mean()
    y_mean_source = C[1].mean()
    print(f"==-={round(sum_theta/np.pi*180,3)}  x={round(x_mean_target-x_mean_source,3)} y={round(y_mean_target-y_mean_source,3)}")
    return B


if __name__=="__main__":
    A = np.array([[5, 30], [-5.85, 30.5], [10.65, -0], [-11.2, -0.1]]).transpose()
    # A=[ [ 7.64,4.3,-3.59,-6.61,-2.91,2.64],[ 0.,7.44,6.22 ,0.,-5.05,-4.57]]
    # A=np.array([[1.0, 1.0], [2, 2], [3, 3], [4, 4], [5, 5], [0, 0], [0 , 0], [0, 0]])[0::2]

    theta_pre = math.pi / 3
    R_pre = np.array([[math.cos(theta_pre), math.sin(theta_pre)], [-math.sin(theta_pre), math.cos(theta_pre)]])
    # C = A+10
    B = np.matmul(R_pre, A) +10
    
    
    ICP(A, B)