from cmath import sqrt
import numpy as np
import cv2
from matplotlib import pyplot as plt


def cross_point(line1,line2):#计算交点函数
    x1=line1.x0#取四点坐标
    y1=line1.y0
    x2=line1.x1
    y2=line1.y1
    
    x3=line2.x0
    y3=line2.y0
    x4=line2.x1
    y4=line2.y1
    
    if x2 - x1 == 0:  # L1 直线斜率不存在
        k1 = None
        b1 = 0
    else:
        k1 = (y2 - y1) * 1.0 / (x2 - x1)  # 计算k1,由于点均为整数，需要进行浮点数转化
        b1 = y1 * 1.0 - x1 * k1 * 1.0  # 整型转浮点型是关键
 
    if (x4 - x3) == 0:  # L2直线斜率不存在操作
        k2 = None
        b2 = 0
    else:
        k2 = (y4 - y3) * 1.0 / (x4 - x3)  # 斜率存在操作
        b2 = y3 * 1.0 - x3 * k2 * 1.0
 
    if k1 is None and k2 is None:  # L1与L2直线斜率都不存在，两条直线均与y轴平行
        if x1 == x3:  # 两条直线实际为同一直线
            return [x1, y1]  # 均为交点，返回任意一个点
        else:
            return None  # 平行线无交点
    elif k1 is not None and k2 is None:  # 若L2与y轴平行，L1为一般直线，交点横坐标为L2的x坐标
        x = x3
        y = k1 * x * 1.0 + b1 * 1.0
    elif k1 is None and k2 is not None:  # 若L1与y轴平行，L2为一般直线，交点横坐标为L1的x坐标
        x = x1
        y = k2 * x * 1.0 + b2 * 1.0
    else:  # 两条一般直线
        if k1 == k2:  # 两直线斜率相同
            if b1 == b2:  # 截距相同，说明两直线为同一直线，返回任一点
                return [x1, y1]
            else:  # 截距不同，两直线平行，无交点
                return None
        else:  # 两直线不平行，必然存在交点
            x = (b2 - b1) * 1.0 / (k1 - k2)
            y = k1 * x * 1.0 + b1 * 1.0
    return [int(x), int(y)]





image = cv2.imread("/home/mrlzr/Ros_1_2/Ros2_project/src/test04/test04/test01.jpg")
print(image.shape)
image_copy = image.copy()

# 转换成灰度图
image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 边缘检测, Sobel算子大小为3
edges = cv2.Canny(image_gray, 170, 220, apertureSize=3)

# 霍夫曼直线检测
lines = cv2.HoughLines(edges, 4.9, np.pi / 180, 250)
SET=[]
# 遍历
for line in lines:
    # 获取rho和theta
    rho, theta = line[0]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))
    SET.append(set(x1,x2,y1,y2))
    # cv2.line(image_copy, (x1, y1), (x2, y2), (0, 0, 255), thickness=2)



SET_1=[]
# 取交点
for i in SET:
    for j in SET:
        a= cross_point(i,j)
        if a!=None:
            SET_1.append(a)



SET_1=np.array(SET_1)

from sklearn.cluster import KMeans
y_pred = KMeans(n_clusters=4, random_state=10).fit_predict(SET_1)

# plt.scatter(SET_1[:, 0], SET_1[:, 1], c=y_pred)
# plt.show()

# SET_4=My_k_means(SET_1,4,1)



j=0
for i in range(len(SET_1)):
    if int(y_pred[i]) == 3:
        j+=1
        print(tuple(SET_1[i]))
        cv2.circle(image, tuple(SET_1[i]+(100,20482)), 1, (0,100,1), 5)


print(j)



cv2.imshow("img", image)  # 展示结果
#等待
cv2.waitKey(0)
#关闭所有窗口
cv2.destroyAllWindows()
