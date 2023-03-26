​    

# 二维[ ICP ]算法 

[TOC]



## 1)点云归一化

$$
A重心=\frac{1}{n}*\sum_{i=1}^{n}Ai
$$

$$
B重心=\frac{1}{n}*\sum_{i=1}^{n}Bi
$$

$$
ai = Ai - A重心
$$

$$
bi = Bi - B重心
$$

ai , bi 为归一化后的数据

## 2)找到最近的点

使用欧氏距离（固定 i 改变 j ）
$$
距离=\min_{j} \sqrt{(ax[i]-bx[j])^2 + (ay[i]-by[j])^2}
$$

## 3)获取变换矩阵

点云 A , B 对应点的距离和函数为D(R,t)

​	
$$
D(R,t) = \frac{1}{n} \sum_{i=1}^{n} |ai-R·bi|^2
$$
即：

![ ](C:\Users\11401\AppData\Roaming\Typora\typora-user-images\image-20230316230447909.png)

###  ·旋转矩阵：

<img src=".\image-20230316230630470.png" alt="image-20230316230630470" style="zoom:200%;" />

### · 旋转弧度θ  

![image-20230316075415349](C:\Users\11401\AppData\Roaming\Typora\typora-user-images\image-20230316075415349.png)

### · 平移向量t

![image-20230316075426957](C:\Users\11401\AppData\Roaming\Typora\typora-user-images\image-20230316075426957.png)

## 4)点云变换

$$
B'=R·B + t
$$

## 5)计算损失

$$
LOSS=D(R,t)=\frac{1}{n} \sum_{i=1}^{n} |Ai - B'i|^2
$$

## 6)迭代

### 至LOSS值在阈值内





## 7)代码

```python
import numpy as np
import math
import matplotlib.pyplot as plt

# 求出两个点之间的向量弧度，向量方向由点1指向点2
def GetTheAngleOfTwoPoints(x1,y1,x2,y2):
    return math.atan2(y2-y1,x2-x1)

# 求出两个点的欧式距离
def GetDistOfTwoPoints(x1,y1,x2,y2):
    return math.sqrt(math.pow(x2-x1,2) + math.pow(y2-y1,2))

# 在pt_set点集中找到距(p_x，p_y)最近点的id
def GetClosestID(p_x,p_y,pt_set):
    id = 0
    min = 10000000
    for i in range(pt_set.shape[1]):
        dist = GetDistOfTwoPoints(p_x,p_y,pt_set[0][i],pt_set[1][i])
        if dist < min:
            id = i
            min = dist
    return id

# 求两个点集之间的平均点距
def DistOfTwoSet(set1,set2):
    loss = 0;
    for i in range(set1.shape[1]):
        id = GetClosestID(set1[0][i],set1[1][i],set2)
        dist = GetDistOfTwoPoints(set1[0][i],set1[1][i],set2[0][id],set2[1][id])
        loss = loss + dist
    return loss/set1.shape[1]

def ICP(sourcePoints,targetPoints):
    A = targetPoints # A是标准点云
    B = sourcePoints # B是源点云

    iteration_times = 0 # 迭代次数为0
    dist_now = 1 # A,B两点集之间初始化距离
    dist_improve = 1 # A,B两点集之间初始化距离提升
    dist_before = DistOfTwoSet(A,B) # A,B两点集之间距离
    while iteration_times < 10 and dist_improve > 0.001: # 迭代次数小于10 并且 距离提升大于0.001时，继续迭代
        x_mean_target = A[0].mean() # 获得A点云的x坐标轴均值。
        y_mean_target = A[1].mean() # 获得A点云的y坐标轴均值。
        x_mean_source = B[0].mean() # 获得B点云的x坐标轴均值。
        y_mean_source = B[1].mean() # 获得B点云的y坐标轴均值。

        A_ = A - np.array([[x_mean_target],[y_mean_target]]) # 获得A点云的均一化后点云A_
        B_ = B - np.array([[x_mean_target],[y_mean_target]]) # 获得B点云的均一化后点云B_

        w_up = 0 # w_up，表示角度公式中的分母
        w_down = 0 # w_up，表示角度公式中的分子
        for i in range(A_.shape[1]): # 对点云中每个点进行循环
            j = GetClosestID(A_[0][i],A_[1][i],B) # 在B点云中找到距(A_[0][i],A_[1][i])最近点的id
            w_up_i = A_[0][i]*B_[1][j] - A_[1][i]*B_[0][j] # 获得求和公式，分母的一项
            w_down_i = A_[0][i]*B_[0][j] + A_[1][i]*B_[1][j] # 获得求和公式，分子的一项
            w_up = w_up + w_up_i
            w_down = w_down + w_down_i

        TheRadian = math.atan2(w_up,w_down) # 分母与分子之间的角度
        x = x_mean_target - math.cos(TheRadian)*x_mean_source - math.sin(TheRadian)*y_mean_source # x即为x轴偏移量
        y = x_mean_target + math.cos(TheRadian)*x_mean_source - math.sin(TheRadian)*y_mean_source # y即为y轴偏移量
        R = np.array([[math.cos(TheRadian),math.sin(TheRadian)],[-math.sin(TheRadian),math.cos(TheRadian)]]) # 由θ得到旋转矩阵。

        B = np.matmul(R,B) + np.array([x],[y])

        iteration_times = iteration_times + 1 # 迭代次数+1
        dist_now = DistOfTwoSet(A,B) # 变换后两个点云之间的距离
        dist_improve = dist_before - dist_now # 这一次迭代，两个点云之间的距离提升。
        print("迭代第"+str(iteration_times)+"次，损失是"+str(dist_now)+",提升了"+str(dist_improve)) # 打印迭代次数、损失距离、损失提升
        dist_before = dist_now # 将"现在距离"赋值给"以前距离"，开始下一轮迭代循环。

    return B
```

## 8)试制

```python
import numpy as np
import math

class ICP:
    def __init__(self,A,B,length):
        '''
        A:二维向量
        B:二维向量
        '''
        self.A=A
        self.B=B
        self.length
    def Biaozhun_hua(self):
        '''
        标准化
        '''
        self.Ac=[0,0]
        self.Bc=[0,0]
        self.Ac[0]=sum(A[0])/length
        self.Ac[1]=sum(A[1])/length
        self.Bc[0]=sum(B[0])/length
        self.Bc[1]=sum(B[1])/length
        A[0] = A[0] - Ac[0]
        A[1] = A[1] - Ac[1]
        B[0] = B[0] - Bc[0]
        B[1] = B[1] - Bc[1]
        
     def Ou_juli(self,i,j):
        '''欧氏距离'''
        return sqrt(pow(A[0][i]-B[0][j],2)+pow(A[1][i]-B[1][j],2))
    
     def Zhao_zuijin_dian(self,i):
        '''找最近点的ID'''
        min_a=1000
        id=10000
        for j in self.B:
            tmp=Ou_juli(i,j)
            if tmp<min_a:
                min_a=tmp
                id=j
        return id
       
        def Get_thita(self):
            '''计算Θ的值'''
            fen_zi=0
            fen_mu=0
            for i in range(self.length):
                fen_zi+=A[0][i]*B[1][i]-A[1][i]*B[0][i]
                fen_mu+=A[0][i]*A[1][i]+B[0][i]*B[1][i]
                
            self.thita=math.atan2(fen_zi,fen_mu)
            return self.thita
        def Get_rotation_matrix(self):
            '''获取旋转矩阵'''
            self.rotation_matrix=np.array([math.cos(thita) , math.sin(thita)],
                                         [-math.sin(thita),math.cos(thita)])
            return self.rotation_matrix
        def Get_t(self):
            '''获取预测偏移量'''
            self.t=np.array([self.Ac[0],self.Ac[1]])-
            	np.matmul(self.rotation_matrix,np.array[ self.Bc[0], self.Bc[1]])
        def B_pie(self):
            '''预测B'''
            self.B = np.matmul(self.rotation_matrix,self.B) + self.t
        def Get_loss(self):
            '''计算损失函数'''
            self.loss=0
            for i in range(self.length):
                id=Zhao_zuijin_dian(i)
                loss+=Ou_juli(self,i,id)
            self.loss/=self.length
            print(f"loss={self.loss} thita={self.thita} t={self.t}\n")
            return self.loss
       
icp=ICP(A,B,A.shape[0])
icp.Biaozhun_hua()
for i in range(3):
    icp.Get_thita()
    icp.Get_rotation_matrix()
    icp.Get_t()
    icp.B_pie()
    icp.Get_loss()

            
    
```

