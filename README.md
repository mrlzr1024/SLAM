# SLAM
My slam code 
ROS2 功能包

  test04 -------- ICP算法的基本使用
   + change_to_xy2.py ----- 用图像处理的方法解决纯激光定位问题
   + ICP.py ------ 测试ICP算法，解决纯激光定位
   + kmeans.py ------基于sklearn下的kmeans算法
   + opencv_test.py -------- opencv显示激光雷达的信息
   + topic_subscribe_02.py ------ (已弃用) ros2 读取 激光雷达数据
   + test01.jpg 图片测试集
   + test01.png 图片测试集
  
  test05 -------- 结合激光雷达的ICP算法 
   + Icp.py -------- ROS2下，用ICP解决纯激光定位问题(真机)-  精度较低
   + icp02.py ------- ROS2下，激光+IMU 里程计 (真机) -----  未验证
   + ICP03.py ------ ROS2下，激光+IMU 里程计(小车) ----     已弃用
   + ICP04.py ------ ROS2下,激光+IMU 里程计(小车) -----     验证通过
   + test.py ------ 初步使用对雷达插入四个值来实现激光里程计 ----------  验证通过
   + + + ----- 激光雷达的话题更改为 myscan ---------- 
   + test02.py ------ 在test.py的基础上发布小车的全局坐标和局部坐标 ----------  验证通过
   + test03.py ------ 配合test02.py实现小车的动态控制(通过计算局部坐标) ----------  验证通过
   + test04.py ------ 独立实现小车的动态控制(通过计算局部坐标) ----------  验证通过
   + test05.py ------ 在test04.py的基础上引入全局目标点集 ----------  验证通过
   
   
 
