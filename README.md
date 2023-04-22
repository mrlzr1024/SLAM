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
   + test03.py ------ dronekit 读取PX4基本数据 ----------  验证通过
   + ICP03.py ------ ROS2下，激光+IMU 里程计(小车) ----     已弃用
   + ICP04.py ------ ROS2下,激光+IMU 里程计(小车) -----     验证通过
   
 
