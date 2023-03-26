# ROS2读取激光雷达数据

laser ----topic

## 前期准备

### ·查看ROS2话题

<code>ros2 topic list </code>

### ·查看消息类型

`ros2 topic list -t`

`>>> /scan [sensor_msgs/msg/LaserScan]`

### ·查看数据结构

`ros2 interface show sensor_msgs/msg/LaserScan`

```makefile
std_msgs/Header header   # Header也是一个结构体,包含了seq,stamp,frame_id,其中seq
                         # 指的是扫描顺序增加的id,stamp包含了开始扫描的时间和与开始扫描的时间差,frame_id是扫描的参考系名称.注意扫描是逆时针从正前方开始扫描的.   
　　uint32 seq
　　time stamp           
　　string frame_id 　　 # frame在ROS中作用至关重要，消息将和tf绑定才可以读取数据，在这里作为通用可配置，暂定内容为：laser，用户可自定义设置。                                                           
float32 angle_min        # 开始扫描的角度(rad)
float32 angle_max        # 结束扫描的角度(rad)
float32 angle_increment  # 每一次扫描增加的角度(rad)

float32 time_increment   # 测量的时间间隔(second)
float32 scan_time        # 扫描的时间间隔(second)

float32 range_min        # 距离最小值(m)
float32 range_max        # 距离最大值(m)

float32[] ranges         # 距离数组(m) (长度360*n)  (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # 与设备有关,强度数组(长度360*n)

```



### ·创建功能包

`ros2 pkg create test04 --build-type ament_python --dependencies rclpy`

`cd test04/test04`

`touch Get_laser.py`

## 编写接收程序

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class NodeSubscribe02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("节点 %s 启动" % name) # 创建订阅者
        self.command_subscribe_ = self.create_subscription(
            LaserScan,
            "scan",
            self.command_callback,
            100)
    def command_callback(self,msg):
        self.get_logger().info(f'收到[{msg.ranges[0]}]') # 激光雷达 0 °的值

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = NodePublisher02("Get_scan")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy

```

### ·注册结点

 setup.py

```python
    entry_points={
        'console_scripts': [
            "topic_subscribe_02 = test04.Get_laser:main"
        ],
    },
```

### ·运行结点

```makefile
ros2 run test04 topic_subscribe_02
```





