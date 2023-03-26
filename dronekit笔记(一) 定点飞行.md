# Droneki学习笔记(一)

## ·绑定USB端口

### 1.查看USB设备端口信息

`~$ lsusb`

比如：ID 0403:6001

0403 ：**idVendor**
6001 :  **idProduct**

### 2.创建配置文件

`~$ sudo nano /etc/udev/rules.d/usb.rules`

或者使用**gedit**编辑

**usb.rules**是自定义的USB端口注册文件

### 3.注册端口

<code>KERNEL=="ttyACM0", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0777", SYMLINK+="myusb0"</code>

**myusb0**是注册(自定义)的端口名称

飞控的虚拟USB一般搭载在**ttyACM **下

<p>4.应用设置</p>

`~$ sudo udevadm control --reload-rules`

`~$ sudo udevadm trigger`

<p>5.拔插USB</p>

更新USB

## ·连接Pixhawk

`from dronekit import connect`

`vehicle = connect('/dev/ttyAMA0', baud = 115200, wait_ready = True)`

<p>起飞！</p>

### ·解锁

```python
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

def Armed():
    print("====>>基础检测<<====")
    while not vehicle.is_armable:
        print("==等待初始化==")
        time.sleep(1) 
    print("[警报]:即将解锁电机!")
    vehicle.armed = True  #解锁了喔
    while not vehicle.armed:
    print("==等待电机解锁==")
    time.sleep(1)
    
print("准备解锁:")
Armed()
```

### ·修改CH3通道

<p>模拟RC输入</p>

```python
vehicle.channels.overrides = {'5':1144} #给第 5 通道输入值: 1144
```

<p>显示RC通道当前值</p>

```python
def Print_CHX():
	print( "Read channels individually:")
    print( " Ch1: %s" % vehicle.channels['1'])
    print( " Ch2: %s" % vehicle.channels['2'])
    print (" Ch3: %s" % vehicle.channels['3'])
    print( " Ch4: %s" % vehicle.channels['4'])
    print( " Ch5: %s" % vehicle.channels['5'])
    print( " Ch6: %s" % vehicle.channels['6'])
    print( " Ch7: %s" % vehicle.channels['7'])
    print( " Ch8: %s" % vehicle.channels['8'])
    print( "Number of channels: %s" % len(vehicle.channels))
```

<p>设置/显示飞行模式</p>

注：在遥控器+树莓派(机载电脑)的混控模式下

遥控器的优先级更高

修改为定高模式             悬停模式=定点模式

```python
vehicle.mode = VehicleMode("ALT_HOLD") 
```

显示当前飞行模式

```python
print ("Mode: %s" % vehicle.mode.name)
```



### ·起飞函数

```python
ch_increment=500 # 油门增量
ch_step=100 # 油门步进
def Takeoff(key,altitude):
	begin_ch= vehicle.channels[key] # 油门初始值
	for i in range(begin_ch , begin_ch+ch_increment , ch_step): # 缓慢起飞
		vehicle.armed = True
		vehicle.channels.overrides = {key:begin_ch+i}
		time.sleep(1)
		print (f" Ch{key}: %s" % vehicle.channels[key])
    try:
        while True:
            print(" 当前高度: %s m", vehicle.location.global_relative_frame.alt)
            if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
                print(" ===到达设定高度附近===")
                break
            time.sleep(1)
    except:
        print("起飞出错")
        print("正在执行: 降落")
    try:
        print("===开始定高===")
        while True:
            vehicle.channels.overrides = {key:1500} # 开始悬停
            vehicle.armed = True
            time.sleep(1)
    except:
        print("定高中断")
        print("正在执行: 降落")    
	for i in range(begin_ch+ch_increment , begin_ch , -ch_step): # 缓慢降落
		vehicle.armed = True
		vehicle.channels.overrides = {key:begin_ch+i}
		time.sleep(0.5)
		print (f" Ch{key}: %s" % vehicle.channels[key])
        


```

### ·起飞

```python
print("准备解锁:")
Armed()
Takeoff('3',0.5)
vehicle.disarmed = False # 赶紧上锁
vehicle.close() #切断连接
print("空路千万条,飞机第一条.")
print("飞机不规范,炸鸡两行泪.")
```

