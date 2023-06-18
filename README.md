# rocos_plat
### !!!ros2-foxy功能包：利用ros2可视化移动平台的激光雷达数据点，结合tcp通信协议，大小端转换，浮点数转换!!!

## 1. 可视化视频，ros2可视化激光点云数据
### 1.版本1视频
<img src="https://github.com/cheng9911/rocos_plat/blob/main/images/1.gif" alt="show" />

## 运行

```bash
$ source /opt/ros/foxy/setup.bash
$ cd rocos_plat
$ colcon build
$ source install/setup.bash
$ ros2 run cloud2publish cloud2 
##新建终端
$ rviz2
```
