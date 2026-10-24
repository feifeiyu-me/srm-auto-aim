# SRM 校内赛自瞄系统

基于 ROS2 Humble、Python 和 OpenCV 开发的机甲大师自动瞄准项目。

## 项目功能

- 摄像头采集与装甲板识别
- 目标位置检测与瞄准角度计算
- 云台控制与运动模拟
- 上下位机串口通信
- 通信超时安全保护

## 开发环境

- Ubuntu + ROS2 Humble
- Python 3.10
- OpenCV 4.5.4

## 当前进度

**已完成：** 核心代码开发、基础视觉检测、通信协议验证、海康工业相机图像采集、可视化调试窗口和软件仿真测试。

**待完成：** 工业相机标定和实际射击测试。

## 运行说明

在 ROS2 Humble 环境下执行：

```bash
cd ~/srm_aim_ws
colcon build
source install/setup.bash
```

YOLO-Pose 与 C++ 后处理的多装甲板消息定义、原图时间戳约定和独立编译验证方法见
[视觉消息接口说明](src/srm_aim_interfaces/README.md)。该接口包不启动控制或串口节点。

## 说明

本项目仍在开发中，目前测试结果主要来自软件仿真，完成部分相机实车调试。
