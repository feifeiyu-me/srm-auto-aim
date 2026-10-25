# 自瞄视觉消息接口

`srm_aim_interfaces` 是仅生成消息的 `ament_cmake` 包，面向 ROS2 Humble。
现有 `srm_aim_comm` 是 `ament_python` 包；`srm_hik_camera` 是依赖海康
MVS SDK 的 C++17 `ament_cmake` 相机包。两者继续保留各自职责，接口生成独立于
Python 算法包和相机 SDK。本包不创建节点、不访问串口，也不发送控制或发射指令。

## 消息与语义

建议未来 Python 检测节点发布 `/srm_aim/armor_detections`，类型为
`srm_aim_interfaces/msg/ArmorDetectionArray`，供 C++ 后处理节点订阅。
本次只提供消息，不增加发布者、订阅者或 YOLO 推理实现；既有话题保持原样。

| 消息 | 字段 | 含义 |
| --- | --- | --- |
| `Keypoint2D` | `float32 x, y` | 原始 ROS 图像中的亚像素坐标 |
| `ArmorDetection` | `uint8 color` | `COLOR_UNKNOWN=0`、`COLOR_RED=1`、`COLOR_BLUE=2` |
| | `float32 confidence` | 该装甲板实例的检测置信度，有限且在 `[0, 1]` 内 |
| | `Keypoint2D lt, lb, rb, rt` | 左上、左下、右下、右上四点 |
| `ArmorDetectionArray` | `std_msgs/Header header` | 原始 `sensor_msgs/Image.header` 的副本 |
| | `uint32 image_width, image_height` | 该原始图像的尺寸 |
| | `ArmorDetection[] detections` | 同一帧中的零个或多个装甲板实例 |

### 坐标、颜色与多目标约定

- “原始图像”指实际发布的 `sensor_msgs/Image`，包括相机节点已经完成的旋转；
  不指模型缩放、裁剪或 letterbox 后的网络输入。坐标原点在左上，x 向右、y 向下。
- 检测节点必须先逆变换模型预处理，恢复原图坐标，并按训练标注含义映射四点。
  不得假设模型数组天然按 LT/LB/RB/RT 排列，也不能仅按 x/y 大小重新排序倾斜装甲板。
  此处 LT/LB 是左灯条上/下端点，RB/RT 是右灯条下/上端点，与现有 `ArmorPnP`
  的模型点对应。若训练集标注的是装甲板外框而非灯条端点，须先验证几何定义能否适配。
- 颜色描述观察到的装甲板灯条颜色，与串口 `robot_id` 的编码没有隐含映射。
  模型不能识别颜色时使用 `COLOR_UNKNOWN`，不能默认当作红色或蓝色。
- 发布者只应发送四点均有效、坐标有限且位于原图内的实例：
  `0 <= x < image_width`、`0 <= y < image_height`，尺寸须大于零。
  消息未包含单点可见性或单点置信度；缺点实例应过滤，不能以 `(0, 0)` 假装缺失值。
  `(0, 0)` 本身是合法图像坐标。
- 每个数组元素对应一个实例。数组顺序不是稳定跟踪 ID，不保证跨帧身份；目标跟踪
  由后续节点负责。`confidence` 是检测评分，不是数字 2 模板分数或 PnP 重投影误差。
- 正常完成推理但没有有效目标时，仍发布带该帧 Header 和尺寸的空数组。
  未处理或推理异常的帧不应伪装成正常空结果；消费者需要超时处理。
- `.msg` 定义只提供结构和类型，不自动强制上述数值、颜色和几何约束；
  后续发布者和消费者均需验证，C++ PCA/PnP 还需检查四点不退化和点序正确。

### 时间戳和图像同步

发布时完整复制输入图像的 Header：`stamp` 和 `frame_id` 都保持不变。
禁止将 `stamp` 替换为推理结束时的 `now()`。现有相机时间戳是主机接收时间，
本接口保留它的含义，不将其声明为硬件曝光时间。

后续 C++ PCA 节点可缓存 `/srm_aim/image_raw`，按相同的原图时间戳和 `frame_id`
查找对应图像，再校验宽高。缓存须覆盖推理和传输延迟，并有大小/时间上限；找不到
匹配图像时丢弃或报告该帧，不能拿最新图像精修旧检测结果。也可以使用精确时间同步，
但仍需显式校验坐标系和尺寸。单个结果话题应绑定单个图像源，避免多相机时间戳碰撞。

海康图像发布使用 Best Effort、depth 1。未来 C++ 图像订阅应采用兼容的
`SensorDataQoS`；检测结果发布和订阅的 QoS 应共同配置，队列保持有界。
时间戳、尺寸字段提供同步依据，本包不实现同步器或时钟校准。

## Python 使用示例

未来检测节点接收到 `image_msg` 后，按以下形式组装消息。示例不创建任何控制节点。

```python
from copy import deepcopy

from srm_aim_interfaces.msg import (
    ArmorDetection, ArmorDetectionArray, Keypoint2D,
)

result = ArmorDetectionArray()
result.header = deepcopy(image_msg.header)
result.image_width = image_msg.width
result.image_height = image_msg.height

# 假设 points 已恢复为原图像素坐标，并映射为 LT/LB/RB/RT。
armor = ArmorDetection()
armor.color = ArmorDetection.COLOR_UNKNOWN
armor.confidence = float(score)
armor.lt = Keypoint2D(x=float(points[0][0]), y=float(points[0][1]))
armor.lb = Keypoint2D(x=float(points[1][0]), y=float(points[1][1]))
armor.rb = Keypoint2D(x=float(points[2][0]), y=float(points[2][1]))
armor.rt = Keypoint2D(x=float(points[3][0]), y=float(points[3][1]))
result.detections.append(armor)
# 对同一帧的其他有效实例继续 append；零目标时保持空数组。
```

C++ 生成头文件为
`srm_aim_interfaces/msg/armor_detection_array.hpp`，类型为
`srm_aim_interfaces::msg::ArmorDetectionArray`。未来消费包需在 `package.xml`
声明 `<depend>srm_aim_interfaces</depend>`，在 CMake 中执行
`find_package(srm_aim_interfaces REQUIRED)` 并为节点添加该依赖。
Python 消费包同样需声明运行依赖。现有算法尚未消费新消息，所以本次不修改其依赖。

## 编译与验证

在 Ubuntu 22.04 / ROS2 Humble 环境中，从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon list --base-paths src
rosdep install --from-paths src/srm_aim_interfaces --ignore-src -r -y
colcon build --packages-select srm_aim_interfaces \
  --executor sequential --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 interface show srm_aim_interfaces/msg/ArmorDetectionArray
colcon test --packages-select srm_aim_interfaces \
  --event-handlers console_direct+
colcon test-result --test-result-base build/srm_aim_interfaces --verbose
```

只选择接口包即可验证 C++ 和 Python 消息生成，不需要海康 SDK、相机或串口。
如关闭测试，可使用 `-DBUILD_TESTING=OFF`；此时不需要测试专用的
`ament_cmake_gtest`、`ament_cmake_pytest`、`rclcpp`、`rclpy` 依赖。

测试覆盖 C++/Python 类型使用与序列化往返、同帧双目标、四点字段、颜色、原图 Header、
尺寸和空数组。它们不验证模型输出精度、PCA、PnP 或实际硬件控制行为。
构建命令本身不会运行已有控制节点，不需要开启 `enable_serial` 或 `enable_tx`。

本次已在 Ubuntu 22.04 / ROS2 Humble 容器（Python 3.10.12）中完成接口包构建、
`ros2 interface show` 检查，以及 2 个 C++ 和 3 个 Python 测试，全部通过。
实际构建、安装和日志输出位于仓库外。未重新构建依赖外部 MVS SDK 的相机包，
未运行旧算法测试、实际 Python→C++ 跨进程消息收发或硬件测试；这些不属于消息生成验证。

本次新增接口不改变现有 `/srm_aim/target` 等话题，不改变 Python 算法、海康采集或
上下位机 18/30 字节协议。新的 ROS 消息也不应直接当作串口帧发送。

## 第 5 阶段定位接口

新增 `ArmorLocation` / `ArmorLocationArray`，保留源图 Header，逐目标提供数字验证
开关、验证状态/相关分数、PnP 有效状态/原因、歧义、Pose、轴向深度及欧氏距离。
数组的 `optical_frame_id` 专门声明 Pose 的光学坐标系，不将原始 Header.frame_id
静默改名；输入索引仅在本帧内有效。必须检查 pose_valid 后再使用位姿或距离。
关闭数字验证时 digit=0、digit_verified=false、digit_score=-1，不表示数字 2 已识别。
详见[定位节点说明](../../docs/armor_locator.md)。原有三个检测消息字段保持不变。

`ArmorTrack` / `ArmorTrackArray` 为独立三维跟踪输出。位置与速度使用显式 `tracking_frame_id`，目前固定 `camera_optical_frame`，`self_motion_compensated=false`。Header 保留源图像；使用状态/观测/预测各自时间和有效标记，`input_index` 仅当帧内匹配有效。轨迹身份为 `(session_id, track_id)`；数字未经验证不标记为 2。详见 [跟踪文档](../../docs/armor_tracker.md)。

`ArmorLocation` 末尾新增独立 `position_valid`、`orientation_valid`、`position`、`position_reason`。旧 `pose_valid` 仍表示完整严格位姿；仅位置保留时旧 Pose/距离仍是无效占位、`ambiguous=true / reason=ambiguous_ippe` 不变。新位置仅在 `position_valid=true` 时使用，坐标系同 `optical_frame_id`。字段追加改变 CDR 布局，接口包和全部消息使用者必须一起重编译，不能混用旧二进制。当前跟踪器使用独立位置有效性与坐标，可接收仅位置观测；离线圆拟合仍只读取旧完整位姿。详见 [独立位置策略](../../docs/armor_locator.md)。
