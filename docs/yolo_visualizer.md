# YOLO-Pose 实时可视化

`srm_aim_comm/yolo_visualizer_node` 是独立的 ROS2 Python 节点，仅订阅图像和检测结果。
不加载 YOLO/Ultralytics，不进行推理，不访问云台、串口或发射机构。绘制使用普通
OpenCV CPU/HighGUI 路径，不要求 CUDA 或独立显卡。

## 安装和运行

使用 ROS2 Humble / Ubuntu 22.04，桌面已登录并有可访问的 X11/Wayland 显示。
依赖已包含在 `srm_aim_comm/package.xml`：`sensor_msgs`、`srm_aim_interfaces`、
`rclpy`、`rcl_interfaces`、`cv_bridge`、NumPy、OpenCV，无需安装 YOLO extra。

```bash
source /opt/ros/humble/setup.bash
cd ~/srm_aim_ws                 # 改成实际仓库根目录
rosdep install --from-paths src/srm_aim_interfaces src/srm_aim_comm \
  --ignore-src -r -y
colcon build --packages-up-to srm_aim_comm --executor sequential
source install/setup.bash
ros2 run srm_aim_comm yolo_visualizer_node
```

相机和 YOLO 节点在其他终端按原方式运行；可视化节点不启动它们。
按 **q** 退出；Ctrl+C 也会清理窗口和 ROS2 节点。支持窗口可见性查询的后端也可通过
关闭窗口退出；部分 OpenCV 后端不支持该查询，此时请使用 q 或 Ctrl+C。
在 NUC 本机桌面运行时，无需设置 CUDA、OpenGL 或修改 YOLO 的 CPU 推理配置。
SSH 会话须自行配置可用的图形显示；没有显示环境会明确报错，而不是假装已经显示。

窗口默认 1280×960，可拖动缩放。1440×1080 相机图像使用原始像素坐标绘制，
窗口缩放仅改变显示大小，不修改检测消息或推理分辨率：

```bash
ros2 run srm_aim_comm yolo_visualizer_node --ros-args \
  -p window_width:=1440 -p window_height:=1080
```

## 显示内容

- 所有几何有效的装甲板：绿色四边形、LT/LB/RB/RT 四个具名角点、Blue/Red/Unknown
  和实例置信度。读取真实消息的 `lt/lb/rb/rt` 字段，不重排或猜测四点语义。
- `Det FPS (rx)`：最近约 2 秒内检测消息的接收频率，不是窗口刷新率，也不是模型
  内部 forward FPS；缓存保留最多 120 个到达时间点，停止接收后归零。
- `Targets`：实际绘制的目标数；`Invalid`：因非有限值、越界、无效置信度或退化几何
  被跳过的实例数，不修改原始消息。
- 原始图像时间戳（秒.纳秒）和 `frame_id`。
- 匹配到空数组：显示对应原始图像及 `No Target`，不沿用上一帧边框。
- 超过 `display_timeout_sec` 没有新的匹配结果：显示最后匹配图像的原始画面，去掉
  全部目标叠加并标记 `STALE`。启动后尚无匹配时显示等待提示。

## 精确匹配与缓存

匹配键为 **`(header.stamp.sec, header.stamp.nanosec, header.frame_id)`**。
还要求检测消息宽高与原始图像宽高完全一致。不采用最近时间匹配、近似同步或“最新
图像 + 上一次检测”。消息先到、图像先到均可；无法在缓存中匹配时丢弃该可视化结果。

图像以原始 ROS 消息缓存，仅在配对后解码，避免对所有相机帧进行无用转换。
保留最新一个待显示配对；单线程 ROS executor 的定时器负责 GUI 事件和绘制。
晚到的旧配对不会将显示画面倒退。同一 Header 身份重复的图像会被视为歧义并丢弃，
同一配对不重复显示。源图像应提供唯一时间戳；源时钟回退/回放重置后请重启本节点。

缓存过期按本机 `time.monotonic()` 的接收时间计算，不拿历史图像时间戳与当前墙钟
比较。GUI 定时器即使在没有新消息时也会清理过期缓存。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `image_topic` | `/srm_aim/image_raw` | 原始 `sensor_msgs/Image` |
| `detections_topic` | `/srm_aim/armor_detections` | `ArmorDetectionArray` |
| `image_cache_size` | `20` | 最多缓存图像数 |
| `detection_cache_size` | `20` | 最多缓存未匹配检测结果数 |
| `max_cache_bytes` | `100663296` | 图像消息有效载荷上限，96 MiB |
| `cache_ttl_sec` | `1.0` | 未匹配消息和去重记录的最长接收后保留时间 |
| `display_timeout_sec` | `0.5` | 画面进入 STALE 并去掉目标叠加的时间 |
| `refresh_hz` | `30.0` | GUI 刷新及缓存清理频率 |
| `window_name` | `SRM YOLO Pose` | 窗口名 |
| `window_width/height` | `1280/960` | 显示窗口大小 |

参数为启动时只读。1440×1080 BGR8 图像约 4.45 MiB，20 帧约 89 MiB；该上限不包含
ROS/DDS、GUI 自身内存和少量待显示/当前画面缓冲，不是整个进程 RSS 上限。检测缓存
和去重历史也有条目数及 TTL 上限。图像数量、总字节和 TTL 三个限制同时生效。
若处理延迟超过缓存覆盖范围，可适当增加数量和字节上限，但应以 NUC 实际内存为准。

## QoS

图像订阅为 Best Effort、depth 1，兼容现有海康 `SensorDataQoS`。
检测订阅为 Best Effort、depth 10，可接收现有 YOLO 的 Reliable 发布，也支持
Best Effort 发布。可视化允许丢帧，优先显示正确配对，不给检测链路增加可靠传输背压。
有界缓存不能恢复已经被 DDS 丢弃的消息，匹配失败时不会错误叠加。

## 测试与边界

不需要相机、模型或图形桌面的单元/ROS 回调测试（GUI 调用在这些测试中模拟）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/srm_aim_comm/test/test_yolo_visualizer.py -q
```

测试覆盖精确键、两种到达顺序、不能借用最新图像、尺寸不符、过期、数量/字节限制、
连续配对下去重历史的上限、多目标绘制、角点名称、颜色与置信度、No Target、STALE、
非法实例过滤、QoS、无显示时错误以及 q/关闭窗口退出请求。

还可在装有 Xvfb 的测试环境中启动实际窗口，用模拟的 1440×1080 图像和检测话题验证
真实 GUI/ROS 收发。Xvfb 只用于测试，不证明 NUC 显示驱动、工业相机或真实 YOLO 的
端到端性能。图形叠加会消耗一定 CPU，NUC 上应观察 `Det FPS (rx)` 的实际变化。

本轮 ROS2 Humble 构建、针对新增 Python 文件的 flake8 检查通过；31 项可视化测试与
45 项原 YOLO 离线回归全部通过。另在 Xvfb 中启动标准 `ros2 run` 入口，通过两个
进程的真实 DDS 话题验证了 1440×1080 双目标、空目标、检测先到、缓存旧图精确匹配、
约 17 Hz 输入和 STALE 清除叠加；实际 X11 q 键使进程以状态 0 退出。
本机大图 DDS 测试使用了 16 MiB SHM segment 的运行时配置。没有实体 NUC/海康相机
参与该测试，不能据此宣称真实设备上的延迟或 FPS；没有启动控制或串口节点。
