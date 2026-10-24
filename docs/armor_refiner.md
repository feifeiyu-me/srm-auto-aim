# 第 4 阶段：C++ PCA 四角点候选精修

新增独立 `ament_cmake` / C++17 包 `srm_armor_refiner`，复用现有
`srm_aim_interfaces/ArmorDetectionArray`。不修改 YOLO、Python PCA、PnP、相机、控制和
18/30 字节串口协议。节点只订阅视觉输入，发布新视觉结果与诊断，不访问电机或串口。
不依赖海康 SDK 或 Ultralytics，使用 OpenCV core/imgproc 和 cv_bridge。

**PCA 是候选算法：`refined` 仅表示候选通过当前检查，不能解释为精度提高。**
需要同帧真实标注和原始 YOLO 基线对比，才能评估角点误差是否下降。

## 数据流及同步

```text
/srm_aim/image_raw ------------------------+
                                          | exact Header + dimensions
/srm_aim/armor_detections ------------------+--> armor_refiner_node
                                                 |--> /srm_aim/armor_detections_refined
                                                 +--> /srm_aim/armor_refiner/debug
```

- 键为完整 `(stamp.sec, stamp.nanosec, frame_id)`；检测宽高还必须与图像一致。
  支持任一到达顺序，没有近似时间匹配，不把旧角点用于新图像。
- 原始图像缓存最多 20 帧、96 MiB 有效载荷，未匹配检测最多 20 条。接收后 TTL 默认
  1 秒，使用 `steady_clock`；100 ms 定时清理，不依赖源时间戳是否接近当前墙钟。
- 图像订阅为 SensorDataQoS / Best Effort、depth 1；检测订阅为 Best Effort、depth 10，
  可接收现有 YOLO 的 Reliable 发布。新检测输出为 Reliable、depth 1。
- 重复身份的图像视为歧义并丢弃，同一匹配身份不会重复处理。去重历史和待发送的内部
  丢帧诊断队列也有数量上限。源应提供每帧唯一的 Header 身份。
- 过期、尺寸不一致、无法解码或不支持编码时，不发布该帧检测数组，并输出原因。
  支持 `bgr8/rgb8/mono8/bgra8/rgba8`。相机比 YOLO 快时，部分未参与检测的图像正常过期。
- 有效匹配的空检测会发布相同 Header/宽高的空数组。输出顺序是配对完成顺序，不保证
  按源时间戳排序；消费者仍应按 Header 同步。
- 图像缓存字节限制不包含 DDS、当前灰度图、PCA 工作区及诊断消息，不是进程 RSS 上限。

## 候选算法与回退

每个装甲板独立处理，四点始终为 **LT、LB、RB、RT**：

1. 检查 YOLO 原始四点有限、在图像内、边长/面积有效，严格凸且绕序正确。
   无效原始几何不能安全回退，因此丢弃该目标；无效类别/置信度同样丢弃并记录。
2. 左灯条由 LT→LB 初始化，右灯条由 RT→RB 初始化。建立定向窄 ROI，转灰度后
   检查亮度对比度，采用 Otsu 分割并提取连通域。越界或过大的 ROI 回退。
3. 拒绝像素不足、触碰 ROI 边界的亮区；使用 PCA 最大特征向量求主轴，检查细长度、
   相对初始轴夹角、中心偏移和长度。近似同等候选存在歧义时回退。
4. 主轴符号与原始 top→bottom 方向对齐，使用亮像素中心沿轴投影的极值估计两端。
   不按屏幕 y 排序，旋转/倒置时也保留语义。此方法不是光学边界的标定测量，阈值、
   运动模糊和噪声都可能造成端点偏差。
5. 两条灯均成功后组成 LT/LB/RB/RT，检查每个点的欧氏位移不超过
   `min(max_shift_px, max_shift_fraction × 较短原灯条长度)`，再次验证完整四边形。
6. 任一灯或最终检查失败，**整块装甲板四点全部回退为原始有效 YOLO 四点**，不混合
   半精修/半原始端点。各目标相互独立，回退不会阻止其他目标精修。

输出保留输入 Header、宽高、颜色和检测置信度，置信度不因 PCA 通过而提高。
丢弃无效目标后数组索引可能变化，通过诊断的 `input_index` / `output_index` 对应。
这些索引仅限本帧，不是跟踪 ID。

## 参数

所有参数为启动时只读，可用 `--ros-args -p name:=value` 调整。

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `roi_half_width_px` | 8.0 | 灯条初始轴两侧的 ROI 半宽 |
| `min_contrast` | 15.0 | 灰度 98% 与 10% 分位数最小差值 |
| `min_pixels` | 8 | 最小亮区像素数 |
| `min_light_length_px` | 8.0 | 最小可精修灯条长度 |
| `min_pca_ratio` | 4.0 | 主/次特征值比下限 |
| `max_axis_angle_deg` | 20.0 | 相对初始灯条轴的最大夹角 |
| `max_shift_px` | 8.0 | 端点最大位移，像素 |
| `max_shift_fraction` | 0.25 | 相对较短原灯条长度的位移比例上限 |
| `min_quad_area` | 16.0 | 四边形最小面积，像素平方 |
| `min_edge_length` | 2.0 | 四边形最短边，像素 |
| `max_edge_ratio` | 20.0 | 最大/最小边长比上限 |
| `max_roi_pixels` | 65536 | 单灯条 ROI 包围框面积上限 |
| `image_cache_size/detection_cache_size` | 20/20 | 未匹配缓存条数 |
| `max_cache_bytes` | 100663296 | 图像缓存有效载荷字节上限 |
| `cache_ttl_sec` | 1.0 | 未匹配/去重记录最长保留时间 |
| `detections_topic` | `/srm_aim/armor_detections` | 输入检测 |
| `refined_topic` | `/srm_aim/armor_detections_refined` | 独立输出；禁止解析后与输入同名 |

1440×1080 图像中灯条宽度较大时，默认 ROI 可能不足，会回退而不是使用被裁剪亮区。
只能在记录真实图像并检查结果后调整 ROI/阈值；扩大 ROI 也会增加误选亮区的风险。

## NUC 编译及运行

在现有 ROS2 Humble 工作区根目录执行，不需要启动控制或串口：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/srm_aim_interfaces src/srm_armor_refiner --ignore-src -r -y
colcon build --packages-up-to srm_armor_refiner --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source install/setup.bash
ros2 run srm_armor_refiner armor_refiner_node
```

已有相机和 YOLO 按原方式运行。查看诊断：

```bash
ros2 topic echo /srm_aim/armor_refiner/debug
ros2 topic hz /srm_aim/armor_detections_refined
```

诊断使用标准 `diagnostic_msgs/DiagnosticArray`，Header 为对应原帧。逐目标包含
`outcome=refined/fallback/dropped`、`message` 原因、输入/输出索引、原始/输出四点和
候选最大位移；没有完整候选时位移字段为 0，不代表测量误差为零。帧摘要包含各类计数和
`processing_ms`（解码、算法和构造消息的墙钟耗时，不含后续发布和 DDS 排队）。
常见原因包括 `left_low_contrast`、`right_insufficient_pca`、`left_component_touches_roi`、
`max_shift_exceeded`、`invalid_yolo_geometry`、`image_size_mismatch` 和
`unmatched_detection_expired`。图像正常跳帧产生的过期也可在诊断中看到，不会每帧打印警告。

在图形桌面的两个终端比较原始和精修结果（可视化节点须已构建）：

```bash
ros2 run srm_aim_comm yolo_visualizer_node --ros-args \
  -r __node:=yolo_visualizer_raw -p window_name:='YOLO Raw'

ros2 run srm_aim_comm yolo_visualizer_node --ros-args \
  -r __node:=yolo_visualizer_refined -p window_name:='PCA Candidate' \
  -p detections_topic:=/srm_aim/armor_detections_refined
```

需要定量比较时保存相同 Header 的原图、原始/精修检测及诊断；用人工标注的语义端点
分别统计误差、回退率、拒绝率和耗时。仅凭绿色边框更稳定或位移更小不能证明精度提高。

## 无相机测试

```bash
colcon test --packages-select srm_armor_refiner --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon test-result --test-result-base build/srm_armor_refiner --verbose
python3 src/srm_armor_refiner/test/ros_smoke.py
```

C++ 测试覆盖可精修亮条、35/100/180°旋转下语义顺序、低对比度/缺灯/PCA 信息不足/
过大位移/ROI 裁剪的整目标回退、非法原始几何丢弃和同帧多目标独立处理；另验证两种
到达顺序、精确历史图像、Header/尺寸不符、缓存限额/过期/重复身份。

`ros_smoke.py` 在隔离的本机 ROS domain 77 中启动真实 C++ 可执行文件，以合成 BGR8
亮条图和真实消息测试独立进程通信，包括同帧精修/回退/丢弃、空帧、先到检测、三种
同步不匹配、诊断原因和正常退出。测试图 480×240，不使用模型或硬件；不是 NUC 的
1440×1080 性能或真实标注精度验证。运行前请确保 domain 77 未被其他测试占用。

本轮在 Ubuntu 22.04 / ROS2 Humble 中构建通过，20 项 C++ 测试全部通过，
`ros_smoke.py` 的真实跨进程话题验证通过。还分别验证了通过参数和 ROS remap 将输出
指回 YOLO 输入时节点明确拒绝启动（预期退出状态 1）。没有进行 NUC 实机或真实标注
误差对比，不宣称 PCA 精度优于 YOLO。
