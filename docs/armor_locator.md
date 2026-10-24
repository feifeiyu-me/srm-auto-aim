# 第 5 阶段：数字 2 验证与相机光学坐标 PnP

独立 C++17 / ament_cmake 包 `srm_armor_locator`。复用 `srm_armor_refiner::FrameMatcher`
和四点几何检查，不启动精修节点、YOLO、相机或任何控制节点。不修改旧 Python 算法、
18/30 字节协议或已有消息；只向接口包增加定位消息。

## 输入与已确认几何

默认输入 `/srm_aim/armor_detections_refined`，可通过 `detections_topic` 选择
`/srm_aim/armor_detections`。原图为 `/srm_aim/image_raw`。严格匹配完整源时间戳、
frame_id 和宽高；两种到达顺序均可。图像缓存 20 帧/96 MiB，检测缓存 20 条，
接收后 TTL=1 秒，100 ms 定时清理。不能匹配就丢弃并诊断，绝不使用最新图替代。
图像 Best Effort depth 1，检测 Best Effort depth 10，可接收 YOLO/精修的 Reliable 发布。

物体点严格为 LT/LB/RB/RT：
`(-w/2,-h/2,0), (-w/2,h/2,0), (w/2,h/2,0), (w/2,-h/2,0)`。
默认 `width_m=0.135`、`height_m=0.058`，对应用户确认的左右灯条中心距和发光长度，
不是装甲板外框尺寸。不自动猜测大/小装甲板，改变规格必须显式更改尺寸配置。

## 显式标定

`calibration_file` 默认空，必须提供绝对路径；没有默认相机内参或替代文件。
校验：正图像尺寸、3×3 有限 K、正 fx/fy、支持的无 skew 矩阵形式、图像内主点、
畸变数组尺寸和有限性。支持 plumb_bob 的 4/5 系数或 rational_polynomial 的 8 系数，
其他模型/不完整配置启动失败。运行图像尺寸与标定不同也拒绝该帧，不自动缩放内参。
支持 bgr8/rgb8/mono8/bgra8/rgba8，解码失败诊断且不输出定位数组。

本项目指定 `calibration/20261019_182722/camera.yaml`，1440×1080；用户确认采集标定时
与运行时均使用旋转 180°后的相同 ROS 图像方向。不要再次旋转或先去畸变再传入本节点。
配置合法性检查不等于证明物理标定准确，换镜头、ROI、方向或采集配置后仍须复核。

## 数字 2：相关性验证，不是分类概率

模板是已有 `digit_2_template.png` 的逐字节副本，随本包安装至 share/templates，
通过 ament 索引定位；可用 `template_file` 覆盖。启用验证时模板缺失或为常量图则启动失败。

使用四个语义角点做透视归一化至 96×96，保持 LT/LB/RB/RT 映射，再取中央 68×96 ROI。
复用原来的多尺度/横向压缩和 `TM_CCOEFF_NORMED` 思路，保留中心搜索范围限制。
`digit_threshold` 默认 0.65，要求在 (0,1] 内。分数是非负最佳模板相关分数，不是概率；
其他数字、字体、曝光和透视下的误判率需要真实正负样本另行测量。

`enable_digit_verification=false` 用于独立验证 PnP：每个结果都明确
`digit_check_enabled=false, digit_verified=false, digit=0, digit_score=-1`。
即使 PnP 成功也只返回 `ok_digit_unverified`，不宣称数字 2 已识别。
启用时只有匹配通过才设置 digit=2；数字通过但 PnP 失败会保留识别状态并说明 PnP 原因。

## PnP 和歧义

使用 OpenCV `SOLVEPNP_IPPE`，检查有限位姿、所有模型点正深度、t_z 范围和
像素重投影 RMS；在通过这些检查的候选中排序。若前两解重投影误差接近、且旋转或平移
显著不同，则拒绝为 `ambiguous_ippe`，不任意选择。几乎相同的重复解不作为姿态歧义。

| 参数 | 默认值 |
| --- | --- |
| `min_depth_m/max_depth_m` | 0.1 / 20.0 |
| `max_error_px` | 3.0 |
| `ambiguity_gap_px` | 0.1 |
| `ambiguity_translation_m` | 0.01 |
| `ambiguity_angle_deg` | 5.0 |

歧义阈值是保守工程门限，不是置信概率，可能拒绝远距离/近正视姿态。当前没有时序跟踪
辅助消歧，不应用减少门限来掩盖多解问题。参数均为启动时只读。

## 输出

`/srm_aim/armor_locations`：`srm_aim_interfaces/ArmorLocationArray`。

- Header 保留原图 stamp 和 frame_id，另有 `optical_frame_id` 专门声明 pose 坐标系，
  默认 camera_optical_frame，X 向右/Y 向下/Z 向前，指与本次图像和标定一致的光学坐标。
  这只是显式坐标约定，不会发布 TF，也不完成旋转后虚拟光学系到物理安装系的变换。
- 每个输入目标保留一条结果，包括失败目标；`input_index` 是本帧索引，不是跟踪 ID。
  包含原检测、数字开关/状态/分数、pose_valid、ambiguous、原因、Pose、深度和距离。
- `axial_depth_m = t_z`；`distance_m = norm(tvec)`，单位米。前者轴向深度，后者到模型
  中心的直线距离，都相对于相机光心，不能称为枪口距离或云台坐标。
- pose 是模型到该光学坐标系的变换。仅在 pose_valid=true 时使用；无效结果距离为 0、
  重投影误差为 -1（若已算出歧义候选，可能保留该误差），不能继续使用上一帧位置。
- 空输入发布带原 Header/尺寸的空数组。无法配帧或解码时不发布伪造的空结果。

`/srm_aim/armor_locator/debug`：DiagnosticArray，保留源 Header，逐目标记录原因和
数字/位姿状态。配置错误非零退出；模板验证失败、无有效解或歧义等是逐目标失败。
模型尺寸、阈值及标定应与采集记录一并保存；小重投影误差不等于真实距离准确。

## NUC 构建与运行

在仓库根目录执行；只启动定位节点，不启动控制、串口或发射节点：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/srm_aim_interfaces src/srm_armor_refiner \
  src/srm_armor_locator --ignore-src -r -y
colcon build --packages-up-to srm_armor_locator --executor sequential \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 run srm_armor_locator armor_locator_node --ros-args \
  -p calibration_file:="$PWD/calibration/20261019_182722/camera.yaml"
```

与原始 YOLO 对比时改 `-p detections_topic:=/srm_aim/armor_detections`。
独立 PnP 调试加 `-p enable_digit_verification:=false`，此时所有输出都明确未验证数字。
使用 `ros2 topic echo /srm_aim/armor_locations` 和 `/srm_aim/armor_locator/debug` 检查结果。
不要运行 aim_solver_node、aim_control_node 或 serial_comm_node，不将输出 remap 到控制话题。

## 测试与边界

```bash
colcon test --packages-select srm_aim_interfaces srm_armor_refiner srm_armor_locator \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
python3 src/srm_armor_locator/test/ros_smoke.py \
  "$PWD/calibration/20261019_182722/camera.yaml"
```

C++ 测试使用已知位姿合成投影、畸变、旋转、错误几何、深度/重投影门限、歧义判定，
以及模板合成正样本、均匀/随机负样本和透视变换。回归运行接口和精修包的配帧测试。
ROS 测试在 localhost/domain 79 启动真实 C++ 程序，验证两种输入选择、逐目标结果、
数字验证开启/关闭、原始 Header、多目标、空数组、未配帧丢弃及正常退出。
需保证该测试 domain 没有其他节点。大图 DDS 传输如丢失，先检查本机中间件配置；
本云环境使用 Fast DDS 16 MiB 共享内存段进行跨进程大图测试，未改节点算法。

这些测试不使用工业相机、电机或发射机构。合成投影只能证明实现一致性，模板合成样本
不能代表实机识别准确率。实际距离需用独立量具和明确光心/模型中心参考点验证，
覆盖距离、倾角、曝光及运动状态，并分别对比原始和精修角点；不宣称实际精度提升。

本轮 Ubuntu 22.04 / ROS2 Humble 构建通过：新增 8 项定位 C++ 测试、20 项精修包测试和
5 项接口测试均通过。上面的真实跨进程 ROS 测试通过，缺少显式标定和缺少模板的
标准入口测试均按预期以状态 1 拒绝启动。未进行 NUC 实机精度或真实数字数据集评估。
