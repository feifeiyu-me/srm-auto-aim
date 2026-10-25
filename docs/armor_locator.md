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

## 数字 2 ROI 可视化调试（默认关闭）

当前流程保持不变：原图经 cv_bridge 转 mono8；LT/LB/RB/RT 分别映射至
(0,0)/(0,95)/(95,95)/(95,0)，透视图为 96×96；实际匹配 ROI 是 x=[14,82)、
y=[0,96)，即 68×96。没有额外二值化、直方图均衡、旋转或翻转。模板原文件为
40×42 RGB PNG，按灰度加载，高度试探 44/56/68/80/90、横向比例 0.6/0.8/1.0，
使用 TM_CCOEFF_NORMED，并约束匹配位置接近 ROI 中心。阈值仍为 0.65。
YOLO 的 0.85 与模板相关分数 0.43 并不矛盾，二者不是同一种评分。

只增加观察用可视化，不改变评分、裁剪或点序。启用方法：

```bash
ros2 run srm_armor_locator armor_locator_node --ros-args \
  -p calibration_file:="$PWD/calibration/20261019_182722/camera.yaml" \
  -p digit_debug_enabled:=true \
  -p digit_debug_output_dir:="$HOME/srm_digit_debug" \
  -p digit_debug_max_samples:=20 \
  -p digit_debug_interval_sec:=1.0
```

在新的 session 子目录中，每个样本最多生成 5 个文件：

- `_source.png`：参与计算的原图灰度视图，标注目标四点、原始 Header 和分数。
- `_perspective.png`：实际 96×96 矫正图，不加字、不改像素。
- `_roi.png`：实际参与匹配的 68×96 ROI，不加字、不改像素。
- `_comparison.png`：标注 Header/分数/阈值，显示 ROI 最佳位置、原模板、实际匹配
  图块和本次最佳缩放模板。面板为方便观察进行了显示缩放，原始缓冲保存在上述文件中。
- `_metadata.yaml`：完整原始时间戳和 frame_id、目标索引、四点、尺寸、分数/阈值、
  ROI 标准差、最佳匹配矩形、模板路径/尺寸与横向比例。无可用匹配时注明原因。

默认每进程最多 20 次保存尝试，至少间隔 1 秒，覆盖成功和失败识别；全局限量不是
每目标限量。多目标希望一起采样可设 interval=0，仍受总量限制。达到上限即停止写盘；
磁盘写入失败会停用该进程的保存功能并警告，不影响定位判断。每次重启产生新目录，
不覆盖旧文件，因此多次采集后仍需手动清理。仅当数字验证开启时允许启用此调试。

先检查数字是否被截断、矫正后是否倒置/镜像、字号和宽高比例是否与模板相符，再检查
背景和曝光差异、最佳位置是否受中心范围限制。相机整体旋转方向相同不代表矫正 ROI
和模板的数字方向必然一致。没有 NUC 的 ROI 图之前不能确定 0.43–0.45 的具体原因，
不能通过降低阈值掩盖问题。此功能保存可直接查看的 PNG，不增加常驻 GUI 或显示依赖。
调试写盘可能影响帧率，采样完成后恢复关闭；可与原有实时可视化节点同时使用。

本轮构建和 11 项定位 C++ 测试通过（新增 3 项调试测试）。真实 ROS 合成输入验证了
正/负样本可视化输出、原始 Header、数字验证开关和原有 PnP 路径；限额为 2 时实际
仅产生 2 组、共 10 个文件。PnP 实现保持逐字节不变，阈值仍为 0.65；尚未获得 NUC
低分样本，不能据此确定实机低分原因或宣称识别率提升。

## IPPE 双解歧义诊断（默认关闭）

定位拒绝逻辑与阈值保持不变：两个**通过原有有效性筛选**的候选按重投影 RMS 升序排列；当 `error_1 - error_0 <= ambiguity_gap_px` 且（相对旋转角 `> ambiguity_angle_deg` **或**平移向量间距 `> ambiguity_translation_m`）时，仍返回 `pose_valid=false / ambiguous=true / reason=ambiguous_ippe`。默认阈值仍是 0.1 px、5°、0.01 m。诊断不会把歧义结果转换为位置有效，也不会影响后续跟踪。

新增只读参数：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `ippe_debug_enabled` | `false` | 启用独立诊断，不改变定位输出 |
| `ippe_debug_interval_sec` | `1.0` | 单调时钟汇总周期，允许 0.1–3600 秒 |
| `ippe_debug_max_samples` | `2` | 每个汇总窗口最多保留多少条双解详情，允许 0–64；0 为仅统计 |

启用时发布 `diagnostic_msgs/DiagnosticArray` 到 `/srm_aim/armor_locator/ippe_debug`，深度 2；默认关闭时不创建此发布者、不采集候选轨迹。没有逐帧控制台日志或写盘。空闲无新 PnP 尝试时不重复发布。汇总借助现有 100 ms 定时器，实际周期可能受节点负载影响。

每条 `armor_locator/ippe_pair` 样本包含：

- 原始图像 `stamp_sec / stamp_nanosec / frame_id` 和当前帧 `input_index`。数组 Header 为该窗口最近一次 PnP 尝试的源 Header，查看历史样本必须使用样本自身的 Header 字段。
- `candidate_0/1_reprojection_error_px`：四个角点重投影 RMS（px）。0 是最低误差候选，1 是第二候选，不是 OpenCV 原始返回编号。
- `candidate_0/1_t_x_mm / t_y_mm / t_z_mm`：相机光学坐标下平移向量（mm）。
- `candidate_0/1_distance_mm`：各自欧氏距离（mm）。
- `delta_t_x/y/z_mm`：第二候选减最佳候选的向量差。
- `translation_separation_mm = ||t1-t0|| × 1000`：平移向量间距，**这是原有平移歧义判断使用的量**。
- `signed_distance_difference_mm = ||t1||×1000 - ||t0||×1000`，以及绝对值 `absolute_distance_difference_mm`。距离接近不保证三维位置或姿态接近，不可用它替代平移向量间距。
- `rotation_difference_deg`：`R0 * R1^T` 的相对旋转角度。
- `error_gap_px`、三个实际阈值、`error_gap_close`、`translation_exceeds`、`rotation_exceeds`。
- `translation_triggered / rotation_triggered` 及 `trigger=none / translation / rotation / rotation_and_translation`，说明是否实际触发拒绝。仅差异超限而误差差值不够接近时，仍是 `trigger=none`。

`armor_locator/ippe_statistics` 同时提供 `window_*` 和进程累计 `total_*`：PnP 尝试、接受、ambiguous_ippe、0/1 个有效候选、双解数、仅旋转触发/仅平移触发/两者触发、诊断计算错误、被限额省略的详情数。对全部可用双解统计误差差、向量间距、欧氏距离绝对差、旋转差异的 count/min/mean/max。计数单位为**目标的 PnP 尝试**，不是图像帧数；数字验证或几何检查提前拒绝的目标不计入。

每个窗口仅保留最先出现的 N 条详情，并非随机采样；其余仍参与全部统计。`samples_omitted` 非零意味着详情记录不完整，不代表这些目标没有参与定位。单解/无解只计数，不输出不存在的第二解；未出现双解时不输出伪造的均值。计数和详情均在进程重启后重新开始，内存不随帧数增长。

### NUC 运行与采样

在仓库根目录构建并保持已有相机、YOLO、PCA 链路：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to srm_armor_locator --executor sequential \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 run srm_armor_locator armor_locator_node --ros-args \
  -p calibration_file:="$PWD/calibration/20261019_182722/camera.yaml" \
  -p enable_digit_verification:=false \
  -p ippe_debug_enabled:=true \
  -p ippe_debug_interval_sec:=1.0 \
  -p ippe_debug_max_samples:=2
```

另一终端读取一次，避免持续刷屏：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /srm_aim/armor_locator/ippe_debug --once
```

需要保留数据时使用 `ros2 bag record /srm_aim/armor_locator/ippe_debug /srm_aim/armor_locations`，而不是持续打印详情。若希望覆盖约 10–16 条/秒的全部双解详情，可重启定位节点，将 `ippe_debug_max_samples` 设为 32，并检查 `samples_omitted`。统计始终涵盖全部 PnP 尝试。采样结束后先停止图像输入，等待超过一个诊断周期再结束定位节点，以保留最后窗口统计。

重点比较：`total_rotation_only`、`total_translation_only`、`total_both`，以及向量间距和旋转差异的分布。当前仍严格拒绝 ambiguous_ippe；这些诊断本身不能证明位置可安全使用，也不是实测定位精度。不要启动云台控制、串口发送或发射节点。

### 测试复现

```bash
colcon test --packages-select srm_armor_locator --event-handlers console_direct+
colcon test-result --verbose
ROS_DOMAIN_ID=82 ROS_LOCALHOST_ONLY=1 \
  python3 src/srm_armor_locator/test/ippe_ros_smoke.py \
    calibration/20261019_182722/camera.yaml
```

跨进程测试使用合成图像/角点和真实 C++ ROS 节点，分别运行默认关闭与启用诊断，比较全部定位消息，检查原始 Header、mm/度字段、双解统计和样本上限。大图像 DDS 传输受运行环境配置影响；不能将传输超时解释成算法通过。

本次云端验证（2026-10-25）：Ubuntu 22.04 / ROS2 Humble / OpenCV 4.5.4 构建成功、无编译警告；新增 4 项 IPPE 诊断测试与原有 11 项定位测试全部通过。真实跨进程测试通过：默认无诊断发布者，开启后有限额/限频、完整样本 Header、24 次尝试 / 12 次接受 / 12 次歧义计数正确；关闭和开启时 12 组定位消息字段完全一致。原始记录位于 `/workspace/.srm-cloud/ippe-validation.log` 和 `/workspace/.srm-cloud/ippe-ros-smoke.log`。测试使用合成输入，未连接任何控制机构，未验证 NUC 实测双解差异或定位精度。
