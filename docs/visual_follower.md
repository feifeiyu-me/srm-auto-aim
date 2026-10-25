# 独立云台视觉跟随视线角诊断

`srm_visual_follower` 是 ROS2 Humble ament_cmake/C++17 包。输入 `/srm_aim/armor_tracks`（现有 `ArmorTrackArray`，Reliable/KeepLast 5）；唯一业务输出是 `/srm_aim/visual_follower/debug`（DiagnosticArray，深度 5）。不依赖旧控制节点，不发布 Float32 控制角，不连接串口，没有弹道、发射或控制开关。

## 角度与坐标系

输入必须明确为 `camera_optical_frame`，X 右、Y 下、Z 前。选定位置 `(X,Y,Z)` 后：

```text
camera_yaw   = atan2(X, Z)                # 向右为正
camera_pitch = atan2(-Y, hypot(X, Z))     # 向上为正
```

输出同时标注 rad/deg。这是相机视线角，不是云台绝对角，也不是旧控制器/电机的正负号约定。没有相机到云台重合的默认假设。

预留静态外参接口：`extrinsics_valid=false`（默认）、`reference_frame`、`translation_m[3]`、`quaternion_xyzw[4]`。只有用户明确确认外参、设置非空参考系名及合法单位四元数后才计算：

```text
p_reference = R_reference_from_camera * p_camera + translation_m
```

参考系约定 **FLU：X 前、Y 左、Z 上**；参考视线角为 `atan2(Y,X)` 和 `atan2(Z,hypot(X,Y))`，左/上为正。该约定与相机光学坐标不同，不能直接填 identity 来假设相机/云台重合。配置文件中的 identity 仅为未启用的占位值。外参启用但格式错误时拒绝启动；变换后目标在参考系后方等情况下，参考角无效，相机角仍可有效。

当前不做动态 TF/云台姿态插值，不具备电机零位、反馈或机械约束信息。因此**即使外参有效，也只输出参考视线诊断，`hardware_command_valid` 始终 false**。没有生成可直接使用的实机绝对命令。未确认外参时 `reference_los_valid=false`，参考角的零值只是无效占位。

## 轨迹、预测和选择

仅接受 `state_valid=true`、非 deleted 的轨迹，当前位置必须有限、正深度并在距离范围内；检查源 Header、状态和最近观测时间。不会使用失效轨迹的预测点。当前版本的跟踪链路适用于固定相机，此节点不增加自运动补偿。

预测位置需同时满足 `prediction_valid=true`、有限坐标/正深度/距离检查、预测时刻不早于状态时刻且不晚于配置的最大预测跨度，并且尚未早于当前 ROS 时间。满足时优先使用它，`position_source=predicted`；否则使用有效当前位置，`position_source=current`。当前位置仍是明确时间的历史状态，不会改写成“现在”；不在本节点再次外推。

20 ms 定时器会在预测过期时发布一次当前位置回退诊断；状态或最近观测超时后发布无效诊断、清除目标与角度。ROS 时间检查新鲜度，单调接收时间独立检查输入中断，因此 ROS 时间暂停也不会无限保留有效结果。乱序/重复源时间、错误坐标系、重复 ID、过量轨迹等输出无效状态；不会继续输出旧有效角。

可配置策略：

- `closest_to_axis`（默认）：初选光轴夹角最小的目标。
- `nearest`：初选欧氏距离最小的目标。
- `lowest_id`：初选 ID 最小的目标。

同分按 uint64 ID 排序，不依赖数组顺序。`sticky_selection=true` 默认保持已选 ID，只要它仍然有效就不因其他目标评分稍好而切换；无效/消失时重新选择。设 false 可每帧重选。身份必须结合 `session_id` 使用；会话或图像来源/尺寸变化重置选择。没有装甲板整车建模或旋转模型。

## 诊断字段

数组 Header 保留输入图像 Header。状态包含：目标/相机角/参考角有效性、`session_id`、`track_id`、位置来源、所用三维位置、相机与参考角（rad/deg）、原图/状态/最近观测/所用位置各自精确的 sec/nanosec、坐标系、角度约定和失败原因。

无效时 `target_valid=false`、角度为清零占位，不保留上一次角度；必须依据有效标记和时间读取。`hardware_command_valid=false`、`mode=diagnostics_only` 始终明确输出。节点不保存历史轨迹，只有一条有界选中轨迹快照；输入队列 5、默认最多 64 条轨迹。没有逐帧控制台打印。

## NUC 构建与启动

先保持原有视觉、定位、跟踪链路，不启动旧 aim_solver、aim_control 或 serial_comm：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to srm_visual_follower --executor sequential \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 run srm_visual_follower visual_follower_node --ros-args \
  --params-file "$(ros2 pkg prefix srm_visual_follower)/share/srm_visual_follower/config/follower.yaml"
```

默认没有有效外参，仅输出相机视线角。可另一个终端读取一次：

```bash
ros2 topic echo /srm_aim/visual_follower/debug --once
```

参数配置见安装后的 `config/follower.yaml`，全部只读，变更后重启。默认最近观测/状态/接收超时均为 0.30 s，允许源时间领先 0.02 s，最大预测跨度 0.50 s，最小深度 0.05 m、最大距离 50 m。这些是可配置保护初值，不是实测延迟或精度指标。若 NUC 推理延迟超过新鲜度限值，会明确拒绝，需先测量原图到接收的延迟。录包回放需使用正确 ROS 时钟；历史消息直接注入墙钟会因过期被拒绝。

诊断话题禁止重映射到 `/srm_aim/cmd_*` 或旧 yaw/pitch_error 话题，误配置时拒绝启动。程序始终没有控制发布者；不添加硬件控制授权逻辑。

## 测试

```bash
colcon test --packages-select srm_aim_interfaces srm_visual_follower --event-handlers console_direct+
colcon test-result --verbose
ROS_DOMAIN_ID=85 ROS_LOCALHOST_ONLY=1 python3 src/srm_visual_follower/test/ros_smoke.py
```

C++ 测试覆盖视线角符号、预测选择/到期回退、确定性/保持选择、会话切换、无效和异常数值、时间戳/新鲜度/时钟回退、容量/重复 ID、外参变换及非法参数。ROS 跨进程测试启动真实 C++ 节点与 Python 合成轨迹发布者，使用安装后的 YAML，检查来源/角度/时间、超时、外参模式、无控制发布者以及危险重映射拒绝。只在隔离合成测试中使用已知测试外参，不在实机启用未经验证外参。

没有验证真实 NUC 跟随性能、外参精度或实机绝对云台角度。下一步应先离线/静态对照图像目标方向与诊断角度，再独立验证时间同步、真实外参和反馈语义；本节点不执行该硬件阶段。

本次云端验证（2026-10-25）：Ubuntu 22.04 / ROS2 Humble 构建成功，无编译警告；新节点 C++ 12 项、现有接口 C++/Python 9 项全部通过（colcon 含包装测试汇总 24，0 errors/failures/skipped）。真实跨进程 ROS 测试通过安装后的 YAML 启动，覆盖相机/参考角、预测回退、选择保持、无效状态、目标/输入超时，以及无控制发布者与危险重映射拒绝。日志：`/workspace/.srm-cloud/follower-validation.log`、`follower-ros-smoke.log`。没有在实机运行或启用外参，没有验证 NUC 动态跟随性能。
