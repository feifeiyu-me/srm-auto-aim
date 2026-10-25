# 双轴视觉跟随：仅仿真控制包

新增 `srm_gimbal_follower`（ROS2 Humble / ament_cmake / C++17），包括独立控制节点、合成双轴云台/视觉源、隔离仿真 launch、YAML 和测试。它复用现有视觉诊断的角度/已预测位置，不重写 YOLO、PCA、PnP 或卡尔曼算法。

新增的独立 `manual_step_node` 不属于此自动仿真控制路径，人工硬件测试门控与操作见 [手动小步进说明](manual_gimbal_step.md)。下述无硬件发布者约束针对原仿真控制/仿真器节点。

**原自动视觉跟随节点始终只允许仿真。** `mode` 唯一合法值是 `simulation`；没有串口依赖，没有真实控制发布者，没有发射或弹道接口。即使 `calibration_verified=true`，`real_hardware_enabled=false`、`serial_tx_enabled=false` 也固定不变。输出话题不能重映射到仿真命名空间以外。

## 接口与默认保护

| 方向 | 默认话题 | 消息/含义 |
|---|---|---|
| 输入 | `/srm_aim/visual_follower/debug` | DiagnosticArray，严格解析现有 `visual_follower/line_of_sight` 契约 |
| 输入 | `/srm_aim/gimbal_state_stamped` | Vector3Stamped：x/y/z = yaw/pitch/roll，单位度，frame_id=`gimbal_feedback_degrees` |
| 仿真输出 | `/srm_aim/gimbal_follower/sim_reference` | DiagnosticArray，带有效标记的仿真 yaw/pitch 参考角，不是旧控制命令类型 |
| 诊断 | `/srm_aim/gimbal_follower/debug` | 同一控制结果、角度误差、限幅状态、输入时间、目标锁定和失败原因 |

输入和输出都是有界 QoS：视觉/参考/诊断深度 5、反馈深度 10，Reliable；反馈历史默认最多 200 条且不超过 1 秒。不会发布 `/srm_aim/cmd_yaw`、`cmd_pitch`、`cmd_fire`、`yaw_error` 或 `pitch_error`。

直接启动控制节点时，`calibration_verified=false` 且 `allow_unverified_simulation_calibration=false`，所以只输出无效仿真参考和 `calibration_not_verified` 诊断，不会假设已完成实机标定。专用仿真 YAML 才显式允许使用合成外参，该配置只属于合成场景。

## 视觉解析与时间

解析 `target_valid`、`camera_los_valid`、`mode=diagnostics_only`、`hardware_command_valid=false`、相机坐标系/角度约定、session/uint64 ID、`position_source`、位置、相机 yaw/pitch 和源/状态/最近观测/位置四组精确时间戳。重复键、缺失字段、无效数值、角度与位置不一致、错误时间关系均拒绝。不能把任意 DiagnosticArray 当成视觉指令。

`position_source=predicted` 时使用已有预测点，不在控制器里再次卡尔曼外推；`current` 使用当前选定点。预测超过其时刻加 0.03 s 后停止更新，等待新视觉消息。允许同一图像 Header 的 predicted→current 回退；重复/乱序消息不刷新目标新鲜度，不可用重放帧确认目标切换。目标失效标记立即清除控制输入。

反馈保存源时间的有界历史，用 `state_stamp`（图像/轨迹状态时刻）插值 yaw/pitch/roll，只在有包围样本且间隔不超过 0.10 s 时插值，也支持精确匹配；不向历史范围之外外推。预测位置仍表达在其图像状态的相机坐标基底中，所以姿态匹配使用 `state_stamp`，不是未来的 `position_stamp`。这是明确的坐标假设，不能代替自运动补偿。

现有串口反馈时间戳为**主机收到反馈时刻**，相机图像也不是硬件曝光时间；插值不能证明两者真实同步。ROS 时间用于源新鲜度，单调时钟独立检查接收超时/控制 dt。时钟回退清除历史、目标和参考；重复或乱序反馈不刷新新鲜度。反馈 NaN、异常跳变或过快变化使控制暂停，重新获得有效历史与视觉后才恢复。

## 相机到云台的几何与标定

不能把相机 yaw/pitch 直接当作绝对云台角。先用外参将视觉选定三维点变换到云台随动坐标系：

```text
p_body = R_body_from_camera * p_camera + t_body_from_camera
```

- 相机光学坐标：X 右、Y 下、Z 前。
- 云台随动坐标 FLU：X 前、Y 左、Z 上。
- `camera_to_gimbal_quaternion_xyzw`、`camera_to_gimbal_translation_m` 必须对应当前 ROS 图像/定位的坐标方向，包括既有图像旋转。
- 默认数值 `[-0.5, 0.5, -0.5, 0.5]` 和零平移仅描述**合成仿真**中的 optical→FLU 映射，不是 NUC 的实测安装外参。

反馈角到几何姿态：

```text
yaw_FLU = yaw_zero_deg + yaw_feedback_sign * feedback_yaw
pitch_up = pitch_zero_deg + pitch_feedback_sign * feedback_pitch
roll_FLU = roll_zero_deg + roll_feedback_sign * feedback_roll
```

默认 yaw sign=+1、pitch sign=-1，符合人工确认的“向右和向上时对应反馈角度都减小”。roll sign/零位仍是配置假设。使用图像时刻的姿态 `Rz(yaw) * Ry(-pitch_up) * Rx(roll)` 将 `p_body` 转到固定参考系，计算目标方向，再逆映射到反馈角度单位，最后对最新反馈执行控制。因此旧图像的偏差不会直接叠加在最新姿态上。

外参、反馈单位/轴向、零位、机械范围和时间同步仍需独立标定；设置 `calibration_verified` 只是用户声明，不是自动验证。**本版本没有任何参数可启用真实硬件控制**。

## 控制与目标保持

采用简单比例位置参考，默认 `kp=0.6`、死区 0.1°，不引入积分、MPC 或弹道。目标角先限制到可配置机械范围，再形成 `feedback + kp * error` 的参考。

每轴参考同时满足：

- 机械角度范围（合成初值 yaw ±170°、pitch ±45°）。
- 与当前反馈的最大偏差、与上一有效参考的最大步长，默认均不超过 1.5°。
- 相邻参考的角速度限制，默认 30°/s，使用实际单调 dt。

三者约束没有交集时停止更新，不突然追到反馈跳变后的位置。控制 tick 间隔超过 0.10 s 时暂停一拍，重置参考后从最新反馈平滑恢复。单次视觉修正超过 30°、无效外参变换或非正深度也停止更新。这些是仿真保护初值，不是已确认的实机机械限位。

按 `(session_id, track_id)` 锁定目标。新目标出现时先输出无效/保持，默认距离原目标最后有效观测至少 0.30 s，且候选 ID 连续保持至少 0.15 s、有至少两条推进时间戳的新观测，才允许切换。`allow_target_switch=false` 可禁止自动换目标。切换后以当前反馈重新建立受限参考，不能直接跳到新目标角度。

失去目标（默认 0.30 s）、反馈接收/源时间超时（默认 0.15 s）、非法输入时：`simulation_reference_valid=false`、`hold_requested=true`，输出零参考仅为无效占位。仿真器收到无效标记就保持**当前模拟反馈角**，绝不将零值当成回零命令；另有独立 0.10 s 参考消息看门狗。

诊断还包含 sequence、目标/参考/误差角（度）、dt、限幅、原始源时间、反馈时间、locked/pending ID、最后拒绝原因及标定来源。不逐帧打印控制台日志。

## 构建与隔离仿真

仓库根目录、`dev/hybrid-cpp`：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to srm_gimbal_follower --executor sequential \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

# 只启动新增控制节点和新增合成仿真器；不启动相机、旧控制或串口。
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 \
  ros2 launch srm_gimbal_follower simulation.launch.py
```

launch 自动把控制器的两个输入重映射到 `/srm_aim/gimbal_follower/sim_visual_debug` 和 `sim_state_stamped`，不会发布或读取真实反馈话题。同域另一个终端：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=86 ros2 topic echo /srm_aim/gimbal_follower/debug --once
```

仿真器是带 0.15 s 时间常数、60°/s 速率限制的合成位置伺服，不是 NUC 云台模型。它生成静止空间目标与随模拟相机运动变化的视线；不是通过真实图像、YOLO 或卡尔曼滤波得出的测试输入。默认目标在参考系右 10°、上 6°，期望反馈趋近 yaw=-10°、pitch=-6°。

可用仿真器参数注入故障/改变目标，例如在同一个隔离 domain 下：

```bash
ROS_DOMAIN_ID=86 ros2 param set /gimbal_follower_simulator publish_visual false
ROS_DOMAIN_ID=86 ros2 param set /gimbal_follower_simulator publish_visual true
ROS_DOMAIN_ID=86 ros2 param set /gimbal_follower_simulator publish_feedback false
ROS_DOMAIN_ID=86 ros2 param set /gimbal_follower_simulator publish_feedback true
ROS_DOMAIN_ID=86 ros2 param set /gimbal_follower_simulator invalid_feedback true
ROS_DOMAIN_ID=86 ros2 param set /gimbal_follower_simulator invalid_feedback false
```

仿真控制配置为只读，修改 `config/simulation.yaml` 后重启；仿真器的故障参数可动态设置。不要将仿真 YAML 的合成外参标记为实机已验证。

## 测试与边界

```bash
colcon test --packages-select srm_gimbal_follower --event-handlers console_direct+
colcon test-result --verbose
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 \
  python3 src/srm_gimbal_follower/test/ros_smoke.py --output-dir /tmp/srm_gimbal_simulation
```

C++ 测试覆盖协议字段/有效性/预测源、左右/上下符号、历史姿态插值、缺少对齐、外参/仿真门控、步长/速率/机械限幅、超时/异常反馈、目标切换与重放保护、预测过期与 current 回退、三维姿态变换、时钟回退和参考/反馈不连续。

跨进程测试实际启动安装后的 launch 和两个 C++ 节点，以 ROS 话题和参数服务验证正反两个方向的双轴闭环收敛、丢失/故障保持、ID 切换、相邻参考限幅和节点图无硬件发布者。输出 `summary.json` 和 `response.csv`（纯合成数据）；另外检查硬件模式和危险话题重映射都拒绝启动。

**尚未验证真实云台性能。** 现有三维跟踪器以固定相机为前提，移动云台时相机运动会进入相对速度；已有预测和未来位置的坐标基底也需进一步验证。本仿真不证明实际自运动补偿、外参、延迟、机械响应、稳定裕度或安全控制已完成。后续实机闭环必须单独设计/验收这些条件，本次不启用真实发送。

本次云端验证（2026-10-25，Ubuntu 22.04 / ROS2 Humble，非 NUC）：构建成功且无编译警告；13 项 C++ 用例全部通过（colcon 含包装测试汇总 14）。真实 launch 启动两个独立 C++ 进程，双轴稳定窗口内分别达到约 (-9.906°, -5.906°) 和 (7.904°, 3.904°)；227 对连续参考的步长/速率检查通过，最大参考速率约 30°/s。视觉丢失、反馈中断/NaN、ID 切换保持、危险模式/话题重映射拒绝均通过。日志为 `/workspace/.srm-cloud/gimbal-simulation-validation.log`、`gimbal-simulation-ros-smoke.log`；合成 CSV/JSON/响应图位于 `/workspace/.srm-cloud/gimbal-simulation-artifacts/`。CSV 同时保留反馈源时间戳与测试端接收时间。上述数值只验证合成模型，不是实机精度或响应性能。
