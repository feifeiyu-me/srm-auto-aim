# 独立实机小步进适配层（不开火，不接视觉）

`manual_step_node` 是独立 C++17 节点，不使用自动跟随器、仿真外参或目标 ID；只接受人工使能后的新相对角步进。现有 `serial_comm_node` 与 18/30 字节协议文件保持原样，串口节点每帧仍固定 `fire_flag=0`，适配器不发布 fire 话题。

## 首次测试必须知道的边界

- 代码只控制**新发送的目标参考**，不能保证实际电机不超调；实机机械限位、速度和急停行为必须由设备独立保证。
- **停止服务只是停止适配器发布。不是电机急停。** 串口可能继续发送缓存命令直到 `command_timeout_sec`；停止串口进程后，下位机仍可能保留并继续执行上一个目标。此协议没有明确的“取消目标/制动”字段。本节点不伪造停止帧，不发送零角度作为急停，也不自动回到启动角度。需要立即停止运动时使用设备的物理急停/断使能，并确认实际停止。
- 必须先确认下位机 18 字节帧中的 yaw/pitch 是**与反馈同单位、同坐标的绝对角目标**，不是增量、误差或速度。本仓库 Python 打包函数不能证明 MCU 端执行语义；未确认时 `absolute_target_protocol_verified=false`，不能人工使能。
- 必须人工确认当前安装的绝对机械安全范围。没有 ±180°/±20° 默认值。四个范围参数默认全 0（无效），`mechanical_limits_verified=false`。不要把本说明中的软件窗口当成实机已验证限位。
- 首次测试保持发射功能禁用，停止其他控制/串口进程，只使用一个独立 ROS domain 和一个串口进程。命令重映射到私有测试话题，旧视觉/控制输出不会直接进入此测试串口路径。

## 软件保护

冻结启动时首个稳定姿态作为锚点（至少 10 个反馈且跨度 >=0.2 s、波动 <=0.1°），在该节点生命期内不随运动、重连或重新使能重置。测试窗口硬限制为启动锚点 Yaw ±3°、Pitch ±2°，与人工配置的机械范围取交集。

默认 `allow_hardware_commands=false`。除启动许可、协议语义确认、机械范围确认外，还需有效新鲜连接/反馈状态、有效角度、单一发布者拓扑、稳定反馈和人工 enable 服务。使能本身**不产生命令**，还必须收到使能之后带新时间戳的人工 step。重放或使能之前排队的步进拒绝。

单次每轴最大 ±1°；建议首次只测一个轴 0.2°。步进相对请求时的实时反馈，目标超出窗口时拒绝而不是偷偷截断。上一运动未完成时不接受叠加请求。默认双轴参考向量速度 2°/s，参数必须 >0 且 <=5°/s（每个轴也不会超过该上限）。每 20 ms 按单调时钟限速，tick 间隔 >50 ms、反馈与参考偏差 >0.5°、反馈突跳、无效/超时（150 ms）、断连或其他发布者冲突立即撤销使能。

使能租约最多 5 s；运动最多 2.5 s；接近目标后保持短时间再自动撤销使能。停止后发送冷却至少 0.3 s，且反馈恢复稳定后才允许重新使能。任何故障都不会自动重新开始运动。重连不重置锚点。

私有命令是两个 Float32 话题，现有串口节点异步缓存这两个轴。因此适配器保证其参考轨迹的限速，**不宣称 ROS/串口是硬实时、两轴缓存原子更新或真实电机速度必然 <=5°/s**。这些能力需要 MCU 层另外验证。

## NUC 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to srm_gimbal_follower --executor sequential \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

以下每个终端都先 source ROS 和当前 install，并设置 `export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1`。不要同时运行仿真 launch 或旧控制/串口节点。

## 第一步：只读串口与反馈验证

终端 A：先找设备，**人工设置正确的 PORT，不使用默认猜测端口**：

```bash
ls -l /dev/serial/by-id/
read -rp '已确认的串口设备路径: ' PORT
SERIAL_REMAP=(
  -r /srm_aim/cmd_yaw:=/srm_aim/gimbal_step/cmd_yaw
  -r /srm_aim/cmd_pitch:=/srm_aim/gimbal_step/cmd_pitch
  -r /srm_aim/gimbal_state_stamped:=/srm_aim/gimbal_step/feedback
  -r /srm_aim/serial_connected:=/srm_aim/gimbal_step/serial_connected
  -r /srm_aim/feedback_valid:=/srm_aim/gimbal_step/feedback_valid
)
ros2 run srm_aim_comm serial_comm_node --ros-args \
  -p port:="${PORT:?请先设置串口}" -p enable_serial:=true -p enable_tx:=false \
  -p command_timeout_sec:=0.05 "${SERIAL_REMAP[@]}"
```

终端 B：默认只读适配器，不加任何发送许可：

```bash
ros2 run srm_gimbal_follower manual_step_node
```

终端 C：检查反馈单位、连接、新鲜度、启动锚点与异常状态：

```bash
ros2 topic echo /srm_aim/gimbal_step/feedback --once
ros2 topic echo /srm_aim/gimbal_step/debug --once
ros2 topic info /srm_aim/gimbal_step/cmd_yaw --verbose
ros2 topic info /srm_aim/gimbal_step/cmd_pitch --verbose
```

应看到稳定有限的 yaw/pitch/roll（度）、连接与反馈有效、`anchor_valid=true`、`armed=false`、`allow_hardware_commands=false`；未配置机械范围/协议确认时 `ready=false` 是预期行为。此阶段串口 TX 禁用，即使误发私有命令也不会发送串口帧。

## 第二步：准备人工使能（不自动移动）

确认 MCU 语义、现场机械范围和物理停止方式后，Ctrl+C 停止终端 A 的只读串口进程，在**同一个终端 A**用相同端口与重映射重启：

```bash
ros2 run srm_aim_comm serial_comm_node --ros-args \
  -p port:="${PORT:?}" -p enable_serial:=true -p enable_tx:=true \
  -p command_timeout_sec:=0.05 "${SERIAL_REMAP[@]}"
```

此时没有适配器命令，不应产生运动帧。终端 B Ctrl+C 停止默认适配器，输入现场已确认的**窄范围绝对限位**（不可直接抄其他机器的数值）：

```bash
read -rp '已确认 yaw 绝对安全下限(度): ' YAW_MIN
read -rp '已确认 yaw 绝对安全上限(度): ' YAW_MAX
read -rp '已确认 pitch 绝对安全下限(度): ' PITCH_MIN
read -rp '已确认 pitch 绝对安全上限(度): ' PITCH_MAX
ros2 run srm_gimbal_follower manual_step_node --ros-args \
  -p allow_hardware_commands:=true -p mechanical_limits_verified:=true \
  -p absolute_target_protocol_verified:=true \
  -p yaw_min_deg:="${YAW_MIN:?}" -p yaw_max_deg:="${YAW_MAX:?}" \
  -p pitch_min_deg:="${PITCH_MIN:?}" -p pitch_max_deg:="${PITCH_MAX:?}" \
  -p max_rate_deg_sec:=2.0
```

这会重新采集该次节点启动姿态；使能前再次检查 debug 的锚点和有效窗口。`ready=true` 仅表示基础反馈/范围门控满足；还应看到 `armable=true`（发送许可、稳定性、冷却等完整检查通过）。这些标记不代表 MCU/机械已由软件验证。

## 第三步：人工使能后只发一个小步进

终端 C，在设备可随时人工停止的情况下：

```bash
ros2 service call /srm_aim/gimbal_step/enable std_srvs/srv/SetBool '{data: true}'
# 必须确认 success: true；5 秒租约内发一个新的请求。以下 yaw=-0.2 表示反馈角减小。
ros2 topic pub --once /srm_aim/gimbal_step/relative_step geometry_msgs/msg/Vector3Stamped \
  '{header: {stamp: now, frame_id: relative_feedback_degrees}, vector: {x: -0.2, y: 0.0, z: 0.0}}'
```

Humble 的 `stamp: now` 自动填充发布时间。若 CLI 发现耗时导致租约过期或请求超过 0.2 s 新鲜度，节点拒绝并保持禁用；检查诊断、等待稳定后重新人工使能，不要通过放宽超时绕过。

首次先验证单轴方向与实际位移。按此前人工结果，yaw 负步进预期向右；pitch 负步进预期向上，但必须实测确认 MCU 命令语义。完成/自动撤销使能并稳定后才测试另一轴：

```bash
ros2 service call /srm_aim/gimbal_step/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic pub --once /srm_aim/gimbal_step/relative_step geometry_msgs/msg/Vector3Stamped \
  '{header: {stamp: now, frame_id: relative_feedback_degrees}, vector: {x: 0.0, y: -0.2, z: 0.0}}'
```

## 停止操作

```bash
ros2 service call /srm_aim/gimbal_step/stop std_srvs/srv/Trigger '{}'
# 等价的软件撤销使能：
ros2 service call /srm_aim/gimbal_step/enable std_srvs/srv/SetBool '{data: false}'
```

这是**停止适配器发送新目标**，不保证电机实际停止。之后 Ctrl+C 停止串口节点可停止串口写入；缓存可能持续至 0.05 s 超时加调度延迟，MCU 仍可能执行旧目标。需要立即制动时先使用物理急停/断使能，不能等待 ROS 服务返回。无明确停止能力或未验证 MCU 目标语义时，不进行真实运动测试。

## 自动测试（无真实设备）

```bash
colcon test --packages-select srm_gimbal_follower --event-handlers console_direct+
colcon test-result --verbose
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 \
  python3 src/srm_gimbal_follower/test/manual_step_pty.py
# 原仿真回归，脚本自行启动两节点，请勿与手动 launch 在同域同时运行：
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 \
  python3 src/srm_gimbal_follower/test/ros_smoke.py
```

PTY 测试创建自身伪串口，不接受真实端口参数。它使用现有真实 `serial_comm_node` 和协议代码，检查默认禁用、使能本身静默、小步进、停止后的缓存排空、越界拒绝、反馈丢失和全部 18 字节帧 fire=0。单元测试另覆盖硬窗口不重锚、最大速度参数和异常状态。通过这些测试不代表 NUC 实机运动或机械安全已验收。

本次云端验证（2026-10-25）：ROS2 Humble 构建通过；Python 打包提示跳过字节码编译，C++ 无编译警告。新增 10 项手动控制测试和原 13 项仿真测试全部通过（colcon 包含包装项共 25，零失败）。PTY 集成捕获 20 个真实编码的 18 字节帧，全部 fire=0；默认禁用、人工使能但未步进时静默、边界拒绝、停止缓存排空、反馈丢失和关闭 PTY 导致的串口断开保护均通过。测试日志中的 PTY EIO/断开报错是故障注入的预期结果。原双轴跨进程仿真回归通过。日志：`/workspace/.srm-cloud/manual-step-validation.log`、`manual-step-pty.log`。没有连接真实串口设备或云台；没有验证 MCU 实际语义、机械安全范围、实际速度或紧急停止效果。
