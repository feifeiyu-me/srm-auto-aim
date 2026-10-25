# 第 6 阶段：固定相机三维多装甲板跟踪

新增独立 `srm_armor_tracker`（ament_cmake/C++17），只订阅定位结果，输出轨迹和诊断。没有控制、串口、发射或弹道接口。原有 YOLO、PCA、PnP 和通信算法均不修改。

**仅支持固定相机。** 坐标是 `camera_optical_frame`：X 右、Y 下、Z 前，位置 m，速度 m/s。`self_motion_compensated` 始终 false。相机运动会进入估计速度，不能称为目标世界速度。本阶段也不把装甲板轨迹 ID 当成机器人 ID。

## 输入与输出

- 输入 `/srm_aim/armor_locations`，`ArmorLocationArray`，Reliable/KeepLast(5)，兼容现有 Reliable 定位发布端。
- 输出 `/srm_aim/armor_tracks`，`ArmorTrackArray`，Reliable/KeepLast(5)。
- 诊断 `/srm_aim/armor_tracker/debug`，`DiagnosticArray`，深度 5。
- 不订阅原图、不重复配帧、不覆盖定位结果。

数组 Header 完整保留输入原始图像 Header，不改成处理完成时刻。`tracking_frame_id` 单独标明位置/速度坐标系。`session_id` 是进程启动前缀与重置代数，跨重启/重置应丢弃旧会话的活动轨迹；`track_id` 在单进程内递增，不复用帧内 `input_index`。

每次输出是完整活动轨迹快照，外加当次删除事件；未出现的旧 ID 不应继续当作活动轨迹。

每条轨迹有 `state_stamp`、`last_observation_stamp`、位置、速度、6×6 行优先协方差、匹配状态、生命周期、数字验证状态与明确时间的预测位置。

- `state_valid=false`：删除事件或超出短时保持期限，不能使用其位置作为有效跟踪结果。
- `velocity_valid=false`：初始零速度不是实际速度；至少达到命中次数和观测时间跨度才可用。
- `prediction_valid=true` 时才使用 `predicted_position` 和 `prediction_stamp`。
- `matched_this_frame=false` 时 `input_index` 无意义，固定置 0；颜色、置信度、数字状态来自最近关联观测，时间见 `last_observation_stamp`。
- 删除事件只输出一次，保留最后状态的实际时间，不伪造当前观测。
- 输入中断使用最近源 Header 输出一次 `input_timeout` 删除事件，随后清空；恢复输入开启新会话。

## 算法与边界

状态 `[x,y,z,vx,vy,vz]`，观测位置，使用源图像时间戳的实际 dt。线性恒速 KF，离散随机加速度过程噪声每轴为 `sigma_a² * [[dt⁴/4, dt³/2], [dt³/2, dt²]]`。测量协方差初期为可配置 XY/Z 标准差平方；不是从像素重投影误差直接换算，参数未经真实运动基准标定。

更新使用 Joseph 协方差形式并对称化；检查状态有限、协方差有限/对称/半正定。异常轨迹被移除并报告 `numeric_reset`，不会发布损坏状态。后续观测可建立新的轨迹。

只有 `pose_valid=true`、`ambiguous=false`、正深度、有限位置以及合法颜色/置信度的观测参与关联。默认最多 64 个输入条目；超限整帧清空跟踪并报告，不按数组顺序偷偷截断。活动轨迹最多 32，超过容量不再新建并报告 `track_limit`。

按已知颜色约束、三维平方马氏距离和绝对距离门控，Hungarian 全局一对一分配，带未分配虚拟项。未知颜色不会擦除轨迹已有颜色。同一行或列最优与次优代价差不超过歧义门限时，保守阻止竞争更新及相关观测建轨。此策略可能暂时降低更新率，不能保证同色完全遮挡/交叉后永不换 ID。

生命周期：首次为 tentative；连续命中达到 3 次为 confirmed；未匹配为 coasting；默认距最后观测超过 0.20 s 后位置/预测失效，超过 0.30 s 删除。确认过的轨迹短时重新关联后恢复 confirmed，未确认轨迹必须重新累积连续命中。默认预测时间为状态时刻之后 0.10 s，最多允许配置 0.50 s；预测正深度与有限性也会检查。

重复及小幅乱序时间戳直接丢弃，不更新状态。向后跳至少 1 s 视为时间重置并开启新会话。单靠消息无法区分大幅迟到与时钟回退，此策略会安全丢弃旧轨迹；录包回放应重启节点或保证时间连续。向前间隔超过 0.50 s、图像来源/尺寸变化也重置。非 `camera_optical_frame`、空来源 frame_id 或零尺寸拒绝并清空状态。有效消息接收中断用单调时钟检测（重复/乱序消息不刷新期限），默认 0.50 s；不把历史图像时间与当前墙钟直接相减。

相机当前时间戳是主机收到缓冲的时刻，并非硬件曝光时间。第 6 阶段不补偿此延迟，也不实现自运动补偿。未来移动相机需额外验证姿态同步、相机外参及平移运动，不可直接复用云台角度当成完整变换。

## NUC 构建与运行

在仓库根目录、`dev/hybrid-cpp`：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to srm_armor_tracker srm_armor_locator --executor sequential \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 run srm_armor_tracker armor_tracker_node --ros-args \
  --params-file "$(ros2 pkg prefix srm_armor_tracker)/share/srm_armor_tracker/config/tracker.yaml"
```

保持现有相机 → YOLO → PCA 链路。定位节点单独启动，数字暂不验证：

```bash
ros2 run srm_armor_locator armor_locator_node --ros-args \
  -p calibration_file:="$PWD/calibration/20261019_182722/camera.yaml" \
  -p enable_digit_verification:=false
ros2 topic echo /srm_aim/armor_tracks
ros2 topic echo /srm_aim/armor_tracker/debug
```

跟踪器不提供数字验证开关，不修改定位结果。关闭上游验证时，输出保持 `digit_check_enabled=false`、`digit_verified=false`、`digit=0`，不会认定目标数字为 2。

**不要启动 aim_solver_node、aim_control_node 或 serial_comm_node。** 不需要云台或串口输入。所有参数只读，变更后重启；配置见 `config/tracker.yaml`，数值是工程初值，不是精度保证。

## 测试与评估

```bash
colcon test --packages-select srm_aim_interfaces srm_armor_tracker \
  --event-handlers console_direct+
colcon test-result --verbose
# 独立真实 C++ ROS 进程 + Python 发布/订阅进程；无需工业相机或模型
ROS_DOMAIN_ID=81 ROS_LOCALHOST_ONLY=1 \
  python3 src/srm_armor_tracker/test/ros_smoke.py
```

C++ 测试覆盖：静止/变 dt 匀速、速度初始化、预测时刻与误差、检测顺序、同色交叉保守保持、不同色重合、短时丢失恢复/删除、无效 PnP/NaN/异常跳点、空数组、重复/乱序/时钟重置、来源/尺寸/坐标变化、长间隔、中断、上限、协方差与 Hungarian 全局分配。跨进程测试使用真正的生成消息和 C++ 节点，检查同类场景、数字未经验证、原 Header、诊断及无控制发布者。

固定相机实机建议记录 `/srm_aim/armor_locations`、`/srm_aim/armor_tracks`、`/srm_aim/armor_tracker/debug`，依次测试静止、匀速、交叉、遮挡。统计处理耗时分位数、关联中断、人工标注的 ID 切换，以及具有独立运动基准时的预测误差。不能用视觉平滑程度代替速度/预测精度评估。NUC 实时性能、真实运动准确度、自运动补偿均不在合成测试结论内。

## 本次云端验证记录（2026-10-25）

环境：Ubuntu 22.04 / ROS2 Humble / OpenCV 4.5.4 容器；宿主为 AMD EPYC 9V74 虚拟 CPU（5 个可见逻辑核），不是 Intel NUC。没有连接相机、云台、串口或发射机构。

实际执行了上面的构建及测试流程，隔离构建/安装目录位于 `/workspace/.srm-cloud/tracker-{build,install}`；额外对 `srm_armor_locator`、`srm_armor_refiner` 运行了构建与 `colcon test --packages-select` 回归。

- Humble 构建成功，最终构建无编译警告。
- 跟踪 C++ 18 项、接口 C++/Python 7 项、精修 C++ 20 项、定位 C++ 11 项全部通过，共 56 个实际测试用例。colcon 汇总为 62（包含 CTest 包装测试），0 errors / failures / skipped。
- `python3 src/srm_armor_tracker/test/ros_smoke.py`：真实独立 C++ 节点与 Python ROS 进程全部断言通过，包含安装后的 YAML 配置启动。合成序列不能代表实物 ID 稳定性或实际预测精度。
- 启动负例：`measurement_xy:=0.0`、输入输出使用同一话题，均清晰报错并以状态 1 退出。
- `git diff --check` 通过。

云端原始记录：`/workspace/.srm-cloud/tracker-validation.log`、`tracker-ros-smoke.log`、`tracker-startup-check.log`。未在 NUC 上运行，也未使用真实运动标注评估速度或预测精度。
