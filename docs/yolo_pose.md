# Python YOLO-Pose 检测节点

`srm_aim_comm/yolo_pose_node` 是独立入口，不替换 `armor_detector_node`。
本节点只订阅图像、发布 `srm_aim_interfaces/ArmorDetectionArray`；不发布云台命令，
不访问串口，也不启用发射。不要同时将两条检测链路的目标输出接入同一执行器。

## 模型要求

- 本地 Ultralytics YOLO11 Pose `.pt`，`task=pose`，`kpt_shape=[4, 3]`
  （四点，每点有 x/y/置信度），类别名称必须为 `{0: blue, 1: red}`。
- 默认路径 `~/srm_aim_ws/models/best.pt`，可通过 `model_path` 参数覆盖。
  缺失文件、加载错误、错误任务/点数/类别映射均导致启动失败并以非零状态退出。
  不下载默认权重，不回退到其他模型。
- LT/LB/RB/RT 指左灯条上/下端点和右灯条下/上端点，必须核对训练集标注。
  `corner_indices` 的四项分别是这些语义点在模型输出中的索引，默认 `[0,1,2,3]`。
  不按 x/y 大小重新排序倾斜目标。点序参数只能映射标注顺序，不能自动推断语义。
- 通用 `yolo11n-pose.pt` 通常是 COCO 人体模型（17 点、person 类），不能直接用于
  本节点。须先用装甲板四关键点数据训练得到兼容的 `best.pt`。
- 模型权重不入库，`.gitignore` 忽略 `models/` 和 `.pt`。仅加载可信来源的 PyTorch
  checkpoint；`.pt` 文件不是纯文本配置。

## 依赖与构建

ROS2 Humble / Ubuntu 22.04 / Python 3.10。已有 ROS 依赖由 `package.xml` 声明；
Ultralytics 作为可选 Python extra 安装，避免传统检测链路构建时被迫安装 PyTorch。
NumPy 限制在 `<2`，兼容 Humble 的 `cv_bridge`。不要在系统 Python 中升级 NumPy。
创建虚拟环境需要系统已安装 `python3-venv`，构建需要 `colcon`。

```bash
source /opt/ros/humble/setup.bash
cd ~/srm_aim_ws                  # 替换为实际仓库根目录
rosdep install --from-paths src/srm_aim_interfaces src/srm_aim_comm \
  --ignore-src -r -y
python3 -m venv --system-site-packages ~/venvs/srm-yolo
source ~/venvs/srm-yolo/bin/activate
python3 -m pip install --upgrade pip
# CPU 示例；GPU 环境按 PyTorch 官方说明安装对应版本。
python3 -m pip install torch==2.5.1 torchvision==0.20.1 \
  --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install './src/srm_aim_comm[yolo]'
# 使用虚拟环境的 Python 调用 colcon，让新入口脚本使用该环境。
python3 -m colcon build --packages-up-to srm_aim_comm --executor sequential
source install/setup.bash
ros2 run srm_aim_comm yolo_pose_node --ros-args \
  -p model_path:="$HOME/srm_aim_ws/models/best.pt" \
  -p confidence_threshold:=0.25 \
  -p keypoint_confidence_threshold:=0.5 \
  -p corner_indices:='[0, 1, 2, 3]' \
  -p device:=cpu
```

若此前用系统 Python 构建，激活虚拟环境不会改变已有入口的 shebang。
可以重新用虚拟环境 Python 构建，或显式使用：

```bash
ros2 run --prefix "$HOME/venvs/srm-yolo/bin/python" \
  srm_aim_comm yolo_pose_node --ros-args -p model_path:=/absolute/path/best.pt
```

本轮实际验证版本为 Ultralytics 8.4.185、PyTorch 2.5.1+cpu、TorchVision 0.20.1+cpu、
NumPy 1.26.4、OpenCV 4.11.0。可选依赖范围支持 Ultralytics `>=8.3.0,<9.0`；
范围内其他版本未逐一验证。

## 话题与参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `model_path` | `~/srm_aim_ws/models/best.pt` | 本地兼容权重 |
| `image_topic` | `/srm_aim/image_raw` | `sensor_msgs/Image`，Best Effort、depth 1 |
| `detections_topic` | `/srm_aim/armor_detections` | `ArmorDetectionArray`，Reliable、depth 1 |
| `confidence_threshold` | `0.25` | 实例最低置信度 |
| `keypoint_confidence_threshold` | `0.5` | 四个点均须达到的置信度 |
| `corner_indices` | `[0,1,2,3]` | 模型索引按 LT/LB/RB/RT 排列，须为 0..3 的排列 |
| `image_size` | `640` | 网络输入尺寸；输出仍使用原图坐标 |
| `device` | `cpu` | Ultralytics 推理设备 |
| `max_detections` | `100` | Ultralytics `max_det` 上限 |
| `min_quad_area` | `16.0` | 最小面积，像素平方 |
| `min_edge_length` | `2.0` | 最小边长，像素 |
| `max_edge_ratio` | `20.0` | 四条边最大/最小长度之比上限 |
| `warn_inference_sec` | `0.1` | 单帧解码、推理、转换、发布总耗时告警阈值 |

参数在启动时读取并设为只读；调整后重启节点。

## 每帧处理和异常约定

- 支持 `bgr8/rgb8/mono8/bgra8/rgba8`，通过 `CvBridge` 转 BGR8；其他编码明确报错。
  空数据、尺寸不匹配、解码异常均记录限频错误并丢弃该帧。
- 对每个收到且成功完成处理的图像发布一个数组，复制原图完整 Header、宽高；
  推理没有目标或所有实例被过滤时也发布空数组。
- 使用 Ultralytics `keypoints.xy`（已恢复到原图坐标），不再次反算 letterbox。
  校验 `orig_shape` 与输入图像一致。输出坐标为 float32，检查的是实际发布精度。
- 实例类别按 **0→COLOR_BLUE、1→COLOR_RED** 映射；实例和逐点置信度须有限且在
  `[0,1]` 内。四点须完整、有限、在图像边界内，且构成 LT/LB/RB/RT 对应方向的
  严格凸四边形；拒绝重复、共线、自交、凹形、过小和边长比例异常的四边形。
- 推理异常、张量结构错误或缺少关键点置信度，记录错误后丢弃该帧，不伪装成
  “正常推理但无目标”的空结果。单个实例不合格时仅过滤该实例。
- 保留图像时间戳，不替换为推理完成时间。慢推理触发限频告警，不重新打时间戳。
  单线程处理、输入队列深度 1，过载时 DDS 可能丢弃尚未处理的帧；不保证处理每个
  相机帧，不积累无限队列。当前不实现后处理图像同步或控制端超时策略。
- 数值/几何检查不能证明四点的训练语义正确，也不能替代模型精度和实车标定验证。

## 无工业相机测试

```bash
source /opt/ros/humble/setup.bash
source ~/venvs/srm-yolo/bin/activate
source install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=43 \
  python3 -m pytest src/srm_aim_comm/test/test_yolo_pose.py -q
```

测试使用模拟 Ultralytics 输出和真实 ROS 消息，包括一次实际图像话题到检测话题的收发。
不需要 Ultralytics、权重、相机、串口或电机；这不能视为真实模型推理通过。
测试覆盖多目标、空数组、颜色、Header、角点映射、逐点置信度、非法几何过滤、启动失败、
推理错误、编码/空图像和延迟告警。
本轮 45 项测试分别在无 Ultralytics 的系统 Python 和已安装 Ultralytics 的虚拟环境中
全部通过；新增 Python 文件的 flake8 静态检查通过（最大行宽 99）。

本轮上传的 `yolo11n-pose.pt` 实测为 `[17,3]` / `{0: person}`。其通用人体模型 CPU
前向推理在仓库测试图片上完成（零人体检测）；本节点启动时明确拒绝该模型。
没有提供兼容的装甲板 `best.pt`，所以**装甲板真实模型推理尚未验证**。
测试结果和通用人体模型推理不能说明装甲板检测精度。
