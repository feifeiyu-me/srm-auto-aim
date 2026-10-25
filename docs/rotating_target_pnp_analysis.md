# 离线旋转靶 PnP 分析

脚本 `tools/analyze_rotating_target_pnp.py` 不依赖 ROS，不导入实时节点，不连接相机/云台/串口。输入是 `/srm_aim/armor_locations` 的完整 YAML 多文档日志，输出可分享的 JSON、CSV 与 PNG 图。

## 数据采集与运行

在运行已有视觉/定位链路的 NUC 上导出完整数组，不使用 CSV、`--field` 或截断数组：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /srm_aim/armor_locations --full-length > armor_locations.yaml
# 采样完成后 Ctrl+C；确保最后一个 YAML 文档完整。
```

相机必须固定，定位节点继续使用 `enable_digit_verification:=false`。不要启动云台控制、串口发送或发射节点。脚本不推断或修改数字验证状态。

可在另一台电脑离线分析，无需 ROS 环境；Python >= 3.10：

```bash
python3 -m venv /tmp/srm-pnp-analysis-venv
source /tmp/srm-pnp-analysis-venv/bin/activate
python3 -m pip install -r tools/requirements-pnp-analysis.txt
python3 tools/analyze_rotating_target_pnp.py \
  --input armor_locations.yaml \
  --initial-radius 0.21 \
  --output-dir ./rotation_analysis
```

输出目录内固定文件会在重复运行时覆盖。`0.21 m` **仅是优化初值，不是约束或精确真值**；实际拟合可以得到其他半径。脚本也不能通过与 0.21 的接近程度证明测距准确。

## 输入过滤与坐标约定

依据仓库实际 `ArmorLocationArray` / `ArmorLocation` 字段读取：

- 要求 `pose_valid` 为布尔 true、`ambiguous` 为 false；使用 `pose.position.x/y/z`，坐标有限且 Z > 0。
- `pose_valid=false` 的条目不会读取其零坐标。空数组和整帧 PnP 失败只计数，不作为原点参与拟合。
- 使用 `optical_frame_id` 说明三维坐标，不把源 Header 的 frame_id 当成定位坐标系。混合光学坐标系、图像来源或图像尺寸时拒绝分析，请先分割日志。
- 原始 Header 时间保留整数 sec/nanosec，图上使用相对最早采样时刻的秒数。重复时间戳文档仅保留第一次，并统计跳过数量；不要把不同采样会话拼接成同一日志。
- 缺少或无效的重投影误差/置信度不会删除有效三维坐标，只从相应相关性统计中排除；不会用零值伪造缺失评分。
- 使用同帧全部有效装甲板。CSV 中 `array_slot_not_track_id` 只是原文档内的位置，**没有读取 input_index，也没有物理装甲板 ID 或跨帧关联**。

所有观测等权；多目标同帧观测可能共享相机和角点误差，不能当作完全独立样本。几何模型是一个共同的 X-Z 圆，**还要求旋转轴近似平行于相机光学 Y 轴**。固定相机不自动保证这个条件；相机俯仰/横滚可能使 X-Z 投影接近椭圆，此时圆残差并不一定来自 PnP。报告同时保留 Y 坐标统计供检查，不自动拟合轴向或转换坐标系。

## 拟合与异常定义

1. 全量对照：所有可用三维坐标参与几何最小二乘圆拟合，保留离群点。
2. 鲁棒拟合：固定随机种子的三点 RANSAC，按径向残差门限寻找共识，再仅对共识做几何最小二乘并迭代重分类；不把半径限制在初值附近。
3. 默认径向门限 0.02 m，至少 6 个内点且不少于有效观测的 50%。这些是分析配置，不是已验证的传感器噪声或精度。
4. 默认深度一致性门限 0.05 m：根据拟合圆和观测 X，计算两条可能的 Z 分支，取距观测 Z 最近者的差。不会选择某块装甲板的“正确前后分支”。
5. 若 X 超出圆的横向范围，深度分支残差不可计算，单独计数/CSV 留空；不会用夹紧根号伪造深度预测。圆切线附近的分支计算敏感。
6. 图中异常为“径向离群 **或** 深度分支残差超限”；“正常”只表示通过当前模型门限，不表示测量是真值。深度原始范围本就会随旋转变化，不能把正常的 ±半径变化直接当成深度跳点。

可调整配置，保留不同配置的输出作对照：

```bash
python3 tools/analyze_rotating_target_pnp.py \
  --input armor_locations.yaml --initial-radius 0.21 --output-dir rotation_analysis_20mm \
  --radial-threshold 0.02 --depth-threshold 0.05 \
  --trials 1000 --seed 42 --min-inliers 6 --min-inlier-fraction 0.5
```

样本不足、严格共线、没有足够共识或迭代不稳定时返回非零状态，不输出一个伪造的成功半径。短弧或拟合 Jacobian 条件数较差时，报告 `uncertain`；这不是完整的不确定度估计。角度覆盖率只描述样本几何方位，四块静止装甲板也可能覆盖较大方位范围，不能把它解释为实际转过的角度。

## 输出与解读

- `report.json`：旋转中心 X/Z、拟合半径；全量与鲁棒拟合；径向残差的 RMS、MAD、分位数；原始 Z/Y 统计；径向/深度异常数量；过滤与失败原因计数；拟合条件数和局限说明。
- `observations.csv`：保留每个可用观测的原始文档/数组位置、精确时间戳、XYZ、评分、鲁棒圆残差及异常标记。包括鲁棒方法排除的离群点，方便复核。
- `analysis.png`：X-Z 散点与两种拟合圆、时间着色及时间标注、径向残差随时间、重投影误差/YOLO 置信度与几何残差的散点关系。异常用叉号区分。全量拟合圆很差时可能被观测范围裁剪，精确参数仍保留在 JSON。
- `depth.png`：Z 深度随原始采样时间的无连线散点，标出拟合圆深度范围。没有把相邻采样点连成同一装甲板轨迹。

相关性报告同时提供全体与径向内点的 Pearson/Spearman 系数，不输出因果结论。少于 5 个有效评分样本或某一变量为常量时，输出不可计算状态和 null，不伪造相关系数。高 YOLO 置信度或低重投影误差也不能单独保证三维定位准确。

**不实现角速度估计。** 输出 `angular_velocity.status=not_estimated_uncertain`：四块装甲板有 90° 周期对称性，当前没有可靠的身份关联和相位展开依据，不能从无身份点云制造角速度。实时四装甲板旋转跟踪器不在本次范围。

## 合成测试

```bash
python3 -m unittest discover -s tools/tests -p test_rotating_target_pnp.py -v
```

测试构造四块相差 90°、共享半径/中心的装甲板，改变数组顺序和 input_index，并注入离群点、失败零坐标、整帧无有效 PnP、无效评分和坐标。还覆盖初值不是半径约束、混合坐标拒绝、短弧不确定、退化/不足输入和 CLI 输出图表。测试不依赖 pytest、工业相机或 ROS。

真实日志尚未提供，因此目前只能验证合成数据与导出格式处理；不能宣称 NUC 实际圆心、半径、异常比例或预测精度已验证。

本次验证（2026-10-25）：Python 3.12.14、NumPy 2.3.5、SciPy 1.17.0、Matplotlib 3.10.8、PyYAML 6.0.3；9 项 unittest 全部通过，`py_compile` 通过，CLI 成功生成 JSON/CSV/两张 PNG 并检查图像。独立合成示例为 48 帧、168 个有效观测、48 个失败 PnP 条目和 6 帧无有效观测；识别出 6 个径向离群点，拟合中心约 (0.349882, 4.699963) m、半径 0.209976 m。这里只是已知合成几何的程序验证，不是实物精度。未在 NUC Python 3.10 环境或真实采样日志上执行；脚本使用 Python >=3.10 语法及兼容依赖接口。
