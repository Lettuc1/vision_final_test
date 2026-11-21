# ft_tracker：单目标匀速平移跟踪 + 提前量

本包对应考核基础部分第 6 项：**单目标平移跟踪 + 提前量**。
已完成滤波、锁定、丢检预测、轨迹删除，以及 `position_predict` 和 RViz marker。

## 输入输出

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/ft/detection` | `ft_detection/DetectionFrame` | 检测节点输出 |
| 发布 | `/ft/track` | `ft_tracker/Track` | 单目标跟踪结果 |

`Track.msg`：

```text
std_msgs/Header header
uint32 id
bool tracking
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Point position_predict
float64 lead_time
```

- `header.stamp` 直接使用输入检测帧的时间戳，不使用 `ros::Time::now()`；
- `header.frame_id` 使用输入检测帧的相机 frame；
- `position` 是相机系下的滤波位置，单位米；
- `velocity` 是相机系下的滤波速度，单位 m/s；
- `tracking=true` 表示当前帧有观测命中；`tracking=false` 表示纯预测；
- `position_predict = position + velocity * lead_time`，单位米；
- `lead_time` 来自 YAML，默认 0.1 s。

## 滤波模型

状态：

```text
x = [x, y, z, vx, vy, vz]^T
z = [x, y, z]^T
```

使用 OpenCV `cv::KalmanFilter` 的匀速模型：

```text
A = [ I   dt*I ]
    [ 0    I   ]

H = [ I  0 ]
```

`dt` 由两帧 `header.stamp` 相减得到，并夹在 `min_dt ~ max_dt` 之间，避免回放暂停或时间跳变把滤波器推飞。

过程噪声使用白加速度模型，每个轴：

```text
Qpp = dt^4/4 * q_accel^2
Qpv = dt^3/2 * q_accel^2
Qvv = dt^2   * q_accel^2
```

观测噪声 `R = r_position * I`。

参数取值理由：

- `q_accel = 4.0 m/s²`：按机器人短时间内可出现的最大加速度量级取，
  让滤波器在目标变速时还能跟上；
- `r_position = 0.0025 m²`：约等于 PnP 位置标准差 5 cm；
- `gate_distance = 0.5 m`：PDF 建议先给 0.3~0.5 m；100 Hz 数据下目标一帧
  位移很小，0.5 m 主要给深度/角点噪声留余量，同时避免一开始就锁错板；
- `max_innovation = 0.5 m`：第一版单帧 innovation 上限。严格调到 0.2 m 时，
  `red_4_with_detection` 的 tracking 比例会降到 70% 左右并频繁重捕获，
  所以当前和 gate 保持一致，优先保证轨迹维持；如果实车检测框很稳，可以
  下调到 0.2~0.3 m 抑制跳变。

提前量按匀速模型外推：

```text
position_predict = position + velocity * lead_time
```

同时向 `/ft/debug/marker` 发布：

- 绿色球：当前位置（滤波后的装甲板中心）；
- 橙色球：提前量后的预测位置；
- 黄色箭头：当前位置 → 预测位置，方便在 RViz 里看出“往前一截”。

marker 生命周期设为 0（不自动过期），这样 `rosbag play` 暂停后还能截图；
轨迹超过 `max_lost_time` 删除时会主动发送 DELETE，把旧 marker 清掉。

## 锁定和丢检

1. 没有轨迹时，选当前帧中 `confidence` 最高的检测，初始化轨迹；
2. 已有轨迹时，只和**预测位置附近** `gate_distance` 内的检测关联，保持单目标锁定；
3. 关联不到：
   - 继续 `predict`，发布 `tracking=false`；
   - 连续丢失超过 `max_lost_time` 后删除轨迹；
   - 之后出现的检测重新初始化，分配新的 `id`；
4. 关联门限先取 0.5 m，后续可根据帧率和目标速度调整。

当前不要求跨装甲板换面保持同一车体 ID，也不估计车体旋转，符合基础部分要求。

## 参数

全部在 `config/tracker_params.yaml`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `detection_topic` | `/ft/detection` | 检测输入 |
| `track_topic` | `/ft/track` | 跟踪输出 |
| `queue_size` | 50 | 订阅队列 |
| `gate_distance` | 0.5 m | 数据关联门限 |
| `max_lost_time` | 0.5 s | 连续丢失删除时间 |
| `q_accel` | 4.0 m/s² | 过程噪声加速度谱密度 |
| `r_position` | 0.0025 m² | 观测噪声方差（约 5 cm 标准差） |
| `initial_position_var` | 0.1 | 初始位置方差 |
| `initial_velocity_var` | 1.0 | 初始速度方差 |
| `min_dt` / `max_dt` | 0.001 / 0.5 s | dt 夹紧范围 |
| `lead_time` | 0.1 s | 提前量 T |
| `marker_topic` | `/ft/debug/marker` | 当前/预测位置 marker |

## 运行

只启动跟踪节点：

```bash
cd ~/vision_final_test
catkin_make
source devel/setup.bash

roslaunch ft_tracker track.launch
```

用带检测结果的 Bag 单独验证跟踪（推荐先这样，和检测解耦）：

```bash
rosbag play --clock -l /root/rosbags/red_4_with_detection.bag
rostopic echo /ft/track
```

接自己的检测：

```bash
roslaunch ft_detector detect.launch
roslaunch ft_tracker track.launch
rosbag play --clock -l /root/rosbags/red_4.bag
```

## 如何确认效果

现在可以直接看 `/ft/track` 的 `position_predict` 和 `/ft/debug/marker`。

### 1. 看话题有没有连续输出

```bash
rostopic hz /ft/track
rostopic echo -n 1 /ft/track
rostopic echo -n 1 /ft/debug/marker
```

重点看：

- `header.stamp` 是否等于对应检测帧的图像时间戳；
- `tracking` 是否在正常情况下为 `true`；
- `id` 是否只在长时间丢检后变化；
- `position_predict` 是否约等于 `position + velocity * lead_time`；
- marker 的 frame/stamp 是否与检测一致，绿色当前位置、橙色预测位置和黄色箭头是否可见。

### 2. 看位置/速度曲线

```bash
rqt_plot /ft/track/position/x:y:z /ft/track/velocity/x:y:z
```

曲线应该连续；检测短暂丢失时位置继续按匀速模型外推，而不是跳变或断线。

### 3. 用统计脚本做一次量化检查

```bash
# 终端 1：跟踪节点
roslaunch ft_tracker track.launch

# 终端 2：统计脚本，记录 25 秒
python3 tools/check_track.py --duration 25

# 终端 3：回放带检测结果的 Bag
rosbag play --clock -r 1 /root/rosbags/red_4_with_detection.bag
```

脚本会输出：

- `/ft/track` 帧数；
- `tracking=true/false` 比例；
- 轨迹 ID 列表和 ID 切换次数；
- 最长纯预测时长（应不超过 `max_lost_time`）；
- 位置步进速度、滤波速度的 p50/p95/max；
- `position_predict` 与 `position + velocity * lead_time` 的最大误差；
- marker 消息数量；
- NaN/Inf 数量。

建议的判据：

- 有检测时 `tracking=true` 比例尽量 >90%；
- 最长纯预测不超过 `0.5 s`；
- `non-finite samples = 0`；
- ID 切换只发生在长时间丢检后，不应逐帧跳；
- 速度和位置步进不应出现明显不符合实车运动的尖峰；
- 提前量误差应接近 0。

也可以把结果导出：

```bash
python3 tools/check_track.py --duration 25 --csv /tmp/track.csv
```

### 4. 遮挡/丢检测测试

回放带检测的 Bag 时，如果某段 `/ft/detection` 的 `detections[]` 为空：

- 跟踪节点应继续发布 `tracking=false`；
- 位置继续按匀速模型外推；
- 空检测持续超过 `0.5 s` 后轨迹删除；
- 再出现目标时分配新 ID。

注意：如果直接把检测节点停掉，`/ft/track` 没有新的输入帧，跟踪节点不会自己产生新消息；应使用带空检测帧的 Bag 来验证。

### 5. RViz 看 marker

```bash
rviz
```

- Fixed Frame 设为相机 frame（如 `camera_optical_frame`）；
- 添加 Marker 显示，话题 `/ft/debug/marker`；
- 绿色球是滤波后的当前位置，橙色球是提前量后的预测位置；
- Bag 回放时两个球应随目标移动，预测球在运动方向前方一小段。

## 当前验证结果

用 `red_4_with_detection.bag` 验证（约 20 s）：

```text
track messages       : 2470
tracking true/false  : 2281 / 189 (92.3% true)
track ids            : [1, 2, 3, 4]
id switches          : 3（均发生在长时间丢检后）
max lost duration    : 0.495 s
lead prediction error: max=0.000000 m
marker messages      : 7410（每个轨迹帧 3 个 marker）
non-finite samples   : 0
```

同时观察到：

```text
position step speed  : p95=6.21, max=29.05 m/s
filter velocity      : p95=3.44, max=11.40 m/s
```

这不是滤波器自身发散。`red_4_with_detection` 里同一帧经常同时给出两块板
（深度约 3.8 m 和 5 m 附近），且帧内 ID 按左右重排；目标在两块板之间切换时
会产生较大的位置步进。严格限制 innovation 会明显降低轨迹维持率，所以当前
优先保证“不频繁重捕获”；下一步可以通过外观尺寸/板号一致性再做关联，而不是
单纯收紧 Q/R。

## 下一步

1. 录 2~3 分钟演示视频，覆盖一条 launch、TF、`/ft/debug/image`、预测 marker 和动态调参；
2. 进阶部分再考虑神经网络和旋转模型。
