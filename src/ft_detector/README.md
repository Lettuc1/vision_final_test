# ft_detector：传统装甲板检测节点

本包实现考核基础部分中的**图像接入、传统装甲板识别、PnP 位姿解算、TF 发布、
`/ft/detection` 消息发布和调试图发布**。跟踪、提前量以及多装甲板选择属于后续
的 tracker / selector 节点，不在本包内。

## 编译与运行

```bash
cd ~/vision_final_test
catkin_make
source devel/setup.bash

# 终端 1：启动检测节点，自动加载 config/armor_params.yaml
roslaunch ft_detector detect.launch

# 终端 2：回放 Bag（必须带 --clock，launch 中已设置 use_sim_time）
rosbag play --clock -l /root/rosbags/red_4.bag
```

需要图形界面时可以使用：

```bash
roslaunch ft_detector image_test.launch
```

它会同时启动检测节点、`rqt_image_view` 和 `rqt_reconfigure`。

`ft_detection` 是考核提供的消息包，已经放在 `src/ft_detection` 下，由
`catkin_make` 一起编译。

## 话题和消息

| 方向 | 话题 | 类型 |
|---|---|---|
| 订阅 | `/hk_camera/image_raw/compressed` | `sensor_msgs/CompressedImage`（通过 `image_transport`） |
| 订阅 | `/hk_camera/camera_info` | `sensor_msgs/CameraInfo` |
| 发布 | `/ft/detection` | `ft_detection/DetectionFrame` |
| 发布 | `/ft/debug/image` | `sensor_msgs/Image`（BGR8） |
| 发布 | `/ft/debug/mask` | `sensor_msgs/Image`（MONO8，可选） |
| 发布 | `/tf` | 相机坐标系 → `armor_<id>` |

`header.stamp` 一律使用输入图像的时间戳，不使用 `ros::Time::now()`。
`header.frame_id` 优先使用最新 `camera_info` 的 `frame_id`；如果整段数据没有
`camera_info`，才使用图像 header 或 YAML 中的兜底值。

## 检测定义和坐标系

采用考核中的**定义一**：四个点是两根灯条的四个端点。

```
       左上 TL -------- 右上 TR          装甲板坐标系：
          |                |              x：沿装甲板向右
          |     数字       |              y：沿灯条向下
          |                |              z：垂直装甲板向里（远离相机）
       左下 BL -------- 右下 BR          原点：两根灯条中心连线的中点
```

- `armor_small_width = 120 mm`、`armor_large_width = 235 mm`：两根灯条中心距；
- `lightbar_length = 55 mm`：灯条可见长度；
- 物体点先由毫米换算成米，再传入 `cv::solvePnP`；
- 图像点顺序固定为 TL、TR、BR、BL。灯条先按 x 坐标从左到右排序，因此顺序在
  帧间稳定，不会因为配对顺序变化而跳变；
- 先尝试 `SOLVEPNP_IPPE`，失败或误差大时再尝试 `SOLVEPNP_ITERATIVE`，取重投影
  误差更小的解。

以上尺寸都在 YAML 中；如果你的装甲板灯条中心距或灯条长度不同，直接改 YAML，
不要改代码。

## 旧版本的问题和本版本的修复

1. **灯条过曝、中心发白、轮廓空心**
   - 强光 LED 中心会过曝成白色，严格 HSV 阈值会得到两条彩色边或一个空心环。
   - 旧代码在矩形中心采样颜色，中心是黑的，于是真灯条被丢掉。
   - 本版本只把**紧挨彩色光晕**的白色过曝像素补回 mask，再对外轮廓做
     `findContours + drawContours(FILLED)` 填洞，因此空心灯条会变成实心灯条。

2. **地胶 / 地板反光被当成装甲板**
   - 反光颜色可能和灯条相似，但通常是漫反射，`V` 的 90 分位数明显更低；
   - 真灯条要求 `p90(V) >= min_bar_p90_v`，并且满足以下之一：
     - 红色：有足够的过曝白心比例 `min_white_ratio`；
     - 蓝色：有足够的高饱和蓝色核心 `min_sat_ratio`；
   - 配对时还检查长度、宽度、角度、灯条刚性、装甲板内部亮度和彩色泄漏；
   - 最后还会过滤“真实装甲板正下方的镜像反光”：如果下方检测和上方检测的
     x 位置、宽度都很接近，只保留上方真实装甲板。

3. **相邻装甲板的灯条被连成一个假装甲板（red_4 中偶发闪现）**
   - 同一根物理灯条可能被过曝白心切成两个连通域；如果两个碎片分别参与配对，
     就会出现“两个装甲板共用一根灯条”的闪现框。
   - `mergeDuplicateBars()` 会按中心距、灯条长度和包围框重叠率合并同一根灯条的
     碎片；`selectPairs()` 再次检查候选灯条是否与已用灯条属于同一物理灯条。
   - `pairCentrality()` 给“夹在左右灯条中间”的配对加分：真实前装甲板左右两侧
     到最近同色灯条的距离接近，而“左前 + 右侧”“右前 + 左侧”这种跨板配对的
     居中程度低。这个先验是解决 red_4 中两个相邻装甲板被连起来的关键。
   - 两个检测框重叠超过 `max_armor_overlap_ratio` 时，只保留分数高的一个。
   - 增加**高度、倾角和灯条长度硬约束**：`top_diff`、`bottom_diff` 都不能超过
     `max_endpoint_diff_ratio × 平均灯条长度`（默认 0.20），两根灯条中心高度差
     不能超过 `max_pair_center_dy_ratio × 平均灯条长度`（默认 0.35），配对中心线
     倾角不能超过 `max_pair_tilt`（默认 12°），两根灯条长度相对差不能超过
     `max_length_ratio_diff`（默认 0.20）。red_4 中真实前装甲板的两根灯条基本
     水平且等长；而“左前 + 右侧”“右前 + 左侧”的跨板配对、以及半截灯条硬凑的
     配对，会出现明显高度差、倾斜或长度差，直接在这一步被拒绝。
   - 灯条轴夹角上限保留 `max_pair_angle = 30°`，是为了允许透视下两根竖直灯条
     在图像中收敛；它和“配对中心线必须水平”是两回事。

4. **大 / 小装甲板判断**
   - 不再用固定的像素距离 `distance > 80`；
   - 使用 `灯条中心距 / 平均灯条长度` 与真实 `W / L` 的比值判断，远近都能用；
   - 小装甲板允许 `W/L × [0.65, 1.30]`，大装甲板使用更紧的独立范围
     `W/L × [0.85, 1.10]`，中间刻意留出间隔，再按误差最小的一类归类；
   - 上限必须收紧：透视/偏航只会让灯条间距**变小**，不会让间距/灯条长度比
     大幅变大。`blue_guard.bag` 中两根相距很远灯条的比值约为 6.8，直接拒绝；
     `red_1&3.bag` 中“左侧板 + 前左灯条”的跨板比值约为 3.2，低于大装甲板
     下限；“左车侧灯条 + 右车侧灯条”的跨车比值约为 5.0，高于大装甲板上限。
   - 对大装甲板还检查投影框的宽/高比：`max_large_box_aspect` 默认 4.70。
     跨车框约为 4.8~5.0，会被拒绝；宽高比超过 `wide_large_aspect_min`
     （默认 4.00）时，还要求大数字区域有足够证据
     `min_number_score_wide_large`（默认 0.08），否则也丢弃；
     对所有大装甲板还要求 `min_number_score_large`（默认 0.05），
     跨板块/跨车板的空背景内部数字分数接近 0，会被拒绝。

## 参数和动态调参

所有行为参数都在 `config/armor_params.yaml`，由 `launch/detect.launch` 通过
`$(find ft_detector)` 加载，没有写死绝对路径。

考核要求的一部分参数已经通过 `dynamic_reconfigure` 暴露，可以在线调：

```bash
rosrun dynamic_reconfigure dynparam get /image_receiver_node
rosrun dynamic_reconfigure dynparam set /image_receiver_node min_contour_area 200
rosrun dynamic_reconfigure dynparam set /image_receiver_node red_v_min 120
```

节点启动时会把 YAML 的值同步到 dynamic_reconfigure，所以“改 YAML + 重新
launch”和“拖滑块”两种方式都有效，并且以 YAML 为初始值。

调参顺序建议：

1. 先看 `/ft/debug/mask`：只把 `red_v_min` / `blue_v_min` 降到灯条实心为止；
2. 如果墙面、反光出现，优先提高 `min_bar_p90_v`、`min_white_ratio`、
   `min_sat_ratio`，不要为了“抓住反光”而降低它们；
3. 如果真灯条丢失，检查 `min_contour_area`、`min_bar_length`、`max_bar_tilt`；
4. 如果出现跨板配对，优先检查 `max_pair_tilt`（默认 12°）、
   `max_pair_center_dy_ratio`（默认 0.35）、`max_endpoint_diff_ratio`
   （默认 0.20）、`max_length_ratio_diff`（默认 0.20）、
   `pair_centrality_weight`、`pair_centrality_y_band` 和
   `max_armor_overlap_ratio`；
5. 如果出现“两根灯条隔得很远”的超宽框，检查
   `armor_width_upper_tolerance`（默认 0.30）、
   `armor_large_gap_lower_factor`（默认 0.85）、
   `armor_large_gap_upper_factor`（默认 1.10）、
   `max_large_box_aspect`（默认 4.70）、
   `min_number_score_large`（默认 0.05）；不要为了多检出而放大这些值，
   宁可漏检；
6. 大 / 小装甲板：调整 `armor_small_width`、`armor_large_width`、
   `lightbar_length`，`armor_width_tolerance` 一般保持 0.65 左右，
   `armor_width_upper_tolerance` 保持 0.30 左右；
   `armor_large_gap_upper_factor` 保持 1.10 左右。

## 调试图

`/ft/debug/image` 中显示：

- 黄色小点：所有通过质量筛选的灯条候选；
- 绿色四边形：最终装甲板的四个灯条端点；
- 红 / 绿 / 蓝箭头：装甲板 x / y / z 轴在图像上的投影；
- 标签：`#id RED/BLUE LARGE/SMALL 距离 n=数字证据分数`。

最终提交代码中没有使用 `cv::imshow` / `cv::waitKey`，调试图统一发到
`/ft/debug/image` 用 rqt 查看。

## 已知限制

- 传统 HSV 方法不能可靠读取装甲板数字，大数字只作为“这里像装甲板”的加分项，
  不作为 ID；
- `blue_guard.bag` 中远处目标只有几个像素宽，部分帧会漏检；这是“宁可漏检，
  不硬凑”的结果；
- 机器人快速旋转、两个装甲板在图像上严重重叠时，仍可能有个别帧出现多余检测；
  后续 tracker 可以用时序一致性进一步稳定；
- 如果现场装甲板灯条中心距 / 灯条长度与默认值不同，必须改 YAML 中的尺寸。
