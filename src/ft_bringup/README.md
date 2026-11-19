# ft_bringup：一条 launch 跑通检测 + 跟踪 + 可视化

## 启动

```bash
cd ~/vision_final_test
catkin_make
source devel/setup.bash

roslaunch ft_bringup bringup.launch
```

另一终端回放：

```bash
rosbag play --clock -l /root/rosbags/red_4.bag
```

这条 launch 会启动：

- `ft_detector/detect.launch`：`/ft/detection`、`/ft/debug/image`、`/ft/debug/mask`、TF；
- `ft_tracker/track.launch`：`/ft/track`、`/ft/debug/marker`；
- RViz：加载 `rviz/bringup.rviz`，Fixed Frame 为 `camera_optical_frame`；
- `rqt_reconfigure`：在线调检测 HSV / 配对等参数。

## 只做无界面测试

如果当前机器没有显示器：

```bash
roslaunch ft_bringup bringup.launch rviz:=false rqt:=false
rosbag play --clock -l /root/rosbags/red_4.bag
```

然后检查：

```bash
rostopic hz /ft/detection
rostopic hz /ft/track
rostopic hz /ft/debug/image
rostopic echo -n 1 /ft/debug/marker
```

## RViz 应看到

- TF：`camera_optical_frame` 原点附近的机器人/相机坐标轴；
- 远处 `armor_0` / `armor_1`：检测到的装甲板坐标轴；
- `/ft/debug/image`：识别框、ID、大小、坐标轴；
- `/ft/debug/marker`：
  - 绿色球：滤波后的当前位置；
  - 橙色球：提前量后的预测位置；
  - 黄色箭头：当前位置 → 预测位置。

## 动态调参

`rqt_reconfigure` 打开后选择 `image_receiver_node`，可以实时修改：

- 红/蓝 HSV；
- `min_contour_area`；
- `max_pair_distance` / `max_pair_angle`；
- 其他暴露的配对和斜框阈值。

## 验收视频要覆盖

建议录 2~3 分钟，包含：

1. `roslaunch ft_bringup bringup.launch` 一条命令启动；
2. `rosbag play --clock ...` 回放；
3. RViz 的 TF 坐标轴；
4. `/ft/debug/image` 识别框；
5. `/ft/debug/marker` 绿色/橙色球和黄色箭头；
6. rqt_reconfigure 拖动一个 HSV 或面积参数，画面实时变化。

## 已知限制

- `red_1_3` 没有 Bag 自带 TF，Fixed Frame 用 `camera_optical_frame` 即可；
- 跟踪使用单目标匀速模型；`red_4_with_detection` 中同一帧可能出现两块板，单目标关联在近距离换板时会有速度尖峰；
- 先完成基础链路，旋转模型/`/ft/target` 属于进阶。
