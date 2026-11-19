# 视觉组最终考核

本仓库当前已完成**传统视觉装甲板检测节点**（`src/ft_detector`）和**单目标匀速平移跟踪节点**（`src/ft_tracker`，含提前量和 RViz marker）。

```bash
cd ~/vision_final_test
catkin_make
source devel/setup.bash

# 一条命令启动检测 + 跟踪 + RViz + rqt_reconfigure
roslaunch ft_bringup bringup.launch

# 另一终端回放 Bag
rosbag play --clock -l /root/rosbags/red_4.bag
```

无显示器测试：

```bash
roslaunch ft_bringup bringup.launch rviz:=false rqt:=false
rosbag play --clock -l /root/rosbags/red_4.bag
```

需要图形界面时：

```bash
roslaunch ft_detector image_test.launch
```

该 launch 会同时启动检测节点、`rqt_image_view`（查看 `/ft/debug/image`）和
`rqt_reconfigure`（在线调 HSV、最小面积、配对阈值等）。

单独调试跟踪节点时：

```bash
roslaunch ft_tracker track.launch
rosbag play --clock -l /root/rosbags/red_4_with_detection.bag
```

`/ft/track` 包含 `header`、`id`、`tracking`、`position`、`velocity`、
`position_predict`、`lead_time`；`/ft/debug/marker` 发布当前位置和预测位置。

RViz 里 `Fixed Frame` 用 `camera_optical_frame`，TF 显示勾选 `Show Axes`；Marker 话题是 `/ft/debug/marker`。`red_4` 的 `camera_info` 里是 `hk_camera_ca`，代码已通过 `camera_frame_override` 统一到 `camera_optical_frame`。

详细说明见 [src/ft_bringup/README.md](src/ft_bringup/README.md)、
[src/ft_detector/README.md](src/ft_detector/README.md) 和
[src/ft_tracker/README.md](src/ft_tracker/README.md)，包括：

- 话题、消息和 TF；
- 装甲板四点定义与坐标轴方向；
- 过曝空心、地胶反光和相邻装甲板跨板配对的修复思路；
- YAML 参数、动态调参和调参顺序；
- 已知限制。

跟踪部分已完成单目标平移模型 + 提前量（`src/ft_tracker`）；旋转模型和 `/ft/target`
属于后续内容，尚未完成。

## 跟踪效果检查

```bash
roslaunch ft_tracker track.launch
python3 tools/check_track.py --duration 25
rosbag play --clock -r 1 /root/rosbags/red_4_with_detection.bag
```

统计脚本会打印 `/ft/track` 的 tracking 比例、ID 切换、最长丢检时间、速度和 NaN 检查。
详细判据见 [src/ft_tracker/README.md](src/ft_tracker/README.md)。
