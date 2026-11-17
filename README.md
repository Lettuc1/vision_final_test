# 视觉组最终考核

本仓库当前已完成**传统视觉装甲板检测节点**，代码位于 `src/ft_detector`：

```bash
cd ~/vision_final_test
catkin_make
source devel/setup.bash

# 终端 1：启动检测节点
roslaunch ft_detector detect.launch

# 终端 2：回放 Bag
rosbag play --clock -l /root/rosbags/red_4.bag
```

需要图形界面时：

```bash
roslaunch ft_detector image_test.launch
```

该 launch 会同时启动检测节点、`rqt_image_view`（查看 `/ft/debug/image`）和
`rqt_reconfigure`（在线调 HSV、最小面积、配对阈值等）。

详细说明见 [src/ft_detector/README.md](src/ft_detector/README.md)，包括：

- 话题、消息和 TF；
- 装甲板四点定义与坐标轴方向；
- 过曝空心、地胶反光和相邻装甲板跨板配对的修复思路；
- YAML 参数、动态调参和调参顺序；
- 已知限制。

跟踪、提前量、旋转模型和 `/ft/target` 属于后续 tracker / selector 节点，本仓库
目前只完成了检测链路。
