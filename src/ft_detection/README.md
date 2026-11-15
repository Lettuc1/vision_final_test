# 考核检测消息包

本包只提供 `/ft/detection` 的 ROS1 消息定义，不包含检测算法或模型。对应消息类型为 `ft_detection/DetectionFrame`。

将本目录放入自己的 catkin 工作区 `src/` 下，然后构建并加载环境：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rosmsg show ft_detection/DetectionFrame
```

回放 Bag 后，在已 `source devel/setup.bash` 的终端执行：

```bash
rostopic echo /ft/detection
```

`header` 沿用原图像时间戳和相机坐标系。`detections` 可以为空；每个检测的 `pose.position` 单位为米，`corners` 是原图像像素坐标。`confidence` 在发放的参考检测 Bag 中统一为 `1.0` 占位，不能用于比较检测置信度。参考检测由模型生成，不是人工标注真值。
