# Vision Final Test


## Implemented detector

The traditional armour detector is in `src/ft_detector`:

```bash
catkin_make
source devel/setup.bash
roslaunch ft_detector detect.launch
rosbag play --clock -l /root/rosbags/red_4.bag
```

See `src/ft_detector/README.md` for topics, the four-point/coordinate
convention, YAML parameters and the over-exposure/reflection handling.
