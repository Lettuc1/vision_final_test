# ft_detector

Traditional RoboMaster armour detector used by the final test.

## Build / run

```bash
cd ~/vision_final_test
catkin_make
source devel/setup.bash

# terminal 1: detection node (loads config/armor_params.yaml)
roslaunch ft_detector detect.launch

# terminal 2: play a bag with simulated clock
rosbag play --clock -l /root/rosbags/red_4.bag

# optional GUI tools
roslaunch ft_detector image_test.launch
```

`ft_detection` is the message package supplied with the assignment; it is
already copied under `src/ft_detection` and compiled by `catkin_make`.

## Topics

| Direction | Topic | Type |
|---|---|---|
| subscribe | `/hk_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` (through `image_transport`) |
| subscribe | `/hk_camera/camera_info` | `sensor_msgs/CameraInfo` |
| publish | `/ft/detection` | `ft_detection/DetectionFrame` |
| publish | `/ft/debug/image` | `sensor_msgs/Image` (BGR8) |
| publish | `/ft/debug/mask` | `sensor_msgs/Image` (MONO8, optional) |
| publish | `/tf` | `camera frame -> armor_<id>` |

`header.stamp` is always the input image stamp.  `header.frame_id` comes from
the latest `camera_info` (or the image header / YAML fallback if the bag has no
camera_info at all).

## Detection definition (be consistent when explaining the code)

This implementation uses **definition 1** from the assignment: the four
points are the two endpoints of each light bar.

```
       TL -------- TR       object frame:
        |            |        x = right along the plate
        |   number   |        y = down along the light bar
        |            |        z = into the plate (away from camera)
       BL -------- BR       origin = centre between the two bars
```

* `armor_small_width = 120 mm`, `armor_large_width = 235 mm`: distance between
  the two light-bar centres (set in YAML; measure your own plates if needed).
* `lightbar_length = 55 mm`: visible light-bar length.
* Object points are converted to metres before `cv::solvePnP`.
* Image points are always ordered TL, TR, BR, BL (left-bar top/bottom,
  right-bar top/bottom).  Bars are sorted left-to-right first, so the order is
  stable across frames.  `SOLVEPNP_IPPE` is tried first, then
  `SOLVEPNP_ITERATIVE`; the lower reprojection error wins.

## Why the previous version failed

1. **Hollow/over-exposed light bars**: a strong LED clips to white in the
   centre.  A strict HSV mask produces two coloured strips or a ring.  The old
   code sampled the colour at the rectangle centre (black) and discarded the
   bar.  This version adds white core pixels only next to coloured pixels and
   fills the external contour, so a hollow bar becomes solid.
2. **Floor/wax reflections**: they have a similar hue but are diffuse.  Their
   90th-percentile V is much lower and they contain no white-clipped core.
   Bars are accepted only if `p90(V) >= min_bar_p90_v` **and** either the
   white-core ratio (red) or saturated-core ratio (blue) is high.  Pairing also
   checks length/width/angle/top-bottom symmetry and the dark plate interior.
3. **Big/small**: decided from `gap / mean_bar_length` versus the physical
   `W / L` ratio, not a fixed pixel distance.  This works at different ranges.

## Parameters

All values are in `config/armor_params.yaml`, loaded by
`launch/detect.launch` via `$(find ft_detector)`.  The required HSV, area and
pairing values are also exposed through `dynamic_reconfigure`:

```bash
rosrun dynamic_reconfigure dynparam get /image_receiver_node
rosrun dynamic_reconfigure dynparam set /image_receiver_node min_contour_area 200
```

The node pushes the YAML values into the dynamic server at start-up, so YAML
and slider values stay consistent.

Tuning order:

1. Look at `/ft/debug/mask`; lower `red_v_min` / `blue_v_min` only until the
   bars are solid.  If walls appear, raise `min_bar_p90_v`.
2. If reflections survive, raise `min_bar_p90_v` or `min_white_ratio` /
   `min_sat_ratio`; do not lower them to "catch" reflections.
3. If real bars are lost, check `min_contour_area`, `min_bar_length`,
   `max_bar_tilt`, and the pairing limits.  Half/occluded bars should be lost,
   not forced into a pair.
4. Big/small: adjust `armor_small_width`, `armor_large_width`,
   `lightbar_length`; keep `armor_width_tolerance` around 0.6.

## Debug image

The debug image shows:

* yellow dots: all accepted light-bar candidates;
* green polygon: the four light-bar endpoints of an accepted armour;
* red/green/blue axes: `projectPoints` of the armour x/y/z axes;
* label: `#id RED/BLUE LARGE/SMALL distance n=<number score>`.

There is no `cv::imshow` or `cv::waitKey` in the submitted code.

## Known limitations

* Traditional HSV detection cannot read the armour number; the big number is
  used only as an extra score, not as an ID.
* The far/small robot in `blue_guard.bag` is only a few pixels wide; some
  frames are lost (preferable to a false pair).
* A side bar can occasionally pair with a front bar when the robot is very
  close and both plates are coplanar in the image; the number/geometry score
  keeps this rare.  A tracker with temporal gating is the right place to
  remove the remaining flicker.
