# vision_object_tracking

Simple ROS package that performs color-based object tracking using OpenCV and controls a differential-drive robot toward the detected object.

## Build

This package targets ROS (catkin). From your workspace root:

```bash
# install dependencies (if needed)
rosdep install --from-paths src --ignore-src -r -y

# build (catkin)
catkin_make
# or (for ROS2/colcon) adapt accordingly
```

Don't forget to source your workspace:

```bash
source devel/setup.bash
```

## Run in simulation

Start Gazebo with the provided world and robot, plus the vision and controller nodes:

```bash
roslaunch vision_object_tracking simulation.launch
```

You can tune parameters (HSV thresholds, visualization, topics) in `vision_tracking.launch` or via the parameter server. Defaults:

- image_topic: `/camera/image_raw`
- output_topic: `/object_position` (publishes `geometry_msgs/PointStamped`)
- visualize: `true` (shows OpenCV window)
- HSV thresholds: `h_min=0, s_min=120, v_min=70, h_max=10, s_max=255, v_max=255`

Controller params (set in `vision_tracking.launch` or via rosparam):

- `image_width`, `image_height` — used to compute the image center
- `kp_angular`, `kp_linear` — proportional gains for control

## Usage examples & parameter files ✅

You can edit the shipped params at `config/vision_params.yaml` and launch with:

```bash
roslaunch vision_object_tracking vision_tracking.launch
```

To override a single param on the command line:

```bash
roslaunch vision_object_tracking vision_tracking.launch visualize:=false
```

# Dynamic reconfigure (tune at runtime)

The package supports `dynamic_reconfigure`. Start the nodes and run:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Then select `/vision_node` and adjust HSV thresholds, `area_threshold`, and `visualize` in real time.

## CI, badges & contribution

The repository includes a GitHub Actions workflow (`.github/workflows/ros-ci.yml`) that builds the package on pushes and PRs.

[![ROS CI](https://github.com/alouii/vision-object-tracking-ros/actions/workflows/ros-ci.yml/badge.svg)](https://github.com/alouii/vision-object-tracking-ros/actions/workflows/ros-ci.yml)

### Contributing

- Fork the repository and open a PR with a clear description of the change.
- Run `rosdep install --from-paths src --ignore-src -r -y` and `catkin_make` locally before submitting.
- Prefer small, easily reviewable changes and include tests / demos where practical.

## Notes & Suggestions

- Tweak HSV thresholds to match your target color under different lighting.
- If using headless systems, set `visualize=false` to avoid GUI issues.
- Consider adding diagnostic topics (e.g., mask, debug images) and a parameter to publish mask images for offline debugging.

If you'd like, I can add runtime checks, unit tests, or a launch file that exposes more tuning knobs.
