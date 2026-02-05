# diablo_dance (ROS 2) — choreography / record & replay for DIABLO

This package publishes `motion_msgs/msg/MotionCtrl` commands to `/diablo/MotionCmd` (the control topic used by `diablo_ctrl_node`) to run a deterministic choreography.

The DIABLO ROS2 SDK describes `/diablo/MotionCmd` and the `MotionCtrl` fields (forward/left/up/roll/pitch/leg_split and mode flags).  
See the upstream diablo_ros2 docs for the interface.

## 1) Build

Put this package **in the same colcon workspace** as `DDTRobot/diablo_ros2` (so `motion_msgs` is available):

```bash
mkdir -p ~/diablo_ws/src
cd ~/diablo_ws/src
git clone -b basic https://github.com/DDTRobot/diablo_ros2.git
# add this package folder diablo_dance/ into src as well
cd ~/diablo_ws
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
```

If rosdep can't resolve PyYAML on your distro, install it with:
```bash
sudo apt-get install -y python3-yaml
```

## 2) Run (example choreography)

Terminal A (robot driver):
```bash
ros2 run diablo_ctrl diablo_ctrl_node
```

Terminal B (dance):
```bash
ros2 launch diablo_dance diablo_dance.launch.py
```

To run the ~120s stage-style demo shipped with this package:
```bash
ros2 launch diablo_dance diablo_dance.launch.py \
  choreo_yaml:=$(ros2 pkg prefix diablo_dance)/share/diablo_dance/config/dance_show_120s.yaml
```

Services:
- `/dance/start` (std_srvs/Trigger)
- `/dance/stop`  (std_srvs/Trigger)
- `/dance/pause` (std_srvs/SetBool)
- `/dance/reload` (std_srvs/Trigger)

`/dance/stop` publishes a **safe hold** once (zeros forward/turn/roll/pitch and keeps the last commanded height),
then stops publishing.

## 3) Record from teleop, then replay (best for matching a video)

1) Start recorder:
```bash
ros2 run diablo_dance diablo_cmd_recorder --ros-args -p out_yaml:=/tmp/my_dance.yaml -p min_dt:=0.02
ros2 service call /record/start std_srvs/srv/Trigger {}
```

2) In another terminal, run whatever teleop you use and drive the robot to match the video.

3) Stop recorder (writes YAML):
```bash
ros2 service call /record/stop std_srvs/srv/Trigger {}
```

4) Replay:
```bash
ros2 run diablo_dance diablo_dance_orchestrator --ros-args -p choreo_yaml:=/tmp/my_dance.yaml -p auto_start:=true
```

## 4) YAML format

Two formats supported:

### A) step-based
- `steps: [{duration, mode_mark, mode, value, pattern}, ...]`
- `pattern.type`: `sine`, `keyframes`, `hold`

### B) timeline-based
- `timeline: [{t, mode_mark, mode, value}, ...]`
- used by recorder

## 5) Included choreographies

- `config/dance_demo.yaml` (short, conservative)
- `config/dance_show_120s.yaml` (~120s, more stage-like, still mostly in-place)
