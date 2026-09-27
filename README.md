# tb_auto: TurtleBot Sign & Traffic-Light Autonomous Driving

## Overview

This is a ROS 1 Noetic package that lets a TurtleBot drive itself using a
YOLOv8 detector trained on four classes: `left_turn`, `right_turn`,
`red_light`, `green_light`. Inference runs remotely on a laptop (the
TurtleBot's RPi4 can't do real-time YOLO), while the actual motor control is
a dependency-free state machine that runs on the robot itself, so a dropped
wireless link still brings the robot to a safe stop. Planning notes and
decisions made along the way are in `PLAN.md`; this file covers structure
and day-to-day usage.

## Key Capabilities

- **Split-node architecture**: `collector_node`/`detector_node` (laptop, need
  torch/ultralytics) and `controller_node` (robot, pure Python) talk over
  `/sign_light/detections`, so the robot never needs GPU or a full ML stack.
- **Distance without a range sensor**: bbox-height / frame-height ratio
  stands in for proximity, with per-class thresholds tuned from the actual
  camera feed.
- **Debounced detection**: an action only fires after N consecutive frames
  of the same class, so a single misdetection can't trigger a turn or stop.
- **Angle-accurate turns**: `left_turn`/`right_turn` rotate for exactly
  `turn_angle / turn_z` seconds (default 90 degrees), then hand control back
  to the traffic-light state on the next frame.
- **Self-stopping watchdog**: if no detection message arrives for
  `watchdog` seconds (wireless drop, laptop crash, inference stall), the
  robot sends zero velocity itself — it never depends on the remote side to
  say "stop".
- **One-command dataset pipeline**: collection, labeling and train/val
  split are separate, replayable steps (`collect.launch` -> LabelImg ->
  `split.py` -> `yolo detect train`).

## Repository Layout

```
catkin_ws/src/sign_light_driving/
  scripts/collector_node.py    data collection (laptop)
  scripts/detector_node.py     YOLO inference, publishes detections only (laptop)
  scripts/controller_node.py   detections -> cmd_vel state machine (robot)
  launch/collect.launch
  launch/detect.launch
  launch/control.launch
split.py  autolabel.py         dataset tooling (not ROS nodes)
dataset_raw/  dataset/  runs/  data and training output
```

Build the workspace on both the laptop and the robot (once):
```bash
cd ~/tb_auto/catkin_ws && catkin_make
echo 'source ~/tb_auto/catkin_ws/devel/setup.bash' >> ~/.bashrc
```

## Pipeline

**1. Bring up the robot** (`ssh ubuntu@100.88.31.34`, two terminals).
`ROS_IP` must be exported on the robot too:
```bash
export ROS_IP=100.88.31.34
roslaunch aicon_bringup aicon_robot.launch
```
```bash
export ROS_IP=100.88.31.34
roslaunch aicon_bringup aicon_camera.launch      # /camera/image, 320x240@30
```

**2. Set up the laptop terminal** (repeat per terminal):
```bash
source /opt/ros/noetic/setup.bash
source ~/tb_auto/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://100.88.31.34:11311
export ROS_IP=100.76.204.28                      # laptop's tailscale IP
```
One-time install: `pip install ultralytics labelImg` (pulls in CPU torch).

**3. Collect** (laptop, images land locally, no scp needed):
```bash
roslaunch sign_light_driving collect.launch
```
Keys: `1`-`4` select class, `space` save, `q` quit -> `dataset_raw/{class}/*.jpg`.
Target: 200 images per class, mixed angle/distance/lighting.

**4. Label**:
```bash
labelImg dataset_raw dataset_raw/classes.txt dataset_raw/labels
```
Set the format to **YOLO** (bottom-left button, default is PascalVOC), turn on
`View > Auto Save mode`, and check **`Use default label`** so each box is
tagged automatically as you page through with `d`/`a` (arrow keys only nudge
the box, they don't advance the image). Passing `classes.txt` explicitly is
required — without it LabelImg numbers classes by first-seen order per
session, which silently corrupts the dataset across sessions.

**5. Split**:
```bash
python3 split.py        # dataset/{images,labels}/{train,val} + data.yaml, 8:2
```

**6. Train** (run in a persistent terminal — this takes hours):
```bash
yolo detect train model=yolov8n.pt data=dataset/data.yaml \
  epochs=100 imgsz=320 batch=16 patience=20 device=cpu workers=8
```
`imgsz=320` matches the camera's native resolution; 640 would just be an
upscale with no extra information. Output: `runs/detect/train/weights/best.pt`.

**7. Drive**:
```bash
roslaunch sign_light_driving detect.launch model:=<path to best.pt>   # laptop
roslaunch sign_light_driving control.launch                           # robot
```
Behavior: nothing detected -> drive straight / `red_light` -> stop /
`green_light` -> resume / `left_turn`, `right_turn` -> rotate in place by
`turn_angle`, then resume straight (new detections ignored while turning,
plus a cooldown afterward so the same sign can't retrigger the turn).

Verify the control logic alone, no ROS needed:
```bash
rosrun sign_light_driving controller_node.py --selftest
```

## Setup Requirements

**`ROS_IP` must be set on the robot, not just the laptop.** Without it the
robot's nodes advertise themselves as `http://ubuntu:<port>/`, which the
laptop can't resolve — topics show up in `rostopic list` but zero messages
ever arrive. Setting `ROS_IP=100.88.31.34` on the robot makes it advertise
by IP instead.

If camera bringup fails with `cv camera open failed: device_id 0`, a
previous `cv_camera_node` still holds `/dev/video0`; `pkill -f
cv_camera_node` and relaunch after a few seconds.

Tuning parameters (all `control.launch` args, override with
`roslaunch ... name:=value`):

| Parameter | Default | Meaning |
|---|---|---|
| `conf` | 0.6 | detections below this confidence are dropped |
| `light_ratio` | 0.15 | traffic-light bbox-height/240 must reach this to react |
| `turn_ratio` | 0.30 | sign must be this close (bbox ratio) to trigger a turn |
| `debounce` | 3 | N consecutive same-class frames required to act |
| `speed` | 0.1 | default straight-driving speed, m/s |
| `turn_z` | 0.5 | rotation speed, rad/s |
| `turn_angle` | pi/2 | rotation target angle, rad (turn duration = turn_angle/turn_z) |
| `watchdog` | 0.5 | seconds without a detection message before forcing a stop |

**Never touch the watchdog logic.** Inference is remote over wireless; if the
link drops, the robot must stop on its own using its last-known state, not
wait for a message that may never come.
