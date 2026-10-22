# 2026 intelligent low-altitude firefighting mission

Current runtime uses only these two launch files:

```bash
roslaunch location location.launch
roslaunch navigation_2d navigation_2d.launch
```

Gazebo Classic and PX4 SITL are kept in the separate `gc_2026_sim` package;
that package is not installed on the flight computer.

`location.launch` starts Pixhawk 6C/PX4, RPLidar S2, Cartographer, TFmini, and `/mavros/vision_pose/pose`. `navigation_2d.launch` starts the selected planner, the mission state machine, the USB camera, servos, and the optional Foxglove Bridge. Gazebo resources and PX4 SITL nodes live only in `gc_2026_sim`.

The aircraft keeps its takeoff heading for the whole mission; planners only command horizontal translation. Servo 0 starts front-facing, turns down after takeoff, and returns front-facing at the landing point. The landing mode request waits for the front-facing servo acknowledgement and settle interval.

The real servo node is C++ and writes PCA9685 over Linux I2C; it does not load
Python ServoKit or Jetson GPIO. Configure `/dev/i2c-1`, board address, channel
mapping, pulse range, and calibrated front/down angles in
`navigation_2d.launch`. Camera-move acknowledgements include a settling delay;
failed down-facing confirmation aborts the route, while failed front-facing
confirmation prevents an automatic landing request.

Legacy letter-recognition and laser-targeting code are not part of the current CMake target or launch chain.

## Mission flow

```text
WAIT_FCU -> TAKEOFF -> NAV_HAZARD -> HAZARD_HOVER
-> HAZARD_ALIGN -> HAZARD_DROP -> NAV_PERSON
-> [stage 2] NAV_RESCUE -> RESCUE_HOVER
   -> RESCUE_ALIGN -> RESCUE_DROP
-> RETURN_HOME -> LANDING -> COMPLETE
```

Round 1 is the same flight with the `NAV_RESCUE` leg omitted, so `NAV_PERSON` goes
straight to `RETURN_HOME`. The round is chosen with `stage:=1` or `stage:=2`, and
the two are reconciled with the older `rescue_drop_enabled` flag inside the node,
not in the launch file.

`person_route` is the tour of the 人员救助任务区 and must contain a waypoint over
**every** person card: the requirement is to pass over all of them before
returning home, and `tools/check_config.py` fails if a `person_zone_card_*` model
in the world has no waypoint over it. The rescue-material drop is a separate leg
to a position drawn by lot on the day, so it is not part of the tour and cannot be
baked into the route table.

The hazard target uses yellow candidates and a red box. The person target uses
blue candidates and a blue box. Candidates are ordered from top to bottom and then
left to right. `hazard_target_index` and `person_target_index` select the drawn
position, and `hazard_target_index` must name the same card the last `hazard_route`
waypoint sits on - that waypoint is also the reference the waypoint-drop fallback
drops on, and the gate checks the pair against the world file. The selected
candidate must be stable for several frames; the node times out instead of drawing
a box or dropping on an empty image.

## How the drop point is decided

`drop_align_mode` picks between three strategies, and it applies to both drops:

| mode | behaviour | when to use |
| --- | --- | --- |
| `vision` | camera only; no stable detection aborts the run | the drawing is not trusted, the camera is |
| `waypoint` | authored waypoint only; the camera is not used | the detector misbehaves at the venue |
| `vision_with_fallback` | camera first, then the authored waypoint | **default, recommended** |

The recommended mode is `vision_with_fallback`, and the reasoning is short: the
camera is the only measurement here of where the *target* is, everything else
measures the *vehicle*. It aligns on the card when the card is seen, and when it is
not it drops on the authored waypoint - but only after checking the vehicle is
already within `drop_waypoint_tolerance` of that waypoint. So a missed blob can
cost accuracy, or cost a hold, but it can never move the drop to a place nobody
chose. The old camera-only behaviour turned one missed blob into the end of the
run (`Hazard target detection timeout` aborts), which is the worst trade available
when a waypoint drop would still score.

The authored waypoint is the last `hazard_route` point for the hazard drop and
`rescue_drop_x` / `rescue_drop_y` for the rescue drop. Both are logged at the start
of the phase, and a fallback drop is logged as a warning that names how far the
vehicle was from the waypoint, so an aligned drop and a fallback drop never look
the same in a log.

## Launch modes

Final first stage (round 1), one drop, no rescue material:

```bash
roslaunch navigation_2d navigation_2d.launch auto_start:=true stage:=1 start_foxglove_bridge:=true
```

Final second stage (round 2), two drops. The drawn position is a start-up
parameter in odom/field metres - takeoff is `(0, 0)`, the field is
`x -0.65..3.35`, `y -3.35..0.65`, and mat millimetres convert as
`x = mat_x/1000 - 0.65`, `y = mat_y/1000 - 3.35`:

```bash
# the lot drew 人员救助任务区 2 (mat x 3350, mat y 1450)
roslaunch navigation_2d navigation_2d.launch auto_start:=true stage:=2 \
  rescue_drop_x:=2.70 rescue_drop_y:=-1.85 start_foxglove_bridge:=true
```

Both parameters default to `999.0`, which is outside the field and is what "not
supplied" looks like. A stage-2 run that does not set them logs an error and never
leaves `WAIT_FCU`, rather than dropping the rescue material at the take-off pad.

Select a local planner with `planner_backend:=dwa`, `planner_backend:=teb`, or
`planner_backend:=ego`. EGO runs without `move_base` and uses the same mission
waypoint goals, laser scan, and map-to-base TF; its trajectory is converted to
bounded `/cmd_vel` by the included tracker.

Run `location.launch` and `navigation_2d.launch` under the same ROS Master. Before
starting the mission, verify `/mavros/state`, `/mavros/local_position/pose`,
`/scan`, and the `map -> base_link` TF.

## Compressed video

The new vision node publishes only:

```text
/mission/video/compressed       sensor_msgs/CompressedImage
/mission/vision/detection       navigation_2d/TargetDetection
```

Raw camera frames stay in the onboard process for detection and annotation. The mission frame is resized and JPEG encoded before publishing, so the old `/detect_result` raw image is not sent. Defaults are 640 px wide, 15 fps, JPEG quality 55. For weak wireless links, start with `video_width=480`, `camera_fps=5`, and `jpeg_quality=40-55`.

With `start_foxglove_bridge:=true`, Foxglove Bridge listens on port `8765`; select `/mission/video/compressed` in the Image panel. RViz remains useful for `/map`, `/scan`, TF, and move_base planning. Do not subscribe to an old raw image topic during the run.

## Gazebo simulation reference

The simulation implementation lives in `src/gc_2026_sim`; this section is
kept as a quick reference for the shared mission workflow.

The repository includes a Gazebo Classic field scene based on the rule dimensions:

- field: `4.0 m x 4.0 m`
- fixed obstacles: `2.0 m` high
- random obstacles: `0.20 m` diameter and `2.0 m` high
- target cards: `0.40 m x 0.40 m`
- hazard cards: yellow cards with red borders
- person cards: blue cards

Start the complete WSL simulation with:

```bash
sudo apt install ros-melodic-gazebo-ros-pkgs ros-melodic-cv-bridge
cd ~/catkin_ws
catkin build gc_2026_sim navigation_2d ego_planner_2d
source devel/setup.bash
roslaunch gc_2026_sim gazebo_field.launch
```

The launch starts Gazebo, the field, a visual quadrotor, a simulated 2D laser, move_base, simulated MAVROS state/pose services, the mission state machine, and the Gazebo down-facing camera. It does not open `/dev/video0`, Pixhawk, RPLidar, TFmini, or Jetson GPIO. Use `gui:=false` for a headless run, and `stage:=2 rescue_drop_x:=... rescue_drop_y:=...` for the second final stage.

The world origin is the takeoff point. The `x` axis points to the right side of the drawing and the `y` axis points toward the top of the drawing. With the takeoff point at 650 mm from the left and top boundaries, the field bounds are `x=-0.65..3.35 m` and `y=-3.35..0.65 m`. The fixed obstacle and target positions follow the supplied field drawing; random obstacle positions can be changed directly in `gazebo/worlds/gc_2026_field.world`. The simulation vehicle is a control-flow, route-planning, and vision-validation model, not a replacement for PX4 flight dynamics. Real PX4 SITL can be connected later through the same MAVROS topics and services.

## PX4 SITL backend

For PX4 commit `1c8ab2a0d7`, close previous PX4/Gazebo runs with Ctrl+C, then
start the ROS1 simulation through one entry point:

```bash
cd ~/catkin_ws
bash src/gc_2026_sim/scripts/start_px4_slam.sh
```

The script extends `~/cartographer_ws/install_isolated/setup.bash`,
`~/foxglove_bridge_ws/devel/setup.bash` and this workspace's `devel/setup.bash`.
Override `CARTOGRAPHER_SETUP`, `FOXGLOVE_SETUP`, or `PX4_ROOT` when installed
elsewhere. It checks dependencies and refuses to start over an existing
PX4/Gazebo process. Build `gc_2026_sim navigation_2d` after source changes.

The launch loads the Gazebo ROS API and `/clock`, spawns Iris at `(0, 0, 0.25)`
above the takeoff pad, and starts MAVROS, sensors, Cartographer and Foxglove.
It does not arm or start the mission by default. Use `gui:=false` for headless
operation. Do not additionally run `make ... gazebo` or another MAVROS launch.

To start the autonomous mission in SITL:

```bash
bash src/gc_2026_sim/scripts/start_px4_slam.sh auto_start:=true
```

The local PX4 startup wrapper enables bit 2 of `COM_RCL_EXCEPT` (OFFBOARD
only), because this simulation has no RC/joystick input. Other exception
bits are preserved; OFFBOARD setpoint-loss protection is not disabled.
Without this setting, PX4 1.13 can repeatedly leave OFFBOARD before the
mission gets an opportunity to arm. QGC connectivity is not RC input.
This wrapper is used only by `px4_gazebo_slam.launch`, not the real-aircraft
launch. Do not copy this setting blindly to the aircraft.

`sitl_no_rc:=false` skips this configuration but does not reset a previously
saved SITL parameter. To restore RC-loss checking for OFFBOARD, clear bit 2
of `COM_RCL_EXCEPT` in the SITL parameter editor (set to `0` if no other
exceptions are needed). Never use Gazebo Reset Time while PX4 is running.

The old make target spawns Iris at `(1.01, 0.98, 0.83)`, outside this field
(`y <= 0.65`), so it falls indefinitely and the follow camera loses the map.
Changing world physics alone does not fix that. Reset Time makes PX4's
sensor clock jump backwards; restart the entire launch instead.

Cartographer consumes `/scan` and `/mavros/imu/data`, without odometry, GPS,
or Gazebo truth input. It publishes `map -> odom -> base_link`, with a static
`base_link -> laser_link` mounting transform. The occupancy grid node publishes
the measured `/map`; this is not a pre-rendered rule map. The separate
`/simulation/ground_truth/odom` is diagnostic only and publishes no TF.

In Foxglove, connect via Foxglove WebSocket to `ws://localhost:8765`.
In the 3D panel choose display frame `map`, and enable `/map`, `/scan` and TF.
Use `/mission/video/compressed` in the Image panel. WSL IP can be used instead
of localhost when Windows localhost forwarding is unavailable.

Observe a running, disarmed simulation for 45 seconds:

```bash
source ~/catkin_ws/devel/setup.bash
python3 ~/catkin_ws/src/gc_2026_sim/scripts/check_px4_slam.py
```

The PX4 launch uses `udp://:14540@127.0.0.1:14580`, matching this checkout's
Onboard MAVLink port. This project has one ordinary RGB camera whose viewing
direction is changed by servo 0; it does not use a depth camera. The auxiliary
collision-free sensor rig follows Iris and provides `/camera/image_raw` and
horizontal `/scan`. The real-aircraft launch does not use the sensor rig.

```bash
rostopic hz /scan
rostopic hz /map
rosrun tf tf_echo map laser_link
```

The PX4 SITL launch does not convert depth images into laser scans. The auxiliary rig uses a ray sensor to emulate the horizontal RPLidar S2 scan. Set `start_sensor_rig:=false` only when another model or a simulated RPLidar already publishes both topics. The rig's camera is fixed down-facing for ground-card validation; servo 0 front/down motion remains part of the real-aircraft hardware path.

The filename is `gc_2026_field.world`, with internal world name `default`.
The optional smoke test checks only SDF parsing and Iris spawning while paused;
it does not prove sustained PX4 operation, rendering, or SLAM:

```bash
python3 src/gc_2026_sim/scripts/check_px4_field.py
```

## Parameters requiring calibration

The routes in `navigation_2d.launch` are examples and must be changed to the actual field coordinates and the drawn random-obstacle layout:

- `hazard_route`: route from takeoff to the hazard task area; its last point is
  both the hazard drop goal and the fallback drop reference, and must sit on
  `hazard_zone_card_1` when `hazard_target_index:=0`.
- `person_route`: the tour of the 人员救助任务区, one waypoint over **each** card.
- `return_route`: route from the person task area back to takeoff.

The mission-side parameters of the two rounds:

- `stage`: `1` (hazard drop only) or `2` (hazard drop + rescue-material drop).
- `rescue_drop_x`, `rescue_drop_y`: the drawn round-2 drop position, in odom
  metres. Required for stage 2; a stage-2 run without a position inside the field
  will not take off.
- `drop_align_mode`: `vision`, `waypoint` or `vision_with_fallback` (default).
- `drop_vision_timeout`: seconds the camera gets before the fallback fires
  (default 5.0, must stay below `task_timeout`).
- `drop_waypoint_tolerance`: how close to the authored waypoint the vehicle must
  already be for a fallback drop to be allowed (default 0.20 m).

Servo defaults are placeholders and must be calibrated on the aircraft:

- Servo 0: `front_angle`, `down_angle`.
- Servo 1: `drop_safe_angle`, then one angle per material - `fire_drop_angle` for
  the hazard drop and `rescue_drop_angle` for the rescue drop. Both are launch
  arguments, and the angle is what selects which container releases, so they must
  differ enough that the mechanism cannot confuse them (the node warns below 5
  degrees apart, and from a parked angle within 5 degrees of either).
- Drop hold time: `drop_action_seconds`.

Camera alignment parameters also need calibration: `camera_fx`, `camera_fy`, `pixel_to_body_x_sign`, `pixel_to_body_y_sign`, and `max_task_correction`.

## Safety behavior

- While waiting for FCU, switching to OFFBOARD, and arming, the node holds the current pose and never treats default height 0 as a descend target.
- An unexpected OFFBOARD loss enters `ABORT`; PX4's configured failsafe remains in control.
- Navigation timeouts resend a goal a limited number of times. An abort holds the current position; `AUTO.LAND` is only requested when already within `abort_land_home_radius` of the captured home position.
- A stage-2 run without a usable drawn drop position never leaves `WAIT_FCU`; it logs the two parameters to set instead of flying. Dropping the rescue material in the wrong place cannot be undone, so the missing-parameter case is a refusal, not a default.
- A `vision_with_fallback` drop only falls back when the vehicle is already within `drop_waypoint_tolerance` of the authored waypoint. If it is not, the node holds and reports rather than dropping blind, and `task_timeout` still ends the phase.
- Mission completion requires PX4 to report `armed=false`, not just a successful landing-service call.
- Before real flight, use `auto_start:=false` and validate OFFBOARD and arming with the existing ground-station procedure.
