# MATLAB Integration Guide

This folder contains MATLAB nodes for:

- dynamic wind generation and ROS2 publishing
- landing decision inference using wind + drone state + AprilTag stability
- startup takeoff sequencing and XY PID hold using AprilTag predicted center

## Files

- `landing_decision_matlab.m`
  - main decision node
  - publishes `/landing_decision` (`std_msgs/String`: `land`, `caution`, `wait`)
- `startWindPublisher.m`
  - public entrypoint to start dynamic wind publishing loop
- `wind_publisher_matlab.m`
  - source implementation for wind synthesis and publish logic

## ROS Interfaces

### Inputs

- `/wind_condition` (`std_msgs/Float32MultiArray`)
- `/drone/gt_pose` (`geometry_msgs/Pose`)
- `/drone/bottom/tags` (`apriltag_msgs/msg/AprilTagDetectionArray`) if MATLAB custom type is available
- `/landing_tag_state` (`std_msgs/Float32MultiArray`) fallback bridge topic

### Outputs

- `/drone/takeoff` (`std_msgs/Empty`) startup takeoff trigger
- `/drone/cmd_vel` (`geometry_msgs/Twist`) XY hold commands
- `/landing_decision` (`std_msgs/String`)
- `/wind_command` (`std_msgs/Float32MultiArray`) when MATLAB wind publisher is enabled

## Wind Generation Algorithm

Implemented in `startWindPublisher` as a fixed-rate timer loop.

### Model composition

Total horizontal wind vector:

- `W = W_steady + W_dryden + W_gust`

with direction-dependent decomposition into x/y components.

### Components

1. Steady wind
- magnitude: `steady_speed`
- direction: `steady_dir` (deg)

2. Wind shear (power law)
- `U(z) = U_ref * (z / z_ref)^alpha`
- altitude `z` from pose topic when available

3. Dryden-like turbulence (time-domain approximation)
- first-order exponential filters on white noise for longitudinal/lateral components
- parameters:
  - `dryden.Lu`, `dryden.Lv`
  - `dryden.sigma_u`, `dryden.sigma_v`

4. Gust events
- Poisson-triggered burst events with exponential decay
- parameters:
  - `gust.prob`
  - `gust.amp`

### Publish paths

- service path (optional): `/set_wind`
- topic path (continuous): `/wind_command`

Recommended robust mode for this workspace:

- `use_set_wind_service = false`
- continuous topic publish to `/wind_command`

## Landing Decision Algorithm

Implemented in `landing_decision_matlab.m`.

### Stage A: drone/wind safety gates

- wind speed thresholds:
  - `wind_speed_caution`
  - `wind_speed_unsafe`
- attitude limit from quaternion -> roll/pitch
- vertical velocity guard (`max_vz_land`)

### Stage B: landing-zone observability (AprilTag)

Tag feature extraction:

- target id selection (`tag_target_id`, default 0)
- center in pixels (`centre` or `center`, fallback from corners)
- polygon area from tag corners
- detection quality from `decision_margin`

Stability features:

- center jitter: RMS of frame-to-frame center displacement (px)
- area jitter ratio: `std(area)/mean(area)`
- margin mean
- center offset in normalized image coordinates

Composite stability score in `[0,1]` combines:

- center jitter term
- area jitter term
- margin quality term

### Stage C: decision output

Rule-based output:

- `wait`: unsafe wind/attitude or missing/poor/unreliable tag state
- `caution`: intermediate risk (off-center, jitter high, small tag area, low score)
- `land`: all safety + stability checks satisfied

### Stage D: startup + XY hold control

- phase flow: `wait_ready -> takeoff -> hover_settle -> xy_hold`
- startup prefers `/drone/state` and falls back to altitude-based inference when state is stale
- XY hold uses predicted tag center (`u_pred`, `v_pred`) and falls back to current center when needed
- control output publishes to `/drone/cmd_vel` with zero z/yaw command

Current startup gate for experiments:

- phase flow is now `wait_ready -> pre_takeoff_stabilize -> takeoff -> hover_settle -> xy_hold`
- in `pre_takeoff_stabilize`, node continuously publishes zero wind command (`/wind_command = [0, 0]`)
- takeoff is allowed only after zero-wind settle time and continuous tag-center hold are satisfied

### Stage E: tag loss continuity

- on valid detection, the node caches the latest tag state (`id`, center, area, margin)
- if detection is briefly lost, cached state is reused for `tag_hold_timeout_sec`
- this prevents abrupt tag-position discontinuities during temporary detector dropouts

## Fallback for MATLAB custom message limitations

If MATLAB cannot create subscriber for `apriltag_msgs`, the script automatically falls back to `/landing_tag_state` (`std_msgs/Float32MultiArray`) and continues full stability inference.

Bridge vector format:

- `[detected, tag_id, center_x_px, center_y_px, area_px2, margin, num_tags]`

## Run Procedure

1. Build and source workspace:

```bash
cd /home/j/INCSL/IICC26_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source /home/j/INCSL/IICC26_ws/install/setup.bash
```

2. Launch simulation + AprilTag + bridge:

```bash
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py \
  use_gui:=false \
  use_rviz:=false \
  controller:=joystick \
  takeoff_hover_height:=2.0 \
  takeoff_vertical_speed:=0.8 \
  use_apriltag:=true \
  apriltag_camera:=/drone/bottom \
  apriltag_image:=image_raw \
  apriltag_tags:=tags \
  apriltag_type:=umich \
  apriltag_bridge_topic:=/landing_tag_state
```

3. Run MATLAB:

```matlab
run('/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/landing_decision_matlab.m')
```

## Practical Tuning Tips

- If decision is too conservative:
  - increase `tag_center_tolerance`
  - increase `tag_jitter_warn_px`
  - decrease `tag_stability_score_warn`
- If false landing approvals occur:
  - increase `tag_margin_warn`
  - increase `tag_min_area_px2`
  - decrease `tag_area_jitter_warn_ratio`

## Key Runtime Parameters (Current Defaults)

- `params.tag_predict_timeout_sec = 0.6`
- `params.tag_hold_last_state = true`
- `params.tag_hold_timeout_sec = 0.6`
- `params.pre_takeoff_zero_wind_enabled = true`
- `params.pre_takeoff_zero_wind_settle_sec = 2.0`
- `params.pre_takeoff_require_tag_centered = false`
- `params.pre_takeoff_tag_center_tolerance = 0.03`
- `params.pre_takeoff_tag_center_hold_sec = 1.0`
- `params.xy_pid_kp = 1.2`
- `params.xy_control_center_deadband = 0.05`
- `params.flying_altitude_threshold = 0.20`
- `params.state_stale_timeout_sec = 1.0`
- `cfg.wind_start_delay_after_hover_sec = 5.0`
- `params.wind_start_require_tag_centered = true`
- `params.wind_start_tag_center_hold_sec = 1.0`

## PID Direction and Visualization

`landing_decision_matlab.m` live plot includes a dedicated panel:

- Tag-Drone alignment (`u,v` normalized frame)
- current tag point and predicted tag point
- alignment vector (predicted tag -> center)
- command vector (PID intent) from current `cmd_x`,`cmd_y`
- phase/state text (`control_phase`, `drone_state`, `align_err`)

If drone moves away from the tag, tune sign mapping:

- `params.xy_map_sign_x_from_v`
- `params.xy_map_sign_y_from_u`

Flip only the axis that is reversed (set `1.0` <-> `-1.0`).
