# Straight-Line Track Runner Recording

Draw the current default waypoint path:

```bash
python3 analyse/plot_track_waypoints.py
```

This writes `analyse/current_track_waypoints.svg` and `analyse/current_track_waypoints.csv`.

Record the closed-loop A->B straight segment:

```bash
cd ~/flight_ws/car
source install/setup.bash
python3 analyse/record_straight_line.py --stop-on-exit
```

Start the runner in another terminal:

```bash
ros2 topic pub --once /car/track_runner/command std_msgs/msg/String "{data: start}"
```

For a short 0.5 m straight test:

```bash
python3 analyse/record_straight_line.py \
  --straight-length-m 0.5 \
  --auto-stop-at-straight-end \
  --stop-on-exit
```

The CSV is written under `analyse/straight_line_logs/` by default. Useful columns:

- `pose_x`, `pose_y`, `pose_yaw`: latest `/car/pose`.
- `linear_x_m_s`, `angular_z_rad_s`: latest `/cmd_vel`.
- `cmd_vel_publishers`, `cmd_vel_subscribers`: graph counts for `/cmd_vel`.
- `state`, `index`, `progress_m`, `yaw_error_rad`, `target_x`, `target_y`, `reason`: parsed `/car/track_runner/status`.
- `target_dx`, `target_dy`: target point relative to the latest pose.

During the first straight segment, `target_dx` and `yaw_error_rad` should stay close to zero, while `target_dy` stays positive.

Record angular-rate tuner target/current angular velocity:

```bash
cd ~/flight_ws/car
source install/setup.bash
python3 analyse/record_angular_rate.py --stop-on-exit
```

The CSV and PNG tracking plot are written under `analyse/log/` by default. Useful columns:

- `target_w_rad_s`: target angular velocity from `/car/angular_rate_tuner/status`.
- `current_w_rad_s`: measured angular velocity, using status first and `/car/odom/carto` as fallback.
- `cmd_w_rad_s`, `w_error_rad_s`, `w_error_integral_rad`, `w_error_derivative_rad_s2`, `k_w_rate`, `k_i_rate`, `k_d_rate`: angular-rate tuner command and PID gain data.

Plot an existing angular-rate CSV:

```bash
python3 analyse/record_angular_rate.py --plot-only analyse/log/angular_rate_20260729_213525.csv
```

Disable automatic plotting while recording:

```bash
python3 analyse/record_angular_rate.py --no-plot
```

Record 0.75 m circle angle-loop target/current yaw and angular velocity:

```bash
cd ~/flight_ws/car
source install/setup.bash
python3 analyse/record_circle_angle.py --send-start --stop-on-exit
```

The CSV and PNG tracking plot are written under `analyse/log/` by default. Useful columns:

- `pose_yaw_rad`, `target_yaw_rad`, `yaw_error_rad`: angle-loop tracking data.
- `target_w_rad_s`, `measured_w_rad_s`, `cmd_w_rad_s`: angular-rate inner-loop data.
- `target_x`, `target_y`, `reference_theta_rad`: smooth circle target-point data.
- `k_w`, `k_w_rate`, `k_i_rate`, `k_d_rate`, `lookahead_m`: active tuning parameters.

Plot an existing circle-angle CSV:

```bash
python3 analyse/record_circle_angle.py --plot-only analyse/log/circle_angle_YYYYMMDD_HHMMSS.csv
```

Record live x-y pose and target points from track_runner, then plot on exit:

```bash
cd ~/flight_ws/car
source install/setup.bash
python3 analyse/record_xy.py --send-start --stop-on-exit
```

The script subscribes to `/car/pose` and `/car/track_runner/status` by default, writes a CSV under `analyse/log/`, and automatically writes an `_xy.png` plot with both actual pose and target path points.
