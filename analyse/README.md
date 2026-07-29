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
