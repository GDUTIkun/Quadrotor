# car_bringup

整车建图启动包。默认拉起：

- `stm_bridge`
- LSN10P 雷达驱动
- `car_base_link -> car_laser` 静态 TF
- `car_base_link -> car_imu_link` 静态 TF
- `car_base_link -> target` 静态 TF
- Cartographer 建图
- `/car/pose` 小车定位坐标发布
- `/car/target_pose` target 坐标发布

香橙派侧启动：

```bash
ros2 launch car_bringup mapping.launch.py
```

如果 STM 或雷达已单独启动，可以关闭对应部分：

```bash
ros2 launch car_bringup mapping.launch.py start_stm:=false
ros2 launch car_bringup mapping.launch.py start_lidar:=false
```

当前静态 TF 默认：

```text
car_base_link -> car_laser: x=0.055, y=0, z=0.015, roll=0, pitch=0, yaw=-1.5708
car_base_link -> car_imu_link: x=0, y=-0.0146, z=0.075, roll=0, pitch=0, yaw=0
car_base_link -> target: x=0.09553, y=0, z=0, roll=0, pitch=0, yaw=0
```

实车安装位置不同就用 launch 参数覆盖：

```bash
ros2 launch car_bringup mapping.launch.py laser_x:=0.055 laser_z:=0.015 laser_yaw:=-1.5708
```

`/car/pose` 和 `/car/target_pose` 由 `car_localization` 从 Cartographer TF 转换得到，默认坐标系为 `car_map`：

```text
+x 向右，+y 向前，+z 向上
```

## Track Runner Offboard

车端一条命令拉起 STM、雷达、Cartographer 建图、`/car/pose` 发布和 `track_runner`：

```bash
cd ~/car_ws
source install/setup.bash
ros2 launch car_bringup track_runner_offboard.launch.py
```

启动后 `track_runner` 默认处于 idle，只会持续发布零速，默认速度为 `0.01 m/s`，默认跑 `1` 圈。另一台设备加入同一个 ROS 2 网络后，只需要发 `start`：

```bash
export ROS_DOMAIN_ID=<和车端一致>
source install/setup.bash

ros2 topic pub --once /car/track_runner/command std_msgs/msg/String "{data: start}"
```

运行中调速直接再次发布速度：

```bash
ros2 topic pub --once /car/track_runner/speed std_msgs/msg/Float64 "{data: 0.03}"
```

暂停、继续和停止：

```bash
ros2 topic pub --once /car/track_runner/command std_msgs/msg/String "{data: pause}"
ros2 topic pub --once /car/track_runner/command std_msgs/msg/String "{data: resume}"
ros2 topic pub --once /car/track_runner/command std_msgs/msg/String "{data: stop}"
```

常用覆盖参数：

```bash
ros2 launch car_bringup track_runner_offboard.launch.py \
  default_speed_m_s:=0.01 \
  default_laps:=1 \
  straight_length_m:=1.35 \
  radius_m:=0.75
```

使用时确保 `/cmd_vel` 只有 `track_runner_node` 一个发布者，不要同时启动其他路径控制器或手动发布 `/cmd_vel`。
