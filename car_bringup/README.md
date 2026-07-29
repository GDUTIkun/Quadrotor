# car_bringup

整车建图启动包。默认拉起：

- `stm_bridge`
- LSN10P 雷达驱动
- `car_base_link -> car_laser` 静态 TF
- `car_base_link -> car_imu_link` 静态 TF
- Cartographer 建图
- `/car/pose` 小车定位坐标发布

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
```

实车安装位置不同就用 launch 参数覆盖：

```bash
ros2 launch car_bringup mapping.launch.py laser_x:=0.055 laser_z:=0.015 laser_yaw:=-1.5708
```

`/car/pose` 由 `car_localization` 从 Cartographer 的 `map -> base_link` TF 转换得到，默认坐标系为 `car_map`：

```text
+x 向右，+y 向前，+z 向上
```
