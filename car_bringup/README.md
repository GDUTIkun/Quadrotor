# car_bringup

整车建图启动包。默认拉起：

- `stm_bridge`
- LSN10P 雷达驱动
- `base_link -> laser` 静态 TF
- Cartographer 建图

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
base_link -> laser: x=0, y=0, z=0.088, roll=0, pitch=0, yaw=0
```

实车安装位置不同就用 launch 参数覆盖：

```bash
ros2 launch car_bringup mapping.launch.py laser_x:=0.05 laser_z:=0.12 laser_yaw:=0.0
```

