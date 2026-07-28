# stm_bridge

ROS2 与 STM32 底盘控制板的串口桥接包，协议见仓库根目录 `program.md` 的“STM 串口通信接口冻结版 v1.0”。

## 构建

```bash
colcon build --packages-select stm_bridge --symlink-install
source install/setup.bash
```

## 启动

香橙派侧执行：

```bash
ros2 launch stm_bridge stm_bridge.launch.py
```

当前默认使用 UART0_M2：

```text
/dev/ttyS0 @ 576000
```

如果以后加 udev 固定名，可临时覆盖实际串口：

```bash
ros2 launch stm_bridge stm_bridge.launch.py port:=/dev/wheeltec_controller baudrate:=576000
```

## UART0_M2 自发自收测试

UART0_M2 使用 `/dev/ttyS0`，波特率 `576000`。短接 Pin 8 TX(GPIO4_A3) 与 Pin 10 RX(GPIO4_A4) 后执行：

```bash
ros2 run stm_bridge uart_loopback_test
```

没有构建安装时，可直接从源码运行：

```bash
PYTHONPATH=car/stm_bridge python3 -m stm_bridge.uart_loopback_test
```

脚本会先测原始字节回环，再测一组 `CMD_VEL` 协议帧回环。通过时最后输出 `PASS`。

## ROS 接口

订阅：

```text
/cmd_vel    geometry_msgs/msg/Twist
```

发布：

```text
/track2vision/imu/data_valid    sensor_msgs/msg/Imu
/stm/status                     diagnostic_msgs/msg/DiagnosticArray
```

预留调试接口，默认关闭：

```text
/odom/wheel                     nav_msgs/msg/Odometry
```

小车首版的 `map -> odom -> base_link` 由 Cartographer 发布，`stm_bridge` 默认不发布 wheel odom，也不发布 `odom -> base_link` TF。

## 首轮实测顺序

1. STM 先 1 Hz 发送 `STATUS`。
2. ROS 侧检查：

```bash
ros2 topic echo /stm/status
```

3. STM 再 50 Hz 发送 `IMU`。
4. ROS 侧检查：

```bash
ros2 topic hz /track2vision/imu/data_valid
ros2 topic echo /track2vision/imu/data_valid --once
```

5. 抬轮后测试 ROS 下发速度：

```bash
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

停止发布后，ROS 会在 300 ms 超时后下发 `enable=0` 的零速帧；STM 侧也应做独立通信 watchdog。

如需临时看 STM 上报的轮式里程计，再显式打开：

```bash
ros2 launch stm_bridge stm_bridge.launch.py publish_wheel_odom:=true
```

## 样例帧

```text
# ROS -> STM, seq=0, v=0, w=0, enable=0
AA 55 01 00 05 00 00 00 00 00 C1 99

# ROS -> STM, seq=1, v=200mm/s, w=0, enable=1
AA 55 01 01 05 C8 00 00 00 01 F1 49

# STM -> ROS, seq=0, STATUS ready, 12V, stamp=1000ms
AA 55 83 00 0B E0 2E 00 00 01 00 00 E8 03 00 00 85 A4

# STM -> ROS, seq=0, IMU horizontal static, az=1000mg, stamp=1000ms
AA 55 81 00 16 00 00 00 00 E8 03 00 00 00 00 00 00 00 00 00 00 00 00 E8 03 00 00 1D 51
```
