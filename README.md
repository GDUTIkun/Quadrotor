### 地面站

7 寸串口屏。

当前最小验证节点默认使用 Orange Pi 3B 的 UART7：

- 设备名：`/dev/ttyS7`
- 波特率：`115200`
- UART7 RX：40pin 物理 15 脚，`GPIO4_A2` / GPIO130，接串口屏 TX
- UART7 TX：40pin 物理 16 脚，`GPIO4_A3` / GPIO131，接串口屏 RX
- GND：40pin 任意 GND 脚，与串口屏 GND 共地

启用 UART7 overlay 后重启：

```bash
sudo nano /boot/orangepiEnv.txt
```

```text
overlays=uart7-m2
```

如果已有其他 overlay，保留在同一行并用空格分隔，例如：

```text
overlays=uart7-m2 i2c2-m1
```

重启后确认 UART7 已启用：

```bash
cat /proc/device-tree/serial@fe6b0000/status
stty -F /dev/ttyS7 -a
```

运行监听节点：

```bash
source /opt/ros/humble/setup.bash
source ~/flight_ws/install/setup.bash
ROS_LOCALHOST_ONLY=1 ros2 launch quadrotor_ground_station hmi_touch_verify.launch.py
```
