# ROS2 串口屏触摸返回最小通信验证

## 目标

只验证串口屏触摸控件返回包，不实现禁飞区、航线规划、任务管理或动物统计。

期望触摸返回帧：

```text
65 00 01 00 FF FF FF
```

含义：

```text
0x65        触摸事件帧头
0x00        page_id = 0
0x01        component_id = 1
0x00        event = 0
FF FF FF    串口屏帧结束
```

## 硬件连接

- 目标设备：香橙派 3B
- 串口：UART2
- 默认设备名：`/dev/ttyS2`
- 默认波特率：`115200`
- 串口格式：`8N1`
- 接线要求：
  - 串口屏 TX 接香橙派 UART2 RX
  - 串口屏 RX 接香橙派 UART2 TX
  - GND 必须共地

## 工程内容

当前只包含一个 ROS2 C++ 节点：

```text
hmi_touch_verify_node
```

节点功能：

1. 打开串口 `/dev/ttyS2`。
2. 持续读取串口字节。
3. 查找 USART_HMI 触摸返回包：

   ```text
   65 page_id component_id event FF FF FF
   ```

4. 打印收到的完整十六进制帧。
5. 如果收到 `65 00 01 00 FF FF FF`，打印匹配成功。

## 编译

在 ROS2 Humble / Ubuntu 22.04 环境中执行：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 运行

默认使用 `/dev/ttyS2` 和 `115200`：

```bash
ros2 launch quadrotor_ground_station hmi_touch_verify.launch.py
```

如果串口设备名不同：

```bash
ros2 launch quadrotor_ground_station hmi_touch_verify.launch.py serial_port:=/dev/ttyS2 baud_rate:=115200
```

## 通过标准

点击串口屏上的目标触摸控件后，终端能看到类似输出：

```text
touch frame: 65 00 01 00 FF FF FF page=0 component=1 event=0
matched expected frame: 65 00 01 00 FF FF FF
```

如果没有输出，优先检查：

- 串口屏控件是否开启“触摸返回”
- TX/RX 是否交叉连接
- GND 是否共地
- 波特率是否一致
- UART2 是否启用
- 当前用户是否有 `/dev/ttyS2` 访问权限
