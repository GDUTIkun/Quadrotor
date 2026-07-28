# ROS2 定位小车工程方案

## 1. 项目目标

本工程目标是实现一台基于 ROS2 的 2D 激光定位小车：

- 使用 2D 激光雷达 + Cartographer 完成建图与纯定位。
- STM 侧负责底盘控制，接收 ROS 侧速度指令，并完成轮速解算与速度闭环追踪。
- ROS 与 STM 通过串口通信，ROS 下发运动指令，STM 上传 IMU、轮式里程计和底盘状态。
- 建图完成后，ROS 侧实现轻量级路径规划与路径跟踪逻辑，使小车能够根据目标点绕开少量规则禁入区并运动到指定位置。

首版不接入 Nav2，不实现动态避障和复杂恢复行为。路径环境假设为已知静态场地，障碍物最多只有几块规则禁入区；ROS 侧用轻量规划算法生成路径点，再由路径跟踪控制器输出 `/cmd_vel`，优先保证建图、定位、底盘通信、静态绕行和到点停车闭环跑通。

## 2. 当前工程现状

当前工作区已有：

- `carto`
  - Cartographer 建图配置：`my_laser_with_imu.lua`
  - Cartographer 建图启动文件：`my_laser_with_imu.launch.py`
  - Cartographer 纯定位配置：`backpack_2d_localization.lua`
  - Cartographer 纯定位启动文件：`my_backpack_2d_localization.launch.py`
  - 地图目录：`carto/map/`

- `lsn10_lidar`
  - LSN10 激光雷达 ROS2 驱动。
  - 当前雷达发布 `/scan`。
  - 当前雷达 frame 为 `laser`。
  - 当前雷达参数中串口默认仍有 `/dev/ttyACM0`，后续应固定为 `/dev/wheeltec_lidar`。

Cartographer 当前关键设定：

- `map_frame = "map"`
- `tracking_frame = "base_link"`
- `published_frame = "base_link"`
- `odom_frame = "odom"`
- `provide_odom_frame = true`
- `use_odometry = false`
- `TRAJECTORY_BUILDER_2D.use_imu_data = true`
- IMU remap 到 `/track2vision/imu/data_valid`

因此首版 TF 链路应保持：

```text
map -> odom -> base_link -> laser
```

其中：

- `map -> odom -> base_link` 由 Cartographer 发布。
- `base_link -> laser` 由静态 TF 发布。
- 小车首版不由 STM/ROS 桥发布 odom；`map -> odom -> base_link` 统一由 Cartographer 发布，避免 TF 或 odom 来源冲突。

## 2.1 当前 map 实现方式

当前建图方式是 Cartographer 2D 激光 + IMU 建图，数据链路如下：

```text
LSN10P lidar       -> /scan                         -> Cartographer
STM IMU over UART  -> stm_bridge                    -> /track2vision/imu/data_valid -> Cartographer
static TF launch   -> base_link -> laser            -> Cartographer 查找雷达外参
Cartographer       -> map -> odom -> base_link      -> RViz/后续路径规划与控制使用
Cartographer       -> /map                          -> occupancy grid 地图显示
```

具体实现：

- `lslidar_driver` 发布 `/scan`，`frame_id = laser`。
- `stm_bridge` 解析 STM 上传的 `0x81 IMU` 帧，发布 `/track2vision/imu/data_valid`，`frame_id = base_link`。
- `car_bringup/launch/mapping.launch.py` 使用 `tf2_ros/static_transform_publisher` 发布静态 TF：`base_link -> laser`。
- `carto/my_laser_with_imu.launch.py` 启动 `cartographer_node`，将 Cartographer 的 `imu` remap 到 `/track2vision/imu/data_valid`，`scan` 保持使用 `/scan`。
- `carto/my_laser_with_imu.lua` 配置 `TRAJECTORY_BUILDER_2D.use_imu_data = true`，并设置 `num_laser_scans = 1`。
- `cartographer_occupancy_grid_node` 根据 Cartographer 子图发布 `/map`，默认分辨率 `0.05 m`。

Cartographer frame 责任边界：

- `map_frame = map`
- `odom_frame = odom`
- `tracking_frame = base_link`
- `published_frame = base_link`
- `provide_odom_frame = true`
- `use_odometry = false`

因此当前 map 的位姿来源不是 STM 轮式里程计，而是 Cartographer 使用 `/scan` 和 IMU 做 2D scan matching 后发布的 TF。STM 的职责是提供 IMU 和执行速度指令；小车首版不通过 STM 发布 `odom -> base_link`。

`yaw + pi/2` 当前不参与建图链路。它只是参考原飞机 `track2vision` 工程时发现的 MAVROS 外部视觉坐标适配逻辑，小车是否需要该 yaw offset 待实车观察后决定。

## 3. 工程模块划分

建议新增四个 ROS2 包：

```text
car_ws/
  carto/
  lsn10_lidar/
  stm_bridge/
  path_planner/
  path_controller/
  car_bringup/
  program.md
```

### 3.1 `stm_bridge`

职责：

- 打开 STM 串口。
- 订阅 ROS 侧 `/cmd_vel`。
- 周期性向 STM 下发底盘速度指令。
- 解析 STM 上传帧。
- 发布 IMU 和 STM 状态。
- 轮式里程计上行协议保留为调试/后续融合接口，首版 ROS 节点默认不发布 `/odom/wheel`。
- 实现通信 watchdog，ROS 指令超时后自动下发零速。

ROS 接口：

| 方向 | Topic | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS 到 STM 的速度指令来源 |
| 发布 | `/track2vision/imu/data_valid` | `sensor_msgs/msg/Imu` | Cartographer 使用的 IMU |
| 发布 | `/stm/status` | `diagnostic_msgs/msg/DiagnosticArray` | 电压、电流、状态机、错误码、通信统计 |
| 预留 | `/odom/wheel` | `nav_msgs/msg/Odometry` | 默认关闭，仅调试或后续融合时启用 |

默认参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `port` | `/dev/ttyS0` | STM 串口设备，UART0_M2 |
| `baudrate` | `576000` | 串口波特率 |
| `cmd_rate_hz` | `50.0` | 速度指令下发频率 |
| `cmd_timeout_s` | `0.3` | `/cmd_vel` 超时时间 |
| `base_frame_id` | `base_link` | IMU 与底盘 frame |
| `publish_wheel_odom` | `false` | 是否发布 `/odom/wheel`，首版关闭 |
| `publish_odom_tf` | `false` | 必须保持关闭，`odom -> base_link` 由 Cartographer 发布 |
| `yaw_offset_rad` | `0.0` | 坐标修正预留；是否需要 `+pi/2` 待实车测试确认 |

### 3.2 `path_planner`

职责：

- 接收地图坐标系下的目标点。
- 查询 `map -> base_link` TF。
- 读取静态禁入区配置。
- 将禁入区按小车半径和安全余量膨胀。
- 判断当前位置到目标点的直线路径是否穿过禁入区。
- 若直线路径可行，直接生成起点到目标点的路径。
- 若直线路径不可行，使用轻量可见图算法绕过禁入区角点。
- 发布 `nav_msgs/msg/Path`，交给 `path_controller` 跟踪。

ROS 接口：

| 方向 | Topic | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 地图坐标系目标点 |
| 发布 | `/planned_path` | `nav_msgs/msg/Path` | 从当前位姿到目标点的路径 |
| 发布 | `/path_planner/status` | `std_msgs/msg/String` | 当前规划状态、路径长度、失败原因 |

首版路径规划策略：

1. 将每块禁入区建模为 `map` 坐标系下的轴对齐矩形。凸多边形作为预留能力，首轮实车优先使用矩形。
2. 对禁入区做几何膨胀，膨胀距离为：

```text
inflate_radius = robot_radius_m + safety_margin_m
```

3. 先检查 `start -> goal` 直线是否与任意膨胀禁入区相交。
4. 若不相交，路径为 `[start, goal]`。
5. 若相交，构建可见图：
   - 节点包含 `start`、`goal` 和所有膨胀禁入区顶点。
   - 两节点连线不穿过任何膨胀禁入区时，建立一条边。
   - 边权重为两点欧氏距离。
   - 使用 Dijkstra 求最短路径。
6. 输出路径点时删除距离过近的重复点，并保留目标点 yaw。

首版路径规划不使用离线栅格地图，也不订阅 `/map` 来提取障碍物。障碍区域是已知静态区域，由我们手动写入 `keepout_zones.yaml`；默认配置为 `keepout_zones: []`，表示当前没有禁入区。路径规划先采用可见图而不是栅格 A*，因为场地内只有少量规则禁入区，可见图生成的路径点更少，调试也更直接。

默认参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `global_frame_id` | `map` | 目标点坐标系 |
| `base_frame_id` | `base_link` | 小车本体坐标系 |
| `obstacle_file` | `config/keepout_zones.yaml` | 静态禁入区配置文件 |
| `robot_radius_m` | `0.18` | 小车近似半径，按实车外形调整 |
| `safety_margin_m` | `0.08` | 禁入区额外安全余量 |
| `min_waypoint_spacing_m` | `0.10` | 输出路径点最小间距 |
| `max_plan_length_m` | `20.0` | 超过该长度视为异常规划 |

禁入区配置示例：

```yaml
frame_id: map
keepout_zones:
  - name: table_1
    type: rectangle
    center: [1.20, 0.60]
    size: [0.80, 0.50]
  - name: box_1
    type: polygon
    points:
      - [2.00, -0.20]
      - [2.50, -0.20]
      - [2.50, 0.30]
      - [2.00, 0.30]
```

### 3.3 `path_controller`

职责：

- 订阅 `path_planner` 发布的 `/planned_path`。
- 查询 `map -> base_link` TF。
- 使用路径跟踪算法计算线速度和角速度。
- 输出 `/cmd_vel`，由 `stm_bridge` 下发给 STM。
- 到达终点后执行末端 yaw 调整并停车。

ROS 接口：

| 方向 | Topic | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/planned_path` | `nav_msgs/msg/Path` | 待跟踪路径 |
| 发布 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 输出给底盘的速度指令 |
| 发布 | `/path_controller/status` | `std_msgs/msg/String` | 当前控制阶段和误差状态 |

首版路径跟踪策略：

1. 若当前没有路径，持续发布零速。
2. 收到路径后，先找到离当前车体最近的路径段。
3. 沿路径向前取一个前视点 `lookahead_distance_m`。
4. 将前视点转换到 `base_link` 坐标系。
5. 若前视点方向角误差较大，先低速原地转向。
6. 航向误差进入阈值后，用 Pure Pursuit 计算角速度：

```text
curvature = 2 * y / lookahead_distance^2
w = v * curvature
```

其中 `y` 是前视点在 `base_link` 坐标系下的横向偏差。

7. 线速度按路径剩余距离和转弯强度限速：
   - 剩余距离越短，速度越低。
   - 角速度越大，速度越低。
8. 到达终点位置阈值后停止线速度，只调整末端 yaw。
9. 位置和末端 yaw 都进入阈值后，发布零速并标记到达。

默认参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `global_frame_id` | `map` | 路径坐标系 |
| `base_frame_id` | `base_link` | 小车本体坐标系 |
| `control_rate_hz` | `20.0` | 控制频率 |
| `xy_tolerance_m` | `0.05` | 到点距离阈值 |
| `yaw_tolerance_rad` | `0.087` | 到点角度阈值，约 5 度 |
| `heading_tolerance_rad` | `0.15` | 前进前的航向阈值 |
| `lookahead_distance_m` | `0.35` | Pure Pursuit 前视距离 |
| `v_max_m_s` | `0.25` | 最大线速度 |
| `w_max_rad_s` | `0.8` | 最大角速度 |
| `v_min_m_s` | `0.03` | 最小有效线速度 |
| `w_min_rad_s` | `0.08` | 最小有效角速度 |
| `goal_slowdown_radius_m` | `0.40` | 接近终点时开始降速的距离 |

### 3.4 `car_bringup`

职责：

- 统一启动整车相关节点。
- 提供建图 launch。
- 提供纯定位 + 轻量导航控制 launch。
- 发布 `base_link -> laser` 静态 TF。
- 避免每次手动启动多个 launch。

建议提供两个启动文件：

```text
car_bringup/launch/mapping.launch.py
car_bringup/launch/localization_control.launch.py
```

`mapping.launch.py` 启动：

- LSN10 雷达驱动。
- STM 串口桥。
- `base_link -> laser` 静态 TF。
- Cartographer 建图。

`localization_control.launch.py` 启动：

- LSN10 雷达驱动。
- STM 串口桥。
- `base_link -> laser` 静态 TF。
- Cartographer 纯定位。
- `path_planner` 静态禁入区路径规划节点。
- `path_controller` 路径跟踪控制节点。

## 4. STM 串口通信接口冻结版 v1.0

本节作为 ROS 侧与 STM 侧的对接依据。STM 侧先按这里的帧格式发 `STATUS` 和 `IMU`，ROS 侧能解析并发布 topic 后，再联调 `CMD_VEL` 下发和轮式里程计。

### 4.1 串口物理层

| 项 | 约定 |
| --- | --- |
| 波特率 | `576000` |
| 数据位 | 8 |
| 校验位 | none |
| 停止位 | 1 |
| 流控 | none |
| 字节序 | little-endian |
| 有符号数 | 二进制补码 |
| ROS 默认设备 | `/dev/ttyS0` |

香橙派侧使用 UART0_M2：Pin 8 TX(GPIO4_A3)、Pin 10 RX(GPIO4_A4)。

### 4.2 通用帧格式

```text
header[2] + msg_id[1] + seq[1] + len[1] + payload[len] + crc16[2]
```

| 字段 | 长度 | 说明 |
| --- | --- | --- |
| `header` | 2 | 固定 `0xAA 0x55` |
| `msg_id` | 1 | 消息 ID |
| `seq` | 1 | 帧序号，0-255 循环 |
| `len` | 1 | payload 字节数 |
| `payload` | N | 数据区，最大 64 字节 |
| `crc16` | 2 | CRC-16/IBM，小端，低字节在前 |

解析规则：

- 收到非 `0xAA 0x55` 开头的数据时，逐字节丢弃直到重新找到帧头。
- `len > 64` 视为异常帧，丢弃当前帧头并重新同步。
- CRC 错误时丢弃当前帧头并重新同步。
- 允许半包和粘包，ROS 侧解析器必须能缓存未完整帧。

CRC 规则：

| 项 | 值 |
| --- | --- |
| 名称 | CRC-16/IBM，也称 CRC-16/ARC |
| 多项式 | `0x8005` |
| 反向实现多项式 | `0xA001` |
| 初值 | `0x0000` |
| xorout | `0x0000` |
| 输入反射 | true |
| 输出反射 | true |
| 覆盖范围 | `msg_id + seq + len + payload` |
| 不覆盖 | `header` 和 `crc16` 本身 |

校验向量：

```text
crc16_ibm("123456789") = 0xBB3D
```

### 4.3 消息总表

| msg_id | 方向 | 名称 | payload 长度 | 频率建议 |
| --- | --- | --- | --- | --- |
| `0x01` | ROS -> STM | `CMD_VEL` | 5 | 50 Hz |
| `0x81` | STM -> ROS | `IMU` | 22 | 50-100 Hz |
| `0x82` | STM -> ROS | `WHEEL_ODOM` | 24 | 预留，默认不要求发送 |
| `0x83` | STM -> ROS | `STATUS` | 11 | 1-10 Hz |

首轮通信测试最低要求：

- STM 至少发送 `0x83 STATUS`，用于确认串口、帧头、长度和 CRC。
- 然后发送 `0x81 IMU`，用于启动 Cartographer 需要的 `/track2vision/imu/data_valid`。
- `0x82 WHEEL_ODOM` 是预留调试接口，首版不要求 STM 发送，ROS 默认不发布 `/odom/wheel`。

### 4.4 ROS -> STM：`0x01 CMD_VEL`

ROS 以固定频率下发底盘速度指令。STM 收到后继续负责轮速解算和速度闭环。

payload，长度 5：

```text
int16 v_mm_s
int16 w_mrad_s
uint8 enable
```

| 字段 | 单位 | 正方向 | 说明 |
| --- | --- | --- | --- |
| `v_mm_s` | mm/s | 前进为正 | 车体 x 方向线速度 |
| `w_mrad_s` | mrad/s | 逆时针为正 | 车体 z 轴角速度 |
| `enable` | bool | 1 使能 | `0` 表示停车 |

ROS 侧转换：

```text
v_mm_s = round(cmd_vel.linear.x * 1000)
w_mrad_s = round(cmd_vel.angular.z * 1000)
```

安全约定：

- ROS 正常工作时 50 Hz 连续发送 `CMD_VEL`。
- ROS 超过 300 ms 未收到新的 `/cmd_vel` 时，下发 `v=0, w=0, enable=0`。
- STM 也必须做独立 watchdog，建议 300-500 ms 未收到有效 `CMD_VEL` 后立即停车。
- STM 收到 `enable=0` 时应立即停车，并清除速度目标。

样例帧：

```text
# seq=0, v=0 mm/s, w=0 mrad/s, enable=0
AA 55 01 00 05 00 00 00 00 00 C1 99

# seq=1, v=200 mm/s, w=0 mrad/s, enable=1
AA 55 01 01 05 C8 00 00 00 01 F1 49
```

### 4.5 STM -> ROS：`0x81 IMU`

payload，长度 22：

```text
int16 ax_mg
int16 ay_mg
int16 az_mg
int16 gx_mdps
int16 gy_mdps
int16 gz_mdps
int16 yaw_cdeg
int16 pitch_cdeg
int16 roll_cdeg
uint32 stamp_ms
```

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `ax_mg` | mg | x 轴线加速度 |
| `ay_mg` | mg | y 轴线加速度 |
| `az_mg` | mg | z 轴线加速度，水平静止约 `+1000` |
| `gx_mdps` | mdps | x 轴角速度 |
| `gy_mdps` | mdps | y 轴角速度 |
| `gz_mdps` | mdps | z 轴角速度 |
| `yaw_cdeg` | 0.01 deg | yaw，绕 z 轴 |
| `pitch_cdeg` | 0.01 deg | pitch，绕 y 轴 |
| `roll_cdeg` | 0.01 deg | roll，绕 x 轴 |
| `stamp_ms` | ms | STM 上电后的毫秒时间戳 |

ROS 发布：

- Topic：`/track2vision/imu/data_valid`
- 类型：`sensor_msgs/msg/Imu`
- `header.frame_id = base_link`
- ROS 首版使用接收时刻作为 `header.stamp`，`stamp_ms` 先用于诊断和时序检查。

单位转换：

```text
linear_acceleration = mg * 9.80665 / 1000
angular_velocity = mdps * pi / (180 * 1000)
euler_angle = cdeg * pi / (180 * 100)
```

坐标约定：

- 使用 ROS 右手系。
- x 向前。
- y 向左。
- z 向上。
- yaw 绕 z 轴，逆时针为正。

样例帧：

```text
# seq=0, 水平静止：ax=0, ay=0, az=1000mg, gyro=0, yaw/pitch/roll=0, stamp=1000ms
AA 55 81 00 16 00 00 00 00 E8 03 00 00 00 00 00 00 00 00 00 00 00 00 E8 03 00 00 1D 51
```

### 4.6 STM -> ROS：`0x82 WHEEL_ODOM`，预留接口

小车首版不使用 STM 里程计参与建图或定位，也不让串口桥发布 odom。该消息仅作为调试或后续融合预留；未启用 `publish_wheel_odom` 时，ROS 侧即使收到该帧也不发布 `/odom/wheel`。

payload，长度 24：

```text
int32 x_mm
int32 y_mm
int32 yaw_mrad
int16 vx_mm_s
int16 wz_mrad_s
int16 left_mm_s
int16 right_mm_s
uint32 stamp_ms
```

| 字段 | 单位 | 正方向 | 说明 |
| --- | --- | --- | --- |
| `x_mm` | mm | 前进为正 | STM 积分得到的 x |
| `y_mm` | mm | 左侧为正 | STM 积分得到的 y |
| `yaw_mrad` | mrad | 逆时针为正 | STM 积分得到的 yaw |
| `vx_mm_s` | mm/s | 前进为正 | 当前车体线速度 |
| `wz_mrad_s` | mrad/s | 逆时针为正 | 当前车体角速度 |
| `left_mm_s` | mm/s | 前进为正 | 左轮线速度 |
| `right_mm_s` | mm/s | 前进为正 | 右轮线速度 |
| `stamp_ms` | ms | 单调递增 | STM 上电后的毫秒时间戳 |

ROS 发布：

- Topic：`/odom/wheel`
- 类型：`nav_msgs/msg/Odometry`
- `header.frame_id = odom`
- `child_frame_id = base_link`
- 默认不发布。
- 即使后续打开 `/odom/wheel` 调试，也不发布 `odom -> base_link` TF。
- 小车项目的 `map -> odom -> base_link` 统一由 Cartographer 发布。

样例帧：

```text
# seq=0, 全零里程计，stamp=1000ms
AA 55 82 00 18 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 E8 03 00 00 38 45
```

### 4.7 STM -> ROS：`0x83 STATUS`

payload，长度 11：

```text
uint16 voltage_mv
int16 current_ma
uint8 state
uint16 error_flags
uint32 stamp_ms
```

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `voltage_mv` | mV | 电池或母线电压 |
| `current_ma` | mA | 当前电流；没有电流采样时填 0 |
| `state` | enum | STM 当前状态 |
| `error_flags` | bitmask | 错误标志 |
| `stamp_ms` | ms | STM 上电后的毫秒时间戳 |

ROS 发布：

- Topic：`/stm/status`
- 类型：`diagnostic_msgs/msg/DiagnosticArray`

状态枚举：

| state | 含义 |
| --- | --- |
| 0 | INIT |
| 1 | READY |
| 2 | RUNNING |
| 3 | ESTOP |
| 4 | ERROR |

错误位：

| bit | 含义 |
| --- | --- |
| 0 | CMD_TIMEOUT |
| 1 | IMU_ERROR |
| 2 | ENCODER_ERROR |
| 3 | MOTOR_ERROR |
| 4 | LOW_VOLTAGE |
| 5 | CRC_ERROR |

样例帧：

```text
# seq=0, voltage=12000mV, current=0mA, state=READY, error=0, stamp=1000ms
AA 55 83 00 0B E0 2E 00 00 01 00 00 E8 03 00 00 85 A4
```

### 4.8 STM 侧首轮测试发送顺序

为了先验证 ROS 串口桥，STM 可按以下顺序发送：

1. 1 Hz 发送 `STATUS`，确认 ROS 能收到 `/stm/status`。
2. 50 Hz 发送水平静止 `IMU`，确认 ROS 能收到 `/track2vision/imu/data_valid`。
3. ROS 发布 `/cmd_vel` 后，STM 打印收到的 `CMD_VEL` 中的 `v_mm_s`、`w_mrad_s` 和 `enable`。
4. 停止 ROS `/cmd_vel` 输入后，确认 STM 在 watchdog 时间内停车。
5. 如需调试轮式里程计，再临时打开 `publish_wheel_odom` 并发送 `WHEEL_ODOM`。

## 5. TF 与坐标系设计

### 5.1 坐标系

| frame | 说明 |
| --- | --- |
| `map` | Cartographer 地图坐标系 |
| `odom` | Cartographer 输出的局部连续坐标系 |
| `base_link` | 小车本体坐标系 |
| `laser` | 激光雷达坐标系 |

### 5.2 静态 TF

必须发布：

```text
base_link -> laser
```

建议做成 launch 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `laser_x` | `0.0` | 雷达相对 base_link 的 x |
| `laser_y` | `0.0` | 雷达相对 base_link 的 y |
| `laser_z` | `0.15` | 雷达相对 base_link 的 z |
| `laser_roll` | `0.0` | 雷达 roll |
| `laser_pitch` | `0.0` | 雷达 pitch |
| `laser_yaw` | `0.0` | 雷达 yaw |

实际安装后需要按机械结构修改。

### 5.3 坐标修正待测项

参考原飞机 `track2vision` 工程时发现，飞机侧曾在从 Cartographer 姿态转换到 MAVROS 外部视觉时使用过 `yaw + pi/2`。该修正属于飞机/MAVROS 坐标适配逻辑，小车首版不默认启用。

小车实测时按以下顺序确认：

1. 不加 yaw offset，观察 Cartographer 中 `base_link` 朝向是否与车头一致。
2. 给小车原地逆时针转动，确认 `base_link` yaw 是否按 ROS 右手系增大。
3. 若发现车头方向与地图显示固定相差 90 度，再考虑设置 `yaw_offset_rad = pi/2`。
4. 该修正只能加在明确需要坐标适配的位置，不能同时在 IMU、TF、控制器多处重复修正。

## 6. 建图流程

### 6.1 香橙派依赖安装

雷达驱动 `lslidar_driver` 构建依赖 `diagnostic_updater`。香橙派侧构建整车工作区前先确认该包已安装：

```bash
sudo apt update
sudo apt install ros-jazzy-diagnostic-updater
```

当前实机补充：

- `sudo` 密码是一个空格。
- 若安装时报 `404 Not Found`，通常是 apt 本地索引和镜像仓库不同步。先执行 `sudo apt update` 后重试。
- 如果 `apt update` 后仍然 404，临时切换 ROS2 apt 源或等待镜像同步，再安装 `ros-jazzy-diagnostic-updater`。
- 不要在缺少该依赖时反复构建雷达驱动；先解决系统依赖。

建议安装完成后构建：

```bash
colcon build --packages-up-to car_bringup --symlink-install
source install/setup.bash
```

### 6.2 启动前检查

检查串口：

```bash
ls -l /dev/wheeltec_lidar
ls -l /dev/ttyS0
```

检查雷达：

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

检查 IMU：

```bash
ros2 topic hz /track2vision/imu/data_valid
ros2 topic echo /track2vision/imu/data_valid --once
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo base_link laser
```

### 6.3 建图启动

推荐统一启动：

```bash
ros2 launch car_bringup mapping.launch.py
```

启动内容：

- 雷达驱动。
- STM 串口桥。
- 静态 TF。
- Cartographer 建图。

启动后的建图关系：

- `stm_bridge_node` 只提供 `/track2vision/imu/data_valid` 和 `/stm/status`，默认不提供 `/odom/wheel`。
- `base_link_to_laser_tf` 提供 `base_link -> laser`，用于让 Cartographer 将 `/scan` 从雷达坐标转换到车体坐标。
- `cartographer_node` 消费 `/scan` 和 `/track2vision/imu/data_valid`，并发布 `map -> odom -> base_link`。
- `cartographer_occupancy_grid_node` 发布 `/map`，用于 RViz 显示和保存地图前检查。

建图启动后建议检查：

```bash
ros2 topic hz /scan
ros2 topic hz /track2vision/imu/data_valid
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo map base_link
```

### 6.4 保存地图

建图完成后保存 `.pbstream`：

```bash
ros2 service call /cartographer_node/finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /cartographer_node/write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/t/car_ws/carto/map/my_map.pbstream', include_unfinished_submaps: true}"
```

地图统一保存到：

```text
carto/map/
```

## 7. 纯定位与轻量导航流程

### 7.1 启动纯定位

```bash
ros2 launch car_bringup localization_control.launch.py load_state_filename:=/home/t/car_ws/carto/map/my_map.pbstream
```

启动内容：

- 雷达驱动。
- STM 串口桥。
- 静态 TF。
- Cartographer 纯定位。
- `path_planner` 路径规划节点。
- `path_controller` 路径跟踪节点。

### 7.2 配置静态禁入区

首版禁入区使用 `path_planner/config/keepout_zones.yaml` 手动配置。路径规划器不读取离线地图文件，也不从 `/map` 中识别障碍；默认 `keepout_zones: []`，表示没有禁入区。后续只需要把已知障碍区域按 `map` 坐标系写入该 YAML 文件。

矩形障碍区优先使用：

```yaml
frame_id: map
keepout_zones:
  - name: table_1
    type: rectangle
    center: [1.20, 0.60]
    size: [0.80, 0.50]
```

不规则但仍然简单的障碍区使用：

```yaml
frame_id: map
keepout_zones:
  - name: area_1
    type: polygon
    points:
      - [2.00, -0.20]
      - [2.50, -0.20]
      - [2.50, 0.30]
      - [2.00, 0.30]
```

禁入区只表达“不能进入”的已知静态区域，不负责动态避障。实车测试时应额外保留人工急停和低速限制。

### 7.3 发送目标点

目标点使用 `/goal_pose`：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 1.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

目标点进入系统后的处理链路：

```text
/goal_pose
  -> path_planner 查询 map -> base_link 并生成 /planned_path
  -> path_controller 跟踪 /planned_path 并输出 /cmd_vel
  -> stm_bridge 将 /cmd_vel 转为 CMD_VEL 串口帧
  -> STM 执行底盘速度闭环
```

STM 只需要继续执行速度指令，不需要理解目标点、地图或禁入区。

## 8. 实施步骤

### 阶段 1：文档与接口冻结

- 完成 `program.md`。
- 确认 STM 串口协议字段、单位、坐标系。
- 确认雷达和 STM 的 udev 名称。
- 确认 `base_link -> laser` 的实际安装参数。

### 阶段 2：实现 `stm_bridge`

- 新建 ROS2 C++ 包 `stm_bridge`。
- 实现 CRC-16/IBM。
- 实现二进制帧打包和增量解析。
- 实现 `/cmd_vel` 到 `CMD_VEL` 串口帧。
- 实现 `IMU`、`WHEEL_ODOM`、`STATUS` 三类上行帧解析。
- 发布 `/track2vision/imu/data_valid` 和 `/stm/status`。
- `WHEEL_ODOM` 解析逻辑作为预留接口；首版默认不发布 `/odom/wheel`。
- 加入 ROS 侧 `/cmd_vel` watchdog。

### 阶段 3：实现 `path_planner`

- 新建 ROS2 C++ 包 `path_planner`。
- 增加 `config/keepout_zones.yaml`。
- 订阅 `/goal_pose`。
- 查询 `map -> base_link` 作为规划起点。
- 实现矩形和多边形禁入区加载。
- 实现禁入区膨胀。
- 实现线段与禁入区相交检测。
- 实现可见图构建和 Dijkstra 最短路径搜索。
- 发布 `/planned_path`。
- 发布 `/path_planner/status`。

### 阶段 4：实现 `path_controller`

- 新建 ROS2 C++ 包 `path_controller`。
- 订阅 `/planned_path`。
- 查询 `map -> base_link`。
- 实现 Pure Pursuit 路径跟踪。
- 实现接近终点降速。
- 实现末端 yaw 调整。
- 输出 `/cmd_vel`。
- 到点或路径失效后自动发布零速。

### 阶段 5：实现 `car_bringup`

- 新建 ROS2 包 `car_bringup`。
- 增加 `mapping.launch.py`。
- 增加 `localization_control.launch.py`。
- 在 launch 中发布 `base_link -> laser` 静态 TF。
- 在 launch 中启动 `path_planner` 和 `path_controller`。
- 将雷达串口参数固定为 `/dev/wheeltec_lidar`。
- 将 STM 串口参数固定为 UART0_M2：`/dev/ttyS0`、`576000`。

### 阶段 6：联调

联调顺序：

1. 只测串口协议。
2. 测 STM 上传 IMU。
3. 测雷达 `/scan`。
4. 测静态 TF。
5. 启动 Cartographer 建图。
6. 保存 `.pbstream`。
7. 启动纯定位。
8. 配置一块简单矩形禁入区。
9. 离线测试 `/goal_pose -> /planned_path` 是否绕开禁入区。
10. 抬轮测试 `path_controller` 输出方向。
11. 低速实车测试直线目标。
12. 低速实车测试绕单个禁入区。
13. 低速实车测试连续多个目标点。

## 9. 测试计划

### 9.1 串口协议测试

- CRC 已知向量测试。
- 单帧解析测试。
- 半包测试。
- 粘包测试。
- 噪声字节恢复测试。
- CRC 错误丢帧测试。
- payload 长度异常测试。

### 9.2 ROS 通信测试

```bash
ros2 topic hz /cmd_vel
ros2 topic hz /track2vision/imu/data_valid
ros2 topic echo /stm/status
```

验证项：

- `/cmd_vel` 正常下发。
- `/cmd_vel` 停止后 300 ms 内下发零速。
- IMU frame 为 `base_link`。
- `/stm/status` 能显示错误码和通信统计。
- 默认不存在 `/odom/wheel`，小车 odom/TF 由 Cartographer 发布。

### 9.3 Cartographer 测试

验证项：

- `/scan` 正常。
- `/track2vision/imu/data_valid` 正常。
- `base_link -> laser` 存在。
- Cartographer 不再因为缺少 IMU 或 TF 卡住。
- 建图过程中 `map -> odom -> base_link` 连续发布。
- 保存后的 `.pbstream` 可被纯定位 launch 加载。

### 9.4 路径规划测试

测试项：

- 无禁入区时，规划结果应为起点到目标点的直线路径。
- 直线路径不穿过禁入区时，不应生成多余绕行点。
- 直线路径穿过单个矩形禁入区时，应绕膨胀后的角点通行。
- 多块禁入区同时存在时，路径不得穿过任意膨胀禁入区。
- 目标点位于禁入区内时，应拒绝规划并发布失败状态。
- 起点位于禁入区内时，应拒绝规划并发布失败状态。

验收标准：

- `/planned_path` 的 `header.frame_id` 为 `map`。
- 路径点间距不过密，路径长度不超过合理上限。
- 所有路径线段均不进入膨胀后的禁入区。
- 规划失败时 `path_controller` 不应继续执行旧路径。

### 9.5 路径跟踪控制测试

测试点：

- 原地旋转目标。
- 前方 1 m 目标。
- 左前方目标。
- 右前方目标。
- 连续 3 个目标点。
- 绕过一块矩形禁入区后的目标点。

验收标准：

- 到点后自动停车。
- 目标方向错误时优先转向，不盲目前进。
- 路径跟踪过程中线速度和角速度连续、限幅有效。
- 接近终点时能够降速。
- 通信断开或 `/cmd_vel` 超时时底盘停车。
- 低速测试无明显震荡。

## 10. 风险与注意事项

- 雷达串口和 STM 串口必须用 udev 固定名称，不能依赖 `/dev/ttyACM0`。
- STM 与 ROS 的坐标系必须完全一致，否则 Cartographer、路径规划和路径跟踪都会异常。
- Cartographer 当前使用 IMU，IMU 的 `frame_id`、方向、单位必须正确。
- 小车首版不发布 STM 轮式 odom；`map -> odom -> base_link` 由 Cartographer 统一发布。
- 原飞机工程中的 `yaw + pi/2` 是待实测项，不默认启用，避免未验证的坐标补偿污染 IMU、TF 或控制逻辑。
- 首版只支持静态禁入区绕行，不支持动态避障；场地里出现临时障碍物时需要人工干预或急停。
- 禁入区坐标必须与保存后的 Cartographer 地图一致；地图重建后应重新核对禁入区配置。
- 禁入区膨胀半径必须大于小车实际外形半径和定位误差，否则规划路径可能贴边过近。
- Pure Pursuit 前视距离过小会抖动，过大会切弯明显；实车需要低速调参。
- 实车第一次测试必须限速，并保证急停可用。

## 11. 实机部署前待确认

通信接口按第 4 节冻结为 v1.0。以下问题不再影响协议字段，只影响香橙派实机参数和标定：

1. 香橙派侧 STM 串口设备名已确定为 UART0_M2：`/dev/ttyS0`。
2. STM 实际串口波特率已确定为 `576000`。
3. STM 侧 IMU 坐标轴是否已经对齐 ROS 坐标系：x 前、y 左、z 上。
4. 小车底盘实际参数：轮距、轮径、编码器分辨率是否已经在 STM 内部配置完成。
5. 雷达相对 `base_link` 的实际安装位姿。
6. 是否需要 `yaw_offset_rad = pi/2`，由后续实车观察 Cartographer 朝向后决定。
7. 硬件急停是否接入 STM，并通过 `STATUS.error_flags` 的 bit3 或新增错误位上报。
8. 小车外形半径和安全余量，用于设置 `robot_radius_m` 和 `safety_margin_m`。
9. 首批静态禁入区在 `map` 坐标系下的位置和尺寸。
