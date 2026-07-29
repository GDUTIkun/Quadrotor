# ROS2 定位小车工程方案

## 1. 项目目标

本工程目标是实现一台基于 ROS2 的 2D 激光定位小车：

- 使用 2D 激光雷达 + Cartographer 完成建图与纯定位。
- STM 侧负责底盘控制，接收 ROS 侧速度指令，并完成轮速解算与速度闭环追踪。
- ROS 与 STM 通过串口通信，ROS 下发运动指令，STM 上传 IMU、轮式里程计和底盘状态。
- 建图完成后，ROS 侧实现轻量级定点控制逻辑，使小车能够根据目标点运动到指定位置。

首版不接入 Nav2，不实现全局路径规划和动态避障，优先保证建图、定位、底盘通信、定点控制闭环跑通。

## 2. 当前工程现状

当前工作区已有：

- `carto`

  - Cartographer 建图配置：`my_laser_with_imu.lua`
  - Cartographer 建图启动文件：`my_laser_with_imu.launch.py`
  - Cartographer 纯定位配置：`backpack_2d_localization.lua`
  - Cartographer 纯定位启动文件：`my_backpack_2d_localization.launch.py`
  - 地图目录：`carto/map/`
- `lsn10_lidar`

  - LSN10P 激光雷达 ROS2 驱动。
  - 整车 bringup 下当前雷达发布 `/car/scan`。
  - 整车 bringup 下当前雷达 frame 为 `car_laser`。
  - 当前雷达串口固定为 `/dev/wheeltec_lidar`，实机 udev 指向 UART4_M0 `/dev/ttyS4`。

Cartographer 当前关键设定：

- `map_frame = "car_carto_map"`
- `tracking_frame = "car_imu_link"`
- `published_frame = "car_base_link"`
- `odom_frame = "car_odom"`
- `provide_odom_frame = true`
- `use_odometry = false`
- `TRAJECTORY_BUILDER_2D.use_imu_data = true`
- IMU remap 到 `/car/imu/data_valid`
- Cartographer 节点和 topic 放在 `/car` namespace 下，避免与飞机侧 Cartographer 冲突。

因此首版 TF 链路应保持：

```text
car_carto_map -> car_odom -> car_base_link -> car_laser
                                      └── car_imu_link
```

其中：

- `car_carto_map -> car_odom -> car_base_link` 由 Cartographer 发布。
- `car_base_link -> car_laser` 由静态 TF 发布。
- `car_base_link -> car_imu_link` 由静态 TF 发布。
- 小车首版不由 STM/ROS 桥发布 odom；`car_carto_map -> car_odom -> car_base_link` 统一由 Cartographer 发布，避免 TF 或 odom 来源冲突。

## 2.1 当前 map 实现方式

当前建图方式是 Cartographer 2D 激光 + IMU 建图，数据链路如下：

```text
LSN10P lidar       -> /car/scan                     -> Cartographer
STM IMU over UART  -> stm_bridge                    -> /car/imu/data_valid -> Cartographer
static TF launch   -> car_base_link -> car_laser    -> Cartographer 查找雷达外参
static TF launch   -> car_base_link -> car_imu_link -> Cartographer 查找 IMU 外参
Cartographer       -> car_carto_map -> car_odom -> car_base_link -> RViz/后续路径规划与控制使用
Cartographer       -> /car/map                      -> occupancy grid 地图显示
car_localization   -> /car/pose                     -> 对外发布右/前/上小车坐标
```

具体实现：

- `lslidar_driver` 发布 `/car/scan`，`frame_id = car_laser`。
- `stm_bridge` 解析 STM 上传的 `0x81 IMU` 帧，发布 `/car/imu/data_valid`，`frame_id = car_imu_link`。
- `car_bringup/launch/mapping.launch.py` 使用 `tf2_ros/static_transform_publisher` 发布静态 TF：`car_base_link -> car_laser` 和 `car_base_link -> car_imu_link`。
- `carto/my_laser_with_imu.launch.py` 启动 `/car/cartographer_node`，将 Cartographer 的 `imu` remap 到 `/car/imu/data_valid`，`scan` remap 到 `/car/scan`。
- `carto/my_laser_with_imu.lua` 配置 `TRAJECTORY_BUILDER_2D.use_imu_data = true`，并设置 `num_laser_scans = 1`。
- `/car/cartographer_occupancy_grid_node` 根据 Cartographer 子图发布 `/car/map`，默认分辨率 `0.05 m`。

Cartographer frame 责任边界：

- `map_frame = car_carto_map`
- `odom_frame = car_odom`
- `tracking_frame = car_imu_link`
- `published_frame = car_base_link`
- `provide_odom_frame = true`
- `use_odometry = false`

Cartographer 要求 IMU frame 与 `tracking_frame` 共点；由于实车 IMU 相对底盘 `car_base_link` 有平移外参，因此 `tracking_frame` 使用 `car_imu_link`，`published_frame` 仍使用 `car_base_link` 对外提供底盘位姿。

因此当前 map 的位姿来源不是 STM 轮式里程计，而是 Cartographer 使用 `/car/scan` 和 IMU 做 2D scan matching 后发布的 TF。STM 的职责是提供 IMU 和执行速度指令；小车首版不通过 STM 发布 `car_odom -> car_base_link`。

`yaw + pi/2` 当前不参与建图链路。它只是参考原飞机 `track2vision` 工程时发现的 MAVROS 外部视觉坐标适配逻辑，小车是否需要该 yaw offset 待实车观察后决定。

## 3. 工程模块划分

建议使用这些 ROS2 包：

```text
car_ws/
  carto/
  lsn10_lidar/
  stm_bridge/
  point_controller/
  car_localization/
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
- 轮式里程计上行协议保留为调试/后续融合接口，首版 ROS 节点默认不发布 `/car/odom/wheel`。
- 实现通信 watchdog，ROS 指令超时后自动下发零速。
- 注意：`/cmd_vel` 当前仍是全局 topic。Cartographer 相关 topic/node/frame 已隔离；若飞机侧也会发布 `/cmd_vel`，控制链路还需要进一步切到 `/car/cmd_vel`。

ROS 接口：

| 方向 | Topic                            | 类型                                    | 说明                                 |
| ---- | -------------------------------- | --------------------------------------- | ------------------------------------ |
| 订阅 | `/cmd_vel`                     | `geometry_msgs/msg/Twist`             | ROS 到 STM 的速度指令来源            |
| 发布 | `/car/imu/data_valid`          | `sensor_msgs/msg/Imu`                 | Cartographer 使用的 IMU              |
| 发布 | `/car/stm/status`              | `diagnostic_msgs/msg/DiagnosticArray` | 电压、电流、状态机、错误码、通信统计 |
| 预留 | `/car/odom/wheel`              | `nav_msgs/msg/Odometry`               | 默认关闭，仅调试或后续融合时启用     |

默认参数：

| 参数                   | 默认值                       | 说明                                                     |
| ---------------------- | ---------------------------- | -------------------------------------------------------- |
| `port`               | `/dev/wheeltec_controller` | STM 串口设备                                             |
| `baudrate`           | `576000`                   | 串口波特率                                               |
| `cmd_rate_hz`        | `50.0`                     | 速度指令下发频率                                         |
| `cmd_timeout_s`      | `0.3`                      | `/cmd_vel` 超时时间                                    |
| `base_frame_id`      | `car_base_link`            | IMU 与底盘 frame                                         |
| `odom_frame_id`      | `car_odom`                 | 轮式里程计预留 frame                                     |
| `imu_frame_id`       | `car_imu_link`             | IMU 消息 frame                                           |
| `imu_topic`          | `/car/imu/data_valid`      | IMU 发布 topic                                           |
| `status_topic`       | `/car/stm/status`          | STM 诊断状态 topic                                       |
| `wheel_odom_topic`   | `/car/odom/wheel`          | 轮式里程计预留 topic                                     |
| `publish_wheel_odom` | `false`                    | 是否发布 `/car/odom/wheel`，首版关闭                    |
| `publish_odom_tf`    | `false`                    | 必须保持关闭，`car_odom -> car_base_link` 由 Cartographer 发布 |

### 3.2 `point_controller`

职责：

- 接收地图坐标系下的目标点。
- 查询 `car_carto_map -> car_base_link` TF。
- 计算当前位置到目标点的距离误差和角度误差。
- 输出 `/cmd_vel`，由 `stm_bridge` 下发给 STM。

ROS 接口：

| 方向 | Topic                        | 类型                              | 说明                   |
| ---- | ---------------------------- | --------------------------------- | ---------------------- |
| 订阅 | `/goal_pose`               | `geometry_msgs/msg/PoseStamped` | 地图坐标系目标点       |
| 发布 | `/cmd_vel`                 | `geometry_msgs/msg/Twist`       | 输出给底盘的速度指令   |
| 发布 | `/point_controller/status` | `std_msgs/msg/String`           | 当前控制阶段和误差状态 |

控制策略：

1. 若当前位置到目标点距离大于阈值：

   - 先计算目标方向角。
   - 若航向误差较大，只原地转向。
   - 航向误差进入阈值后，边前进边小角速度修正。
2. 若位置误差进入阈值：

   - 停止线速度。
   - 调整末端 yaw。
3. 若位置和末端 yaw 均进入阈值：

   - 发布零速。
   - 标记目标到达。

默认参数：

| 参数                      | 默认值        | 说明                  |
| ------------------------- | ------------- | --------------------- |
| `global_frame_id`       | `car_carto_map` | 目标点坐标系          |
| `base_frame_id`         | `car_base_link` | 小车本体坐标系        |
| `control_rate_hz`       | `20.0`      | 控制频率              |
| `xy_tolerance_m`        | `0.05`      | 到点距离阈值          |
| `yaw_tolerance_rad`     | `0.087`     | 到点角度阈值，约 5 度 |
| `heading_tolerance_rad` | `0.15`      | 前进前的航向阈值      |
| `target_speed_m_s`      | `0.05`      | 测试路径跟踪匀速目标，实车可按需下调 |
| `v_max_m_s`             | `0.05`      | 最大线速度            |
| `w_max_rad_s`           | `0.5`       | 最大角速度            |
| `v_min_m_s`             | `0.0`       | 最小有效线速度        |
| `w_min_rad_s`           | `0.08`      | 最小有效角速度        |

### 3.3 `car_bringup`

职责：

- 统一启动整车相关节点。
- 提供建图 launch。
- 提供纯定位 + 定点控制 launch。
- 发布 `car_base_link -> car_laser` 和 `car_base_link -> car_imu_link` 静态 TF。
- 启动 `car_localization` 对外发布小车定位坐标。
- 避免每次手动启动多个 launch。

建议提供两个启动文件：

```text
car_bringup/launch/mapping.launch.py
car_bringup/launch/localization_control.launch.py
```

`mapping.launch.py` 启动：

- LSN10 雷达驱动。
- STM 串口桥。
- `car_base_link -> car_laser` 和 `car_base_link -> car_imu_link` 静态 TF。
- Cartographer 建图。
- `car_localization` 小车坐标发布节点。

`localization_control.launch.py` 启动：

- LSN10 雷达驱动。
- STM 串口桥。
- `car_base_link -> car_laser` 和 `car_base_link -> car_imu_link` 静态 TF。
- Cartographer 纯定位。
- `car_localization` 小车坐标发布节点。
- `path_planner` 无障碍几何路径节点，当前支持目标点直连、测试直线和测试圆弧。
- `path_controller` 路径跟踪控制节点。

### 3.4 `car_localization`

职责：

- 查询 Cartographer 发布的 `car_carto_map -> car_base_link` TF。
- 不发布 TF，不改变 Cartographer、路径规划器和控制器使用的 ROS 标准坐标。
- 将 Cartographer 位姿额外转换成对外坐标 topic，坐标约定为 `+x` 向右、`+y` 向前、`+z` 向上。

ROS 接口：

| 方向 | Topic | 类型 | 说明 |
| --- | --- | --- | --- |
| 发布 | `/car/pose` | `geometry_msgs/msg/PoseStamped` | 小车在 `car_map` 坐标系下的位置和 yaw |
| 发布 | `/car/odom/carto` | `nav_msgs/msg/Odometry` | 小车在 `car_map` 坐标系下的位置、yaw，以及由位姿差分得到并低通滤波后的速度 |

坐标转换：

```text
car_pose.x = -cartographer_pose.y
car_pose.y =  cartographer_pose.x
car_pose.z =  cartographer_pose.z
car_pose.yaw = cartographer_yaw + pi/2 + yaw_offset_rad
```

速度估计：

```text
vx_map = diff(car_pose.x) / dt
vy_map = diff(car_pose.y) / dt
wz     = normalize(diff(car_pose.yaw)) / dt

再按当前 yaw 转到车体坐标，并使用一阶低通滤波：
filtered = filtered + alpha * (raw - filtered)
alpha    = dt / (velocity_filter_tau_s + dt)
```

若 TF 时间戳跳变、`dt <= 0` 或 `dt > max_velocity_dt_s`，速度滤波器重置为 0，避免 Cartographer 偶发跳变直接进入控制环。`/car/pose` 保持兼容发布；需要速度项的节点优先订阅 `/car/odom/carto`。

默认参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `global_frame_id` | `car_carto_map` | Cartographer 全局 frame |
| `base_frame_id` | `car_base_link` | 小车本体 frame |
| `output_frame_id` | `car_map` | `/car/pose` 的 frame，轴定义为右/前/上 |
| `pose_topic` | `/car/pose` | 对外发布的小车坐标 topic |
| `odom_topic` | `/car/odom/carto` | 对外发布的 Cartographer 差分里程计 topic |
| `odom_child_frame_id` | `car_base_link` | `/car/odom/carto` 的 child frame |
| `publish_rate_hz` | `20.0` | 坐标发布频率 |
| `yaw_offset_rad` | `0.0` | 实车确认后的额外 yaw 修正，默认不开 |
| `publish_odom` | `true` | 是否发布带速度项的 `/car/odom/carto` |
| `pose_filter_tau_s` | `0.10` | 输出位姿一阶低通时间常数，0 表示不滤波 |
| `velocity_filter_tau_s` | `0.25` | 差分速度一阶低通时间常数 |
| `max_velocity_dt_s` | `0.5` | 超过该采样间隔时重置速度滤波 |

### 3.5 `track_runner`

职责：

- 比赛固定航线跟踪节点，使用 `/car/pose` 闭环计算速度指令。
- 按操场形赛道生成路径点，并根据当前位置和前瞻目标点实时输出 `/cmd_vel`。
- 通过 topic 在终端或飞机任务逻辑中控制速度、圈数、启动、暂停和停止。
- 当前 `/car/pose` 由 `car_localization` 从 Cartographer TF 转换得到，坐标约定为 `+x` 向右、`+y` 向前、`+z` 向上。

赛道几何：

```text
A=(0,0), B=(0,1.5), C=(1.5,1.5), D=(1.5,0)
半径 r=0.75 m
顺序 A -> B -> C -> D -> A
```

以上是局部赛道坐标。默认 `use_start_pose_as_origin=true`，收到 `start` 时会把当前 `/car/pose` 位置锁定为 A 点原点，因此不要求地图中的 A 点绝对坐标正好是 `(0,0)`。小车初始由人工摆在 A 点，车头朝 A->B。默认顺时针执行：

| 段 | 几何 | 路径方向 |
| --- | --- | --- |
| A->B | 左侧直线 `1.5 m` | `+y` |
| B->C | 上半圆，半径 `0.75 m` | 向右半圆 |
| C->D | 右侧直线 `1.5 m` | `-y` |
| D->A | 下半圆，半径 `0.75 m` | 向左半圆 |

一圈长度：

```text
2 * 1.5 + 2 * pi * 0.75 ~= 7.71 m
```

`0.02 m/s` 跑一圈约 `386 s`。若只做快速底盘验证，建议先用 `0.05-0.08 m/s` 抬轮或低速空场测试。

控制方式：

- 节点订阅 `/car/pose` 获取当前 `x/y/yaw`。
- 节点订阅 `/car/odom/carto` 获取当前角速度 `wz`，用于角速度内环；若该 topic 超时，则自动退回只有角度外环的控制。
- 赛道离散为固定路径点，默认点距 `0.02 m`。
- 控制循环中查找当前最近路径进度，并选取前方 `lookahead_distance_m` 的目标点。
- 根据当前车头方向和目标点方向计算 `yaw_error`：

  ```text
  target_yaw = atan2(target.y - pose.y, target.x - pose.x)
  yaw_error  = normalize(target_yaw - pose.yaw)
  ```

- 角度外环输出期望角速度。操场半圆段加入几何前馈角速度，直线段前馈为 0。前馈系数按线速度线性变化：

  ```text
  k_w_ff     = clamp(5.0 - 100.0 * speed, 0.0, 10.0)
  yaw_rate_ff = k_w_ff * speed / radius * direction
  target_w    = clamp(yaw_rate_ff + k_w * yaw_error, -w_max_rad_s, w_max_rad_s)
  ```

- 当前默认关系：`speed=0.01 -> k_w_ff=4.0`，`speed=0.02 -> k_w_ff=3.0`，`speed=0.03 -> k_w_ff=2.0`。

- 角速度内环用 `/car/odom/carto.twist.twist.angular.z` 做反馈：

  ```text
  w_error = target_w - measured_w
  w_i     = clamp(w_i + w_error * dt, -w_error_integral_max, w_error_integral_max)
  w_d     = low_pass((w_error - previous_w_error) / dt)
  cmd_w   = clamp(k_w_rate * w_error + k_i_rate * w_i + k_d_rate * w_d, -w_max_rad_s, w_max_rad_s)
  ```

- 输出 `linear.x=speed`，`angular.z=cmd_w`。
- 当累计路径进度达到目标圈数后自动停车。

角度追踪链路：

```text
/car/pose
  -> 当前 x/y/yaw
  -> 当前路径进度 progress
  -> 前瞻目标点 target
  -> yaw_error
  -> target_w = yaw_rate_ff + k_w * yaw_error
/car/odom/carto
  -> measured_w
target_w + measured_w
  -> w_error
  -> w_i = integral(w_error)
  -> w_d = derivative(w_error)
  -> angular.z = k_w_rate * w_error + k_i_rate * w_i + k_d_rate * w_d
  -> stm_bridge
  -> STM 左右轮速度环
```

因此当前 ROS 侧是串级闭环：外环用路径前瞻点生成航向角误差，角度环给出期望角速度；内环用 Cartographer 转换后 odom 的差分滤波角速度做反馈，由角速度误差直接计算最终下发的 `angular.z`。STM 侧仍负责左右轮速度环，ROS 侧角速度内环用于补偿底盘实际转向跟随误差。

ROS 接口：

| 方向 | Topic | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/car/pose` | `geometry_msgs/msg/PoseStamped` | 闭环跟踪使用的当前位姿，坐标为右/前/上 |
| 订阅 | `/car/odom/carto` | `nav_msgs/msg/Odometry` | 差分滤波后的当前角速度，用于角速度内环 |
| 订阅 | `/car/track_runner/command` | `std_msgs/msg/String` | `start`、`pause`、`resume`、`stop`、`reset` |
| 订阅 | `/car/track_runner/speed` | `std_msgs/msg/Float64` | 运行速度，单位 `m/s` |
| 订阅 | `/car/track_runner/laps` | `std_msgs/msg/Int32` | 目标圈数，最小为 1 |
| 发布 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 下发到底盘的闭环速度指令 |
| 发布 | `/car/track_runner/status` | `std_msgs/msg/String` | 当前状态、圈数、路径进度、目标点、yaw 误差、速度、剩余距离 |

默认参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `straight_length_m` | `1.5` | 直线段长度 |
| `radius_m` | `0.75` | 半圆半径 |
| `path_spacing_m` | `0.02` | 航线路径点间距 |
| `default_speed_m_s` | `0.03` | 默认线速度 |
| `default_laps` | `1` | 默认圈数 |
| `lookahead_distance_m` | `0.25` | 前瞻目标点距离 |
| `waypoint_tolerance_m` | `0.05` | 路径进度更新容差 |
| `goal_tolerance_m` | `0.05` | 终点停车容差 |
| `k_w` | `0.6` | 航向误差到角速度的比例系数 |
| `k_w_ff_speed_intercept` | `5.0` | 线速度到前馈系数的截距 |
| `k_w_ff_speed_slope` | `-100.0` | 线速度到前馈系数的斜率 |
| `k_w_ff_min` | `0.0` | 自动前馈系数下限 |
| `k_w_ff_max` | `10.0` | 自动前馈系数上限 |
| `k_w_rate` | `0.32` | 角速度误差到角速度指令的比例系数 |
| `k_i_rate` | `0.9` | 角速度误差积分到角速度指令的比例系数 |
| `k_d_rate` | `0.0` | 角速度误差导数到角速度指令的比例系数，默认关闭微分 |
| `w_error_integral_max` | `0.5` | 角速度误差积分限幅，防止积分饱和 |
| `w_error_derivative_filter_tau_s` | `0.05` | 角速度误差导数一阶低通时间常数 |
| `w_max_rad_s` | `0.8` | 最大角速度 |
| `pose_timeout_s` | `0.5` | `/car/pose` 超时时间，超时停车 |
| `angular_velocity_timeout_s` | `0.35` | `/car/odom/carto` 角速度超时时间，超时退回角度外环 |
| `use_angular_velocity_feedback` | `true` | 是否启用角速度内环 |
| `use_start_pose_as_origin` | `true` | `start` 时用当前 `/car/pose` 作为 A 点 |
| `control_rate_hz` | `50.0` | `/cmd_vel` 发布频率 |
| `pose_topic` | `/car/pose` | 当前位姿 topic |
| `odom_topic` | `/car/odom/carto` | 当前差分 odom topic |
| `cmd_vel_topic` | `/cmd_vel` | 输出给 STM 的速度 topic |

启动方式：

```bash
cd ~/flight_ws/car
source install/setup.bash
ros2 launch track_runner track_runner.launch.py
```

终端控制：

```bash
ros2 topic pub --once /car/track_runner/speed std_msgs/msg/Float64 "{data: 0.02}"
ros2 topic pub --once /car/track_runner/laps std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /car/track_runner/command std_msgs/msg/String "{data: start}"
ros2 topic pub --once /car/track_runner/command std_msgs/msg/String "{data: pause}"
ros2 topic pub --once /car/track_runner/command std_msgs/msg/String "{data: resume}"
ros2 topic pub --once /car/track_runner/command std_msgs/msg/String "{data: stop}"
```

使用注意：

- 航线跟踪时必须确保 `/cmd_vel` 只有一个 publisher。
- 不要同时启动 `path_controller_node`，也不要同时手动 `ros2 topic pub /cmd_vel`。
- `pause`、`stop`、`reset` 都会持续发布零速，避免 STM watchdog 触发时出现忽动忽停。
- 如果 `/car/pose` 超过 `pose_timeout_s` 未更新，节点会发布零速并在状态中显示 `reason=pose_unavailable`。
- 若直线段左右摆动，优先增大 `lookahead_distance_m` 或减小 `k_w`。
- 若圆弧段转向跟不上，优先增大 `k_w` 或 `w_max_rad_s`。
- 若目标点过近导致轨迹像折线，避免把 `lookahead_distance_m` 调到接近 `path_spacing_m`，建议从 `0.25 m` 起调。

### 3.5.1 `angular_rate_tuner`

职责：

- 单独调试角速度内环，不跑路径、不使用角度外环。
- 订阅 `/car/odom/carto` 获取 `measured_w`。
- 发布 `/cmd_vel`，命令形式为：

```text
error = target_w - measured_w
w_i   = clamp(w_i + error * dt, -w_error_integral_max, w_error_integral_max)
w_d   = low_pass((error - previous_error) / dt)
cmd_w = clamp(k_w_rate * error + k_i_rate * w_i + k_d_rate * w_d, -w_max_rad_s, w_max_rad_s)
linear.x = linear_speed_m_s
```

启动：

```bash
cd ~/flight_ws/car
source install/setup.bash
ros2 launch track_runner angular_rate_tuner.launch.py \
  linear_speed_m_s:=0.02 \
  target_w_rad_s:=0.2 \
  k_w_rate:=0.32 \
  k_i_rate:=0.9 \
  k_d_rate:=0.0 \
  w_error_integral_max:=0.5 \
  w_error_derivative_filter_tau_s:=0.05 \
  w_max_rad_s:=0.3
```

运行前确认不要同时启动 `track_runner_node`、`path_controller_node` 或手动 `/cmd_vel` 发布器。启动后节点默认只发布零速，收到 `start` 后才开始调参：

```bash
ros2 topic echo /car/angular_rate_tuner/status
ros2 topic pub --once /car/angular_rate_tuner/command std_msgs/msg/String "{data: start}"
```

运行中调参：

```bash
ros2 topic pub --once /car/angular_rate_tuner/k_w_rate std_msgs/msg/Float64 "{data: 1.5}"
ros2 topic pub --once /car/angular_rate_tuner/k_i_rate std_msgs/msg/Float64 "{data: 0.05}"
ros2 topic pub --once /car/angular_rate_tuner/k_d_rate std_msgs/msg/Float64 "{data: 0.01}"
ros2 topic pub --once /car/angular_rate_tuner/target_w std_msgs/msg/Float64 "{data: 0.2}"
ros2 topic pub --once /car/angular_rate_tuner/speed std_msgs/msg/Float64 "{data: 0.02}"
ros2 topic pub --once /car/angular_rate_tuner/command std_msgs/msg/String "{data: reset_integral}"
ros2 topic pub --once /car/angular_rate_tuner/command std_msgs/msg/String "{data: reset_pid}"
ros2 topic pub --once /car/angular_rate_tuner/command std_msgs/msg/String "{data: reverse}"
ros2 topic pub --once /car/angular_rate_tuner/command std_msgs/msg/String "{data: stop}"
```

状态中重点观察：

```text
k_w_rate=...
k_i_rate=...
k_d_rate=...
target_w=...
measured_w=...
w_error=...
w_error_integral=...
w_error_derivative=...
cmd_w=...
reason=running
```

调参判断：

- `measured_w` 长时间跟不上 `target_w`，且 `cmd_w` 没顶到 `w_max_rad_s`：增大 `k_w_rate`。
- `measured_w` 超过 `target_w` 或来回振荡：减小 `k_w_rate`。
- `k_w_rate` 已经比较稳但存在长期静差：小幅增大 `k_i_rate`，例如 `0.02 -> 0.05 -> 0.1`。
- 加积分后出现慢速来回摆动或停止后拖尾明显：减小 `k_i_rate` 或降低 `w_error_integral_max`，并发送 `reset_integral`。
- 角速度响应太钝、超调后回落慢：可小幅增加 `k_d_rate`，例如 `0.005 -> 0.01 -> 0.02`。
- 加微分后 `cmd_w` 高频抖动：减小 `k_d_rate`，或增大 `w_error_derivative_filter_tau_s`，例如 `0.05 -> 0.10`。
- `cmd_w` 经常等于 `±w_max_rad_s`：不要继续加 `k_w_rate`，先确认 `w_max_rad_s`、底盘能力和速度滤波。
- `/car/odom/carto` 抖动明显：增大 `pose_velocity_filter_tau_s`，例如 `0.25 -> 0.35`。

调好后，把最终 `k_w_rate`、`k_i_rate` 和 `k_d_rate` 写回 `track_runner.launch.py` 或启动 `track_runner` 时传参。

### 3.5.2 `circle_angle_tuner`

职责：

- 单独调试角度外环，不跑完整操场路径。
- 半径默认 `0.75 m`，收到 `start` 时以当前 `/car/pose` 为圆上起点生成圆心。
- target 点由内部连续参考圆进度生成，不直接用 noisy pose 投影点逐帧重算，因此 `target_x/target_y` 是光滑圆。
- 角度外环按圆轨迹前瞻点计算 `target_yaw/yaw_error/target_w`，并加入圆周几何角速度前馈 `yaw_rate_ff = k_w_ff(speed) * direction * speed / radius`。
- 角速度内环沿用已调好的 PID 参数，发布 `/cmd_vel`。

启动：

```bash
cd ~/flight_ws/car
source install/setup.bash
ros2 launch track_runner circle_angle_tuner.launch.py \
  radius_m:=0.75 \
  linear_speed_m_s:=0.03 \
  lookahead_distance_m:=0.25 \
  k_w:=0.6 \
  k_w_ff_speed_intercept:=5.0 \
  k_w_ff_speed_slope:=-100.0 \
  k_w_rate:=0.32 \
  k_i_rate:=0.9 \
  k_d_rate:=0.0 \
  w_max_rad_s:=0.8
```

开始、停止和反向：

```bash
ros2 topic pub --once /car/circle_angle_tuner/command std_msgs/msg/String "{data: start}"
ros2 topic pub --once /car/circle_angle_tuner/command std_msgs/msg/String "{data: reverse}"
ros2 topic pub --once /car/circle_angle_tuner/command std_msgs/msg/String "{data: stop}"
```

运行中调角度环：

```bash
ros2 topic pub --once /car/circle_angle_tuner/k_w std_msgs/msg/Float64 "{data: 0.5}"
ros2 topic pub --once /car/circle_angle_tuner/lookahead std_msgs/msg/Float64 "{data: 0.30}"
ros2 topic pub --once /car/circle_angle_tuner/speed std_msgs/msg/Float64 "{data: 0.02}"
```

记录角度和角速度：

```bash
python3 analyse/record_circle_angle.py --send-start --stop-on-exit
```

调参判断：

- `yaw_error` 长时间偏同一侧，圆越跑越向外/向内：增大 `k_w`。
- `yaw_error` 正负频繁切换，轨迹左右摆：减小 `k_w` 或增大 `lookahead_distance_m`。
- 弯道明显切内侧、目标点太急：增大 `lookahead_distance_m`。
- 弯道跟不上、圆半径越跑越大：减小 `lookahead_distance_m` 或增大 `k_w/w_max_rad_s`。
- `target_w` 经常顶到 `w_max_rad_s`：角度环已经要满角速度，先不要继续加 `k_w`。

## 4. STM 串口通信接口冻结版 v1.1

本节作为 ROS 侧与 STM 侧的对接依据。STM 侧先按这里的帧格式发 `STATUS` 和 `IMU`，ROS 侧能解析并发布 topic 后，再联调 `CMD_VEL` 下发和轮式里程计。

v1.1 相对 v1.0 的变更：

- `IMU` 角速度字段由 `int16 mdps` 改为 `int16 cdeg/s`，避免 `int16 mdps` 只能表示约 `±0.572 rad/s` 导致角速度饱和。
- `IMU` payload 长度和字段数量不变，仍为 22 字节；ROS 侧只需要同步修改角速度单位换算。

### 4.1 串口物理层

| 项           | 约定                         |
| ------------ | ---------------------------- |
| 波特率       | `576000`                   |
| 数据位       | 8                            |
| 校验位       | none                         |
| 停止位       | 1                            |
| 流控         | none                         |
| 字节序       | little-endian                |
| 有符号数     | 二进制补码                   |
| ROS 默认设备 | `/dev/wheeltec_controller` |

如果 STM 侧现有波特率不是 `576000`，优先把 ROS 参数改成 STM 当前值，不强制 STM 先改固件。

### 4.2 通用帧格式

```text
header[2] + msg_id[1] + seq[1] + len[1] + payload[len] + crc16[2]
```

| 字段        | 长度 | 说明                         |
| ----------- | ---- | ---------------------------- |
| `header`  | 2    | 固定`0xAA 0x55`            |
| `msg_id`  | 1    | 消息 ID                      |
| `seq`     | 1    | 帧序号，0-255 循环           |
| `len`     | 1    | payload 字节数               |
| `payload` | N    | 数据区，最大 64 字节         |
| `crc16`   | 2    | CRC-16/IBM，小端，低字节在前 |

解析规则：

- 收到非 `0xAA 0x55` 开头的数据时，逐字节丢弃直到重新找到帧头。
- `len > 64` 视为异常帧，丢弃当前帧头并重新同步。
- CRC 错误时丢弃当前帧头并重新同步。
- 允许半包和粘包，ROS 侧解析器必须能缓存未完整帧。

CRC 规则：

| 项             | 值                               |
| -------------- | -------------------------------- |
| 名称           | CRC-16/IBM，也称 CRC-16/ARC      |
| 多项式         | `0x8005`                       |
| 反向实现多项式 | `0xA001`                       |
| 初值           | `0x0000`                       |
| xorout         | `0x0000`                       |
| 输入反射       | true                             |
| 输出反射       | true                             |
| 覆盖范围       | `msg_id + seq + len + payload` |
| 不覆盖         | `header` 和 `crc16` 本身     |

校验向量：

```text
crc16_ibm("123456789") = 0xBB3D
```

### 4.3 消息总表

| msg_id   | 方向       | 名称           | payload 长度 | 频率建议             |
| -------- | ---------- | -------------- | ------------ | -------------------- |
| `0x01` | ROS -> STM | `CMD_VEL`    | 5            | 50 Hz                |
| `0x81` | STM -> ROS | `IMU`        | 22           | 50-100 Hz            |
| `0x82` | STM -> ROS | `WHEEL_ODOM` | 24           | 预留，默认不要求发送 |
| `0x83` | STM -> ROS | `STATUS`     | 11           | 1-10 Hz              |

### 4.3.1 数值量程核查

通信字段按当前小车需求核查如下：

| 消息 | 字段 | 编码 | 可表示范围 | 结论 |
| ---- | ---- | ---- | ---------- | ---- |
| `CMD_VEL` | `v_mm_s` | `int16 mm/s` | 约 `±32.767 m/s` | 足够 |
| `CMD_VEL` | `w_mrad_s` | `int16 mrad/s` | 约 `±32.767 rad/s` | 足够 |
| `IMU` | `ax_mg/ay_mg/az_mg` | `int16 mg` | 约 `±32.767 g` | 足够 |
| `IMU` | `gx_cdeg_s/gy_cdeg_s/gz_cdeg_s` | `int16 0.01 deg/s` | 约 `±327.67 deg/s`，即 `±5.72 rad/s` | 足够首版小车使用 |
| `IMU` | `yaw_cdeg/pitch_cdeg/roll_cdeg` | `int16 0.01 deg` | 约 `±327.67 deg` | 角度发送前应归一化到 `[-180, 180] deg` |
| `WHEEL_ODOM` | `x_mm/y_mm` | `int32 mm` | 约 `±2147 km` | 足够 |
| `WHEEL_ODOM` | `yaw_mrad` | `int32 mrad` | 约 `±2.147e6 rad` | 足够 |
| `WHEEL_ODOM` | `vx_mm_s/left_mm_s/right_mm_s` | `int16 mm/s` | 约 `±32.767 m/s` | 足够 |
| `WHEEL_ODOM` | `wz_mrad_s` | `int16 mrad/s` | 约 `±32.767 rad/s` | 足够 |
| `STATUS` | `voltage_mv` | `uint16 mV` | `0-65.535 V` | 足够 |
| `STATUS` | `current_ma` | `int16 mA` | 约 `±32.767 A` | 首版够用；大电流电机可改 `int32 mA` |
| 通用 | `stamp_ms` | `uint32 ms` | 约 `49.7 天` 回绕 | ROS 侧按无符号差值处理 |

首轮通信测试最低要求：

- STM 至少发送 `0x83 STATUS`，用于确认串口、帧头、长度和 CRC。
- 然后发送 `0x81 IMU`，用于启动 Cartographer 需要的 `/car/imu/data_valid`。
- `0x82 WHEEL_ODOM` 是预留调试接口，首版不要求 STM 发送，ROS 默认不发布 `/car/odom/wheel`。

### 4.4 ROS -> STM：`0x01 CMD_VEL`

ROS 以固定频率下发底盘速度指令。STM 收到后继续负责轮速解算和速度闭环。

payload，长度 5：

```text
int16 v_mm_s
int16 w_mrad_s
uint8 enable
```

| 字段         | 单位   | 正方向     | 说明              |
| ------------ | ------ | ---------- | ----------------- |
| `v_mm_s`   | mm/s   | 前进为正   | 车体 x 方向线速度 |
| `w_mrad_s` | mrad/s | 逆时针为正 | 车体 z 轴角速度   |
| `enable`   | bool   | 1 使能     | `0` 表示停车    |

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
int16 gx_cdeg_s
int16 gy_cdeg_s
int16 gz_cdeg_s
int16 yaw_cdeg
int16 pitch_cdeg
int16 roll_cdeg
uint32 stamp_ms
```

| 字段           | 单位     | 说明                              |
| -------------- | -------- | --------------------------------- |
| `ax_mg`      | mg       | x 轴线加速度                      |
| `ay_mg`      | mg       | y 轴线加速度                      |
| `az_mg`      | mg       | z 轴线加速度，水平静止约`+1000` |
| `gx_cdeg_s`  | 0.01 deg/s | x 轴角速度                      |
| `gy_cdeg_s`  | 0.01 deg/s | y 轴角速度                      |
| `gz_cdeg_s`  | 0.01 deg/s | z 轴角速度                      |
| `yaw_cdeg`   | 0.01 deg | yaw，绕 z 轴                      |
| `pitch_cdeg` | 0.01 deg | pitch，绕 y 轴                    |
| `roll_cdeg`  | 0.01 deg | roll，绕 x 轴                     |
| `stamp_ms`   | ms       | STM 上电后的毫秒时间戳            |

ROS 发布：

- Topic：`/car/imu/data_valid`
- 类型：`sensor_msgs/msg/Imu`
- `header.frame_id = car_imu_link`
- ROS 首版使用接收时刻作为 `header.stamp`，`stamp_ms` 先用于诊断和时序检查。

单位转换：

```text
linear_acceleration = mg * 9.80665 / 1000
angular_velocity = cdeg_s * pi / (180 * 100)
euler_angle = cdeg * pi / (180 * 100)
```

量程约定：

- `int16 gx_cdeg_s/gy_cdeg_s/gz_cdeg_s` 可表示约 `±327.67 deg/s`，即约 `±5.72 rad/s`。
- 不使用 `int16 mdps` 表示 IMU 角速度；`int16 mdps` 只能表示约 `±32.767 deg/s`，会在 ROS 侧表现为约 `±0.572 rad/s` 饱和。
- 若后续实测角速度仍超过 `±327.67 deg/s`，应升级 IMU payload 字段为 `int32 mdps` 或 `int32 cdeg_s`，不能只在 ROS 侧放大。

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

小车首版不使用 STM 里程计参与建图或定位，也不让串口桥发布 odom。该消息仅作为调试或后续融合预留；未启用 `publish_wheel_odom` 时，ROS 侧即使收到该帧也不发布 `/car/odom/wheel`。

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

| 字段           | 单位   | 正方向     | 说明                   |
| -------------- | ------ | ---------- | ---------------------- |
| `x_mm`       | mm     | 前进为正   | STM 积分得到的 x       |
| `y_mm`       | mm     | 左侧为正   | STM 积分得到的 y       |
| `yaw_mrad`   | mrad   | 逆时针为正 | STM 积分得到的 yaw     |
| `vx_mm_s`    | mm/s   | 前进为正   | 当前车体线速度         |
| `wz_mrad_s`  | mrad/s | 逆时针为正 | 当前车体角速度         |
| `left_mm_s`  | mm/s   | 前进为正   | 左轮线速度             |
| `right_mm_s` | mm/s   | 前进为正   | 右轮线速度             |
| `stamp_ms`   | ms     | 单调递增   | STM 上电后的毫秒时间戳 |

ROS 发布：

- Topic：`/car/odom/wheel`
- 类型：`nav_msgs/msg/Odometry`
- `header.frame_id = car_odom`
- `child_frame_id = car_base_link`
- 默认不发布。
- 即使后续打开 `/car/odom/wheel` 调试，也不发布 `car_odom -> car_base_link` TF。
- 小车项目的 `car_carto_map -> car_odom -> car_base_link` 统一由 Cartographer 发布。

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

| 字段            | 单位    | 说明                         |
| --------------- | ------- | ---------------------------- |
| `voltage_mv`  | mV      | 电池或母线电压               |
| `current_ma`  | mA      | 当前电流；没有电流采样时填 0 |
| `state`       | enum    | STM 当前状态                 |
| `error_flags` | bitmask | 错误标志                     |
| `stamp_ms`    | ms      | STM 上电后的毫秒时间戳       |

ROS 发布：

- Topic：`/car/stm/status`
- 类型：`diagnostic_msgs/msg/DiagnosticArray`

状态枚举：

| state | 含义    |
| ----- | ------- |
| 0     | INIT    |
| 1     | READY   |
| 2     | RUNNING |
| 3     | ESTOP   |
| 4     | ERROR   |

错误位：

| bit | 含义          |
| --- | ------------- |
| 0   | CMD_TIMEOUT   |
| 1   | IMU_ERROR     |
| 2   | ENCODER_ERROR |
| 3   | MOTOR_ERROR   |
| 4   | LOW_VOLTAGE   |
| 5   | CRC_ERROR     |

样例帧：

```text
# seq=0, voltage=12000mV, current=0mA, state=READY, error=0, stamp=1000ms
AA 55 83 00 0B E0 2E 00 00 01 00 00 E8 03 00 00 85 A4
```

### 4.8 STM 侧首轮测试发送顺序

为了先验证 ROS 串口桥，STM 可按以下顺序发送：

1. 1 Hz 发送 `STATUS`，确认 ROS 能收到 `/car/stm/status`。
2. 50 Hz 发送水平静止 `IMU`，确认 ROS 能收到 `/car/imu/data_valid`。
3. ROS 发布 `/cmd_vel` 后，STM 打印收到的 `CMD_VEL` 中的 `v_mm_s`、`w_mrad_s` 和 `enable`。
4. 停止 ROS `/cmd_vel` 输入后，确认 STM 在 watchdog 时间内停车。
5. 如需调试轮式里程计，再临时打开 `publish_wheel_odom` 并发送 `WHEEL_ODOM`。

## 5. TF 与坐标系设计

### 5.1 坐标系

| frame         | 说明                              |
| ------------- | --------------------------------- |
| `car_carto_map` | Cartographer 地图坐标系 |
| `car_odom` | Cartographer 输出的局部连续坐标系 |
| `car_base_link` | 小车本体坐标系 |
| `car_laser` | 激光雷达坐标系 |
| `car_imu_link` | IMU 坐标系 |
| `car_map` | `/car/pose` 对外坐标发布 frame，`+x` 向右、`+y` 向前、`+z` 向上 |

### 5.2 静态 TF

必须发布：

```text
car_base_link -> car_laser
car_base_link -> car_imu_link
```

建议做成 launch 参数：

| 参数            | 默认值   | 说明                    |
| --------------- | -------- | ----------------------- |
| `laser_x`     | `0.06842` | 雷达相对 `car_base_link` 的 x |
| `laser_y`     | `0.0`    | 雷达相对 `car_base_link` 的 y |
| `laser_z`     | `0.01246` | 雷达相对 `car_base_link` 的 z |
| `laser_roll`  | `0.0`  | 雷达 roll               |
| `laser_pitch` | `0.0`  | 雷达 pitch              |
| `laser_yaw`   | `-1.5708`  | 雷达 yaw，单位 rad；当前实测雷达坐标需相对 `car_base_link` 旋转 -90 度 |
| `imu_x`       | `0.0`     | IMU 相对 `car_base_link` 的 x |
| `imu_y`       | `-0.0146` | IMU 相对 `car_base_link` 的 y |
| `imu_z`       | `0.075`   | IMU 相对 `car_base_link` 的 z |
| `imu_roll`    | `0.0`     | IMU roll |
| `imu_pitch`   | `0.0`     | IMU pitch |
| `imu_yaw`     | `0.0`     | IMU yaw |

实际安装后需要按机械结构修改。

### 5.3 坐标修正待测项

参考原飞机 `track2vision` 工程时发现，飞机侧曾在从 Cartographer 姿态转换到 MAVROS 外部视觉时使用过 `yaw + pi/2`。该修正属于飞机/MAVROS 坐标适配逻辑，小车首版不默认启用。

小车实测时按以下顺序确认：

1. 不加 yaw offset，观察 Cartographer 中 `car_base_link` 朝向是否与车头一致。
2. 给小车原地逆时针转动，确认 `car_base_link` yaw 是否按 ROS 右手系增大。
3. 若发现车头方向与地图显示固定相差 90 度，再考虑设置 `yaw_offset_rad = pi/2`。
4. 该修正只能加在明确需要坐标适配的位置，不能同时在 IMU、TF、控制器多处重复修正。

## 6. 建图流程

### 6.1 启动前检查

检查串口：

```bash
ls -l /dev/wheeltec_lidar
ls -l /dev/wheeltec_controller
```

检查雷达：

```bash
ros2 topic hz /car/scan
ros2 topic echo /car/scan --once
```

检查 IMU：

```bash
ros2 topic hz /car/imu/data_valid
ros2 topic echo /car/imu/data_valid --once
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo car_base_link car_laser
ros2 run tf2_ros tf2_echo car_base_link car_imu_link
ros2 run tf2_ros tf2_echo car_carto_map car_base_link
```

### 6.2 建图启动

推荐统一启动：

```bash
ros2 launch car_bringup mapping.launch.py
```

启动内容：

- 雷达驱动。
- STM 串口桥。
- 静态 TF。
- Cartographer 建图。

### 6.3 保存地图

建图完成后保存 `.pbstream`：

```bash
ros2 service call /car/finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /car/write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/t/car_ws/carto/map/my_map.pbstream', include_unfinished_submaps: true}"
```

地图统一保存到：

```text
carto/map/
```

## 7. 纯定位与定点控制流程

### 7.1 启动纯定位

```bash
ros2 launch car_bringup localization_control.launch.py load_state_filename:=/home/t/car_ws/carto/map/my_map.pbstream
```

启动内容：

- 雷达驱动。
- STM 串口桥。
- 静态 TF。
- Cartographer 纯定位。
- 定点控制节点。

### 7.2 发送目标点

目标点使用 `/goal_pose`：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'car_carto_map'},
  pose: {
    position: {x: 1.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

控制节点读取 `car_carto_map -> car_base_link`，输出 `/cmd_vel`。

STM 只需要继续执行速度指令，不需要理解目标点。

## 8. 实施步骤

### 阶段 1：文档与接口冻结

- 完成 `program.md`。
- 确认 STM 串口协议字段、单位、坐标系。
- 确认雷达和 STM 的 udev 名称。
- 确认 `car_base_link -> car_laser` 的实际安装参数。

### 阶段 2：实现 `stm_bridge`

- 新建 ROS2 C++ 包 `stm_bridge`。
- 实现 CRC-16/IBM。
- 实现二进制帧打包和增量解析。
- 实现 `/cmd_vel` 到 `CMD_VEL` 串口帧。
- 实现 `IMU`、`WHEEL_ODOM`、`STATUS` 三类上行帧解析。
- 发布 `/car/imu/data_valid` 和 `/car/stm/status`。
- `WHEEL_ODOM` 解析逻辑作为预留接口；首版默认不发布 `/car/odom/wheel`。
- 加入 ROS 侧 `/cmd_vel` watchdog。

### 阶段 3：实现 `point_controller`

- 新建 ROS2 C++ 包 `point_controller`。
- 订阅 `/goal_pose`。
- 查询 `car_carto_map -> car_base_link`。
- 实现三阶段控制：
  - 对准目标方向。
  - 前进并修正航向。
  - 到点后调整末端 yaw。
- 输出 `/cmd_vel`。
- 到点后自动发布零速。

### 阶段 4：实现 `car_bringup`

- 新建 ROS2 包 `car_bringup`。
- 增加 `mapping.launch.py`。
- 增加 `localization_control.launch.py`。
- 在 launch 中发布 `car_base_link -> car_laser` 静态 TF。
- 将雷达串口参数固定为 `/dev/wheeltec_lidar`。
- 将 STM 串口参数固定为 `/dev/wheeltec_controller`。

### 阶段 5：联调

联调顺序：

1. 只测串口协议。
2. 测 STM 上传 IMU。
3. 测雷达 `/car/scan`。
4. 测静态 TF。
5. 启动 Cartographer 建图。
6. 保存 `.pbstream`。
7. 启动纯定位。
8. 抬轮测试定点控制输出方向。
9. 低速实车测试单点。
10. 低速实车测试多点。

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
ros2 topic hz /car/imu/data_valid
ros2 topic echo /car/stm/status
```

验证项：

- `/cmd_vel` 正常下发。
- `/cmd_vel` 停止后 300 ms 内下发零速。
- IMU frame 为 `car_imu_link`。
- `/car/stm/status` 能显示错误码和通信统计。
- 默认不存在 `/car/odom/wheel`，小车 odom/TF 由 Cartographer 发布。

### 9.3 Cartographer 测试

验证项：

- `/car/scan` 正常。
- `/car/imu/data_valid` 正常。
- `car_base_link -> car_laser` 存在。
- `car_base_link -> car_imu_link` 存在。
- Cartographer 不再因为缺少 IMU 或 TF 卡住。
- 建图过程中 `car_carto_map -> car_odom -> car_base_link` 连续发布。
- 保存后的 `.pbstream` 可被纯定位 launch 加载。

### 9.4 定点控制测试

测试点：

- 原地旋转目标。
- 前方 1 m 目标。
- 左前方目标。
- 右前方目标。
- 连续 3 个目标点。

验收标准：

- 到点后自动停车。
- 目标方向错误时优先转向，不盲目前进。
- 通信断开或 `/cmd_vel` 超时时底盘停车。
- 低速测试无明显震荡。

## 10. 风险与注意事项

- 雷达串口和 STM 串口必须用 udev 固定名称，不能依赖 `/dev/ttyACM0`。
- STM 与 ROS 的坐标系必须完全一致，否则 Cartographer 和定点控制都会异常。
- Cartographer 当前使用 IMU，IMU 的 `frame_id`、方向、单位必须正确。
- 小车首版不发布 STM 轮式 odom；`car_carto_map -> car_odom -> car_base_link` 由 Cartographer 统一发布。
- 原飞机工程中的 `yaw + pi/2` 是待实测项，不默认启用，避免未验证的坐标补偿污染 IMU、TF 或控制逻辑。
- 定点控制没有避障能力，只适合在已知、低速、安全环境中测试。
- 实车第一次测试必须限速，并保证急停可用。

## 11. 实机部署前待确认

通信接口按第 4 节冻结为 v1.1。以下问题不再影响协议字段，只影响香橙派实机参数和标定：

1. 香橙派侧 STM 当前使用 UART0_M2 `/dev/ttyS0`，波特率 `576000`；后续可再固定为 `/dev/wheeltec_controller`。
2. STM 侧 IMU 角速度已按 v1.1 使用 `int16 cdeg/s`，ROS 侧按 `cdeg_s * pi / (180 * 100)` 转为 rad/s。
3. STM 侧 IMU 坐标轴已按 ROS 坐标系初步验证：x 前、y 左、z 上；左转 `angular_velocity.z` 为正，右转为负。
4. 小车底盘实际参数：轮距、轮径、编码器分辨率是否已经在 STM 内部配置完成。
5. 雷达相对 `car_base_link` 的当前实测安装位姿：`x=0.06842, y=0.0, z=0.01246, yaw=-1.5708 rad`。
6. 是否需要额外 `yaw_offset_rad`：当前不需要；已通过 `car_base_link -> car_laser` 静态 TF 修正雷达 yaw。
7. 硬件急停是否接入 STM，并通过 `STATUS.error_flags` 的 bit3 或新增错误位上报。
