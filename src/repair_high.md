# STP23 高度跳变补偿方案

## 背景

当前任务是 Offboard 飞行追踪地面移动靶，并最终降落到靶子上。飞机在追踪前会定高到约 1.5m。STP23 激光测距朝下照射，原本测到的是飞机到地面的距离；当飞机飞到靶子上方时，激光会照到靶面，测距会突然变短。

实测靶子参数：

- 形状：圆形
- 直径：约 60cm
- 半径：约 30cm
- 高度：约 19cm

由于靶面高度、激光安装角、飞机姿态、靶子运动、定位误差都会带来偏差，补偿逻辑需要加入足够容差，不能只按严格的 0.19m 跳变判断。

## 当前 STP23 数据流

当前工程中 STP23 主要发布 `/stp23/range`：

- `src/stp23_ros2/config/stp23.yaml`
  - `publish_range: true`
  - `publish_point: false`
  - `publish_scan: false`
  - `range_topic: /stp23/range`

STP23 数据有两条可能影响 PX4 高度估计的路径：

1. `track2vision/src/laser_transfer.cpp`

   - 订阅 `/stp23/range`
   - 计算 `vehicle_z_enu = range + rangefinder_z_offset_m`
   - 发布到 `/mavros/odometry/out`
   - PX4 将其作为外部视觉/里程计高度观测
2. `mavros/mavros/launch/px4_config.yaml`

   - `distance_sensor` 插件订阅 `/stp23/range`
   - 通过 MAVLink `DISTANCE_SENSOR` 直接发给 PX4

因此补偿时不能只改 `/mavros/odometry/out` 的 z。如果 MAVROS `distance_sensor` 仍然把原始 `/stp23/range` 发给 PX4，PX4 仍可能收到靶面造成的 19cm 距离跳变。

## 核心思路

不要让 PX4 直接看到“激光打到靶面后的短距离”，而是让 PX4 看到连续的世界高度。

定义：

```text
raw_range          = STP23 原始测距，表示飞机到当前照射表面的距离
raw_z              = raw_range + rangefinder_z_offset_m
surface_bias       = 当前照射表面相对起飞地面的高度
corrected_z        = raw_z + surface_bias
height_to_surface  = raw_range
```

地面上方：

```text
raw_z        ~= 1.50
surface_bias = 0.00
corrected_z  = 1.50
```

靶子上方：

```text
raw_z        ~= 1.31
surface_bias = 0.19
corrected_z  = 1.50
```

降落到靶面：

```text
raw_z        从 1.31 平滑降到接近 0.19
surface_bias 保持 0.19
corrected_z  从 1.50 平滑降到接近 0.38
```

PX4 用 `corrected_z` 做高度估计，降落安全判断使用 `raw_range` 或 `height_to_surface`。

## 推荐参数

靶子实测高度约 0.19m，建议使用较宽容差：

```text
target_surface_height_m       = 0.19
target_surface_height_min_m   = 0.11
target_surface_height_max_m   = 0.30
target_surface_height_tol_m   = 0.08
```

圆形靶直径约 0.60m，考虑定位误差和靶子运动，空间门控不要只用 0.30m 半径：

```text
target_radius_m               = 0.30
target_xy_gate_radius_m       = 0.50
target_xy_strict_radius_m     = 0.38
target_xy_release_radius_m    = 0.75
```

跳变检测建议：

```text
jump_enter_min_m              = 0.10
jump_enter_max_m              = 0.32
jump_release_min_m            = 0.10
jump_release_max_m            = 0.32
jump_confirm_frames           = 3
jump_confirm_time_s           = 0.20
```

速度和滤波建议：

```text
raw_range_filter_alpha        = 0.35
corrected_z_filter_alpha      = 0.45
max_corrected_z_step_m        = 0.04
fake_vz_zero_time_s           = 0.25
```

这些参数的含义是：只要检测到约 10cm 到 32cm 的快速测距缩短，就认为可能照到了靶面。这个窗口覆盖了 19cm 靶高，同时给安装角、靶面边缘、运动和测距噪声留了余量。

## 状态机

建议在 `laser_transfer.cpp` 内维护一个补偿状态机。

```text
GROUND_NORMAL
  surface_bias = 0
  PX4 使用 raw_z

TARGET_CANDIDATE
  检测到疑似 -0.19m 跳变
  等待连续若干帧确认

TARGET_OVERHEAD
  surface_bias = estimated_target_height
  PX4 使用 corrected_z = raw_z + surface_bias
  飞机保持 1.5m 时不会因为激光打到靶面而上窜

LAND_ON_TARGET
  surface_bias 继续保持
  Offboard z setpoint 平滑下降
  使用 raw_range 判断离靶面高度和触地条件

TARGET_RELEASE
  非降落阶段检测到离开靶面
  surface_bias 平滑回到 0
```

进入靶面判断：

```text
dz = raw_z_filtered - last_raw_z_filtered

if target_tracking_enabled
   and inside_target_xy_gate
   and dz < -jump_enter_min_m
   and dz > -jump_enter_max_m:
       enter_candidate_count += 1

if enter_candidate_count >= jump_confirm_frames:
       estimated_target_height = clamp(-dz_accumulated,
                                       target_surface_height_min_m,
                                       target_surface_height_max_m)
       surface_bias = estimated_target_height
       state = TARGET_OVERHEAD
```

如果没有可靠的移动靶中心位置，也可以先用 Offboard mission phase 做门控：

```text
target_tracking_enabled =
    phase is MOVE_TO_TARGET
    or phase is TARGET_HOLD
    or phase is TARGET_DESCEND
    or phase is LAND_ON_TARGET
```

离开靶面判断：

```text
if state == TARGET_OVERHEAD
   and not landing_on_target
   and dz > jump_release_min_m
   and dz < jump_release_max_m:
       release_candidate_count += 1

if release_candidate_count >= jump_confirm_frames:
       surface_bias = 0
       state = GROUND_NORMAL
```

降落阶段不要释放 `surface_bias`。只要目标是降落到靶面，`surface_bias` 必须保持到降落完成或切到 AUTO.LAND/DISARM。

## 对 PX4 发布的数据

### `/mavros/odometry/out`

`laser_transfer.cpp` 里不要再直接发布 raw z：

```cpp
latest_range_z_enu_ = raw_z;
```

应改成：

```cpp
latest_raw_range_m_ = msg->range;
latest_raw_z_enu_ = msg->range + rangefinder_z_offset_m_;
latest_range_z_enu_ = corrected_z;
```

并且 `filtered_vz_enu_` 必须用 `corrected_z` 计算，不能用 `raw_z` 计算。检测到靶面跳变并更新 `surface_bias` 的那一刻，应将 `filtered_vz_enu_` 置 0 或短时间衰减，避免给 PX4 一个假的竖直速度。

### MAVROS `distance_sensor`

当前 `px4_config.yaml` 订阅的是原始 `/stp23/range`：

```yaml
/stp23/range:
  subscriber: true
```

建议二选一：

1. 禁用 MAVROS `distance_sensor` 这一路，只让 PX4 使用 `/mavros/odometry/out` 的补偿高度。
2. 新增补偿后的 Range 话题，例如 `/stp23/range_px4`，并让 MAVROS 订阅它。

如果选择方案 2，则补偿后的 Range 应发布：

```text
range_px4.range = corrected_z - rangefinder_z_offset_m
```

在靶子上方 1.5m 定高时：

```text
raw_range       ~= 1.31 - offset
range_px4.range ~= 1.50 - offset
```

这样 PX4 的两路高度输入保持一致，不会一路补偿、一路仍然跳变。

## Offboard 降落使用方式

Offboard setpoint 仍然发布世界坐标 z。假设起飞地面为 z=0，靶面高度约 0.19m：

```text
追踪高度: z_sp = 1.50
降落末端: z_sp ~= 0.19 + landing_clearance
```

建议：

```text
landing_clearance_m          = 0.03 到 0.08
landing_final_z_m            = 0.22 到 0.27
landing_descent_rate_mps     = 0.15 到 0.30
touchdown_raw_range_m        = 0.08 到 0.15
```

降落期间判断离靶面高度时，使用原始 `raw_range`，不要用 `corrected_z`：

```text
if raw_range < touchdown_raw_range_m
   and vertical_speed small
   and xy error small:
       request AUTO.LAND 或 disarm/land final procedure
```

## 调试话题

建议新增调试发布，方便 rosbag 和 rqt_plot：

```text
/track2vision/height/raw_range
/track2vision/height/raw_z
/track2vision/height/corrected_z
/track2vision/height/surface_bias
/track2vision/height/state
/track2vision/height/target_detected
/stp23/range_px4
```

重点观察：

```text
raw_z:       飞到靶面时允许出现约 -0.19m 跳变
surface_bias: 同时从 0 增加到约 0.19
corrected_z: 不应出现明显台阶
local_position.z: 不应被拉动约 0.19m
飞机实际高度: 不应在飞到靶面时上窜
```

## 真机验证步骤

1. 桨叶拆除或固定飞机，手持靶子从 STP23 下方移入移出。

   - 检查 `raw_z` 是否有约 0.19m 跳变。
   - 检查 `surface_bias` 是否正确补偿。
   - 检查 `corrected_z` 是否连续。
2. 低高度悬停测试，先不要降落。

   - 飞机定高 0.8m 到 1.0m。
   - 靶子慢速移动到飞机下方。
   - 观察飞机是否上窜。
3. 1.5m 追踪测试，不进入降落。

   - 验证追踪过程中 `corrected_z` 连续。
   - 验证离开靶面时非降落阶段能释放 `surface_bias`。
4. 进入降落测试。

   - 靶面上方保持 `surface_bias ~= 0.19`。
   - z setpoint 平滑下降到约 0.22m 到 0.27m。
   - 用 raw range 判断离靶面距离。
5. 最后再接入完整任务流程。

## 安全注意

- 第一次测试不要直接用 1.5m 完整降落，先低高度、低速度、短流程验证补偿。
- 如果 PX4 同时融合 `/mavros/odometry/out` 和 `DISTANCE_SENSOR`，必须保证两路都补偿，或禁用其中一路。
- `surface_bias` 在降落阶段不能因为检测到跳变恢复而清零。
- 如果 raw range 丢失、超范围或连续异常，应提高 z 协方差或暂时保持上一帧 corrected z，不要向 PX4 发布突变高度。
- 如果靶子边缘导致激光在地面和靶面之间快速抖动，应增加确认帧数、释放半径和状态保持时间。