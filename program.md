# ROS2 地面站总体开发与逐步调试流程

## 1. 总体目标

将当前工程逐步改造成 ROS2 Humble / Ubuntu 22.04 工程，主语言使用 C++ `rclcpp`。

原流程中的“地面站 MCU”职责改由香橙派 3B 上的 ROS2 节点承担。串口屏通过 UART2 与香橙派通信，主机先用于写代码和仿真，最终部署到香橙派 3B。

硬件默认规划：

- 设备：香橙派 3B
- 串口：UART2
- 默认设备名：`/dev/ttyS2`
- 默认波特率：`115200`
- 串口格式：`8N1`
- 接线：串口屏 TX/RX 与香橙派 UART2 RX/TX 交叉连接，GND 共地
- 用户给定引脚：RX、TX 对应 `GPIO0`、`D1`

开发原则：

```text
每次只实现一个最小功能
  ↓
单独编译
  ↓
单独测试
  ↓
确认通过
  ↓
再进入下一步
```

当前阶段只实现最小串口通信验证，不直接实现禁飞区、航线规划、任务管理或动物统计。

---

## 2. 最终 ROS2 工程分工

最终工程计划拆成以下 ROS2 节点。

| 节点 | 作用 |
| --- | --- |
| `hmi_serial_node` | 串口屏 USART_HMI 指令收发、触摸事件解析 |
| `mission_manager_node` | 保存禁飞区、任务状态、识别记录、动物统计 |
| `route_planner_node` | 根据 9x7 地图和禁飞区生成巡查航线 |
| `drone_comm_node` | 后续接入真实无人机通信 |
| `sim_drone_node` | 主机调试阶段模拟无人机识别数据 |

当前已实现的最小节点：

| 节点 | 作用 |
| --- | --- |
| `hmi_touch_verify_node` | 只验证串口屏触摸返回帧 |

---

## 3. 最终 ROS2 接口规划

### 3.1 Topics

| Topic | Type | 说明 |
| --- | --- | --- |
| `/hmi/touch_event` | `HmiTouchEvent` | 串口屏触摸事件 |
| `/hmi/command` | `std_msgs/msg/String` | 发送给串口屏的 USART_HMI 指令 |
| `/mission/blocked_grid` | `BlockedGrid` | 63 个禁飞区状态 |
| `/mission/route` | `Route` | 规划后的巡查航线 |
| `/mission/state` | `MissionState` | 当前任务状态 |
| `/drone/start` | `std_msgs/msg/Bool` | 启动巡查命令 |
| `/drone/animal_detection` | `AnimalDetection` | 无人机识别结果 |
| `/drone/mission_complete` | `std_msgs/msg/Bool` | 无人机任务完成事件 |

### 3.2 Services

| Service | Type | 说明 |
| --- | --- | --- |
| `/mission/clear_blocked` | `std_srvs/srv/Trigger` | 清空禁飞区 |
| `/mission/plan_route` | `std_srvs/srv/Trigger` | 根据当前禁飞区生成航线 |
| `/mission/start` | `std_srvs/srv/Trigger` | 开始巡查 |
| `/mission/show_result` | `std_srvs/srv/Trigger` | 显示本次结果 |

注意：这些接口是最终规划。当前最小通信验证阶段暂不创建这些消息和服务，避免一开始工程过大。

---

## 4. 阶段 1：最小串口通信验证

### 4.1 目标

只验证串口屏触摸控件能否通过 UART2 返回正确字节。

目标返回帧：

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

### 4.2 当前工程内容

当前 ROS2 包：

```text
src/quadrotor_ground_station
```

当前节点：

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

### 4.3 编译

在 ROS2 Humble / Ubuntu 22.04 环境中执行：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 4.4 运行

默认使用 `/dev/ttyS2` 和 `115200`：

```bash
ros2 launch quadrotor_ground_station hmi_touch_verify.launch.py
```

如果串口设备名不同：

```bash
ros2 launch quadrotor_ground_station hmi_touch_verify.launch.py serial_port:=/dev/ttyS2 baud_rate:=115200
```

### 4.5 通过标准

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

---

## 5. 后续阶段开发顺序

### 阶段 2：串口屏基础指令发送

目标：

- 实现 ROS2 节点向串口屏发送 USART_HMI 指令。
- 每条指令末尾追加 `FF FF FF`。
- 验证能修改 `page0.t_state.txt`。

最小验证：

```text
page0.t_state.txt="通信正常"
FF FF FF
```

通过标准：

- 屏幕文字能从“串口测试”变为“通信正常”。
- 多次重启节点后都能稳定更新。

### 阶段 3：触摸事件抽象成 ROS2 Topic

目标：

- 将触摸返回帧解析成 `/hmi/touch_event`。
- 后续业务节点不再直接处理串口字节。

计划消息字段：

```text
uint8 page_id
uint8 component_id
uint8 event
```

通过标准：

- 点击控件后能在 ROS2 topic 中看到正确 `page_id`、`component_id`、`event`。

### 阶段 4：禁飞区设置

目标：

- 实现 63 个方格按钮 `g0` 到 `g62`。
- 触摸事件映射到 `blocked[63]`。
- 支持清空禁飞区。

通过标准：

- 63 个方格均可点击。
- 同一方格可选中和取消。
- 清空按钮能同步清空屏幕和内部数组。

### 阶段 5：航线显示

目标：

- 先显示固定测试线。
- 再显示固定测试航线。
- 最后接入真实航线规划结果。

通过标准：

- 航线显示在 9x7 地图中心点之间。
- 航线不经过禁飞区。
- 航线覆盖所有非禁飞方格。

### 阶段 6：一键启动巡查

目标：

- 未生成航线时拒绝启动。
- 已生成航线时发布启动命令，并切换到巡查页面。

通过标准：

- 未生成航线时屏幕提示“请先生成航线”。
- 已生成航线时只发送一次启动命令。

### 阶段 7：动物识别数据显示

目标：

- 先使用模拟节点产生识别结果。
- 再接入真实无人机上传数据。

最小数据：

```text
grid_index
animal_type
count
```

动物编号：

| 编号 | 动物 |
| ---: | --- |
| 0 | 象 |
| 1 | 虎 |
| 2 | 狼 |
| 3 | 猴 |
| 4 | 孔雀 |

通过标准：

- 方格代码正确。
- 五种动物中文无乱码。
- 数量显示正确。

### 阶段 8：记录保存与结果查看

目标：

- 保存本次任务识别记录。
- 累加五种动物总数。
- 任务完成后进入结果页。
- 支持上一条、下一条查看。

最小数据结构：

```c
typedef struct
{
    uint8_t grid_index;
    uint8_t animal_type;
    uint8_t count;
} AnimalRecord;
```

通过标准：

- 记录顺序正确。
- 五种动物总数正确。
- 边界处上一条、下一条不越界。

---

## 6. 最终完整运行流程

```text
系统上电
  ↓
进入 page0
  ↓
用户点击方格设置禁飞区
  ↓
用户点击“生成航线”
  ↓
ROS2 规划并在 9x7 地图上画出航线
  ↓
用户点击“开始巡查”
  ↓
ROS2 向无人机发送启动命令
  ↓
进入 page1
  ↓
无人机上传动物识别结果
  ↓
ROS2 保存记录、累加数量、实时更新 page1
  ↓
无人机发送任务完成
  ↓
进入 page2
  ↓
显示五种动物总数
  ↓
通过上一条和下一条查看本次所有识别记录
```

---

## 7. 保存范围

最小版本只保存当前上电期间最近一次任务数据。

暂不实现：

- 断电保存
- 多次历史任务管理
- Flash / 数据库持久化

如果后续赛题明确要求断电后仍能调出记录，再单独增加持久化阶段。
