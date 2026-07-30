# Offboard Task1 状态话题接口

本文档定义 `offboard_task1_node` 给地面站显示任务状态使用的 ROS 2 话题接口。

## 话题

| 项目 | 定义 |
| --- | --- |
| Topic | `/offboard_task1/status` |
| Message | `std_msgs/msg/String` |
| QoS | `reliable + transient_local`，深度 1 |
| 发布方 | `offboard_task1_node` |
| 订阅方 | 地面站 |

相关 ROS 参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `status_topic` | `/offboard_task1/status` | 状态话题名称 |
| `status_publish_period` | `1.0` | 当前状态重复发布周期，单位秒，最小值 0.1 |

地面站可以通过以下命令调试：

```bash
ros2 topic echo /offboard_task1/status
```

## 消息格式

`std_msgs/msg/String.data` 填写任务状态码。状态码使用大写英文，地面站负责映射成中文显示。

示例：

```yaml
data: "FOLLOW"
```

## 状态码

| 状态码 | 中文显示 | 含义 |
| --- | --- | --- |
| `TAKEOFF` | 起飞 | 飞机正在等待定位、发送起飞 setpoint、切 OFFBOARD/解锁，或爬升到任务高度 |
| `FOLLOW` | 伴飞 | 飞机已到达任务高度，正在根据小车位置进行伴飞 |
| `DROP` | 抛投 | 已触发抛投流程，包含下降到抛投高度、执行舵机释放、释放后爬升 |
| `LAND` | 降落 | 抛投后正在返航、下降到降落请求高度，或已经请求降落模式 |
| `COMPLETE` | 完成 | 飞机已降落并解除 armed |
| `FAULT` | 异常 | 节点检测到任务异常，例如小车坐标跳变导致伴飞锁定 |

## 内部 Phase 映射建议

`offboard_task1_node` 内部可以保留更细的 `Phase` 状态机，发布给地面站时统一映射为上面的业务状态。

| 内部 Phase | 对外状态 |
| --- | --- |
| `WAIT_FOR_POSE` | `TAKEOFF` |
| `STREAM_SETPOINTS` | `TAKEOFF` |
| `ARM_AND_OFFBOARD` | `TAKEOFF` |
| `TAKEOFF` | `TAKEOFF` |
| `FOLLOW` | `FOLLOW` |
| `DESCEND_FOR_DROP` | `DROP` |
| `RELEASE_PAYLOAD` | `DROP` |
| `ASCEND_AFTER_DROP` | `DROP` |
| `RETURN_HOME` | `LAND` |
| `DESCEND_FOR_LAND` | `LAND` |
| `LAND` | `LAND` |
| `COMPLETE` | `COMPLETE` |

如果 `vehicle_follow_locked_ == true`，无论当前内部 Phase 是什么，对外都应发布 `FAULT`。

## 发布策略

推荐同时满足两种发布条件：

1. 状态变化时立即发布一次。
2. 当前状态每 1 秒重复发布一次。

这样地面站既能快速刷新，也能根据最后接收时间判断链路是否正常。

## 地面站显示建议

地面站只依赖 `TAKEOFF/FOLLOW/DROP/LAND/COMPLETE/FAULT` 这些稳定状态码，不直接依赖节点内部 `Phase` 名称。

建议显示映射：

| 接收状态码 | 显示文本 |
| --- | --- |
| `TAKEOFF` | 起飞 |
| `FOLLOW` | 伴飞 |
| `DROP` | 抛投 |
| `LAND` | 降落 |
| `COMPLETE` | 完成 |
| `FAULT` | 异常 |

如果地面站超过 3 秒没有收到 `/offboard_task1/status`，建议显示 `通信中断` 或 `状态超时`。
