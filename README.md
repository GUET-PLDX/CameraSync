# CameraSync

MCU 侧相机同步模块。

## 作用

`CameraSync` 订阅一个带 Topic envelope timestamp 的 IMU topic，按 IMU 样本分频触发相机 GPIO，并发布同步结果 topic。

- IMU 驱动只负责正常发布传感器数据
- 相机默认触发频率由 `CameraSync.trigger_div` 控制，建议默认值保守一些，避免 MCU 单独运行时相机触发过快
- 上位机命令创建一个同步标记，并显式给出同步完成后的运行分频
- 同步命令会先制造一次可预测的异常图像间隔，完成同步后切到 `run_trigger_div`
- 完成分频后的第一个有效电平边沿是同步点
- 同步结果 topic 的 envelope timestamp 就是该同步点实际对齐的 IMU 传感器时间

## Topic

输入：

- `imu_topic_name`，默认 `bmi088_gyro`
- `camera_sync_command_topic_name`，默认 `camera_sync_command`

输出：

- `camera_sync_topic_name`，默认 `camera_sync_result`

命令消息：

```cpp
struct SyncCommand {
  uint8_t flags;
  uint8_t active_level;
  uint8_t seq;
  uint8_t sync_probe_div;
  uint8_t run_trigger_div;
};
```

- `flags`：`0` 表示普通同步命令；`RESET_TO_DEFAULT` 表示恢复构造参数里的默认触发分频。
- `active_level`：有效触发电平。`0` 表示低有效，非 `0` 表示高有效。
- `seq`：上位机每次同步自增的序号，MCU 在对应同步点原样回传；reset 命令不使用。
- `sync_probe_div`：同步探针间隔，单位是当前运行分频的倍数，必须大于 `0`。
- `run_trigger_div`：同步完成后的正式触发分频，单位是 IMU 样本数，必须大于 `0`。

`RESET_TO_DEFAULT` 命令立即取消待执行同步，切回默认触发分频，触发 GPIO 保持无效电平；
它不触发相机，也不发布同步结果。

输出消息：

```cpp
struct SyncEvent {
  uint8_t seq;
  uint8_t run_trigger_div;
  uint8_t active_level;
};
```

实际对齐的 IMU 时间使用 topic 消息自带的 timestamp。
普通相机触发不发布回执；只有命令对应的同步点发布 `SyncEvent`。

## 时序语义

命令回调不会直接写 GPIO，也不会直接发布同步结果。它只登记一个 pending 命令。

每条命令开始执行后，MCU 先把相机触发线保持到无效电平。等待
`当前运行分频 * sync_probe_div` 个 IMU 样本后输出一次有效触发脉冲，这个边沿就是
同步点，MCU 在同一个 IMU 回调中发布 `SyncEvent{seq, run_trigger_div}`。随后模块把
正常触发分频切到 `run_trigger_div`。

上位机应等待当前 `seq` 回执后再发送下一条同步命令；模块只保留一个 pending 命令，
用于保持 IMU 回调热路径简单且固定。

## 配置示例

```yaml
module: CameraSync
entry_header: Modules/CameraSync/CameraSync.hpp
constructor_args:
  - camera_pin_name: "CAMERA"
  - camera_sync_topic_name: "camera_sync_result"
  - imu_topic_name: "bmi088_gyro"
  - trigger_div: 50
  - camera_sync_command_topic_name: "camera_sync_command"
template_args: []
```

## 接入要求

- IMU topic 必须由传感器模块用真实采样时间戳发布
- AHRS topic 应继续使用对应 IMU 样本的 timestamp 发布，上位机侧按 timestamp 关联
- 本模块不依赖 `BMI088` 内部 callback，也不控制 IMU 发布频率
- Webots 仿真里上位机侧应通过 SharedTopic 收发
  `camera_sync_command` / `camera_sync_result`
