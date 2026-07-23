#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 由带时间戳 IMU 消息驱动的 MCU 侧相机触发同步模块
constructor_args:
  - camera_pin_name: "CAMERA"
  - camera_sync_topic_name: "camera_sync_result"
  - imu_topic_name: "bmi088_gyro"
  - trigger_div: 50
  - camera_sync_command_topic_name: "camera_sync_command"
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstdint>

#include "app_framework.hpp"
#include "gpio.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "transform.hpp"

/**
 * @brief 相机同步触发模块。
 * @details 模块只使用 IMU topic 的消息时间戳。
 *          正常状态每 trigger_div 个 IMU 样本输出一次触发脉冲。同步命令会先
 *          制造一次可预测的探针图像间隔，再切换同步完成后的运行分频。SyncEvent
 *          回传 seq 和实际采用的运行分频，同步时间由 Topic 消息时间戳表示。
 */
class CameraSync : public LibXR::Application {
 public:
  using ImuSample = Eigen::Matrix<float, 3, 1>;

  /**
   * @brief 同步命令标志位。
   *
   * 当前只定义恢复默认分频命令；普通同步命令必须保持 flags 为 0。
   */
  enum SyncCommandFlags : uint8_t {
    RESET_TO_DEFAULT = 1U << 0,  ///< 取消同步状态并恢复构造参数 trigger_div。
  };

  /**
   * @brief 上位机同步命令。
   * @details flags 为 0 时执行普通同步；RESET_TO_DEFAULT 只恢复默认触发分频，
   *          不触发相机也不发布 SyncEvent。sync_probe_div 是当前运行分频的探针
   *          倍率，run_trigger_div 是同步完成后的正常触发分频，单位是 IMU
   * 样本数。
   */
  struct SyncCommand {
    uint8_t flags = 0;  ///< 0 为普通同步；见 SyncCommandFlags。
    uint8_t active_level =
        1;            ///< 相机触发有效电平，0 为低有效，非 0 为高有效。
    uint8_t seq = 0;  ///< 普通同步序号，reset 命令不使用。
    uint8_t sync_probe_div = 3;  ///< 探针间隔倍率，普通同步时必须非 0。
    uint8_t run_trigger_div =
        50;  ///< 同步完成后的运行分频，普通同步时必须非 0。
  };

  /**
   * @brief 同步点回执。
   * @details 实际同步时间使用 topic 消息自带 timestamp。
   */
  struct SyncEvent {
    uint8_t seq = 0;               ///< 对应 SyncCommand::seq。
    uint8_t run_trigger_div = 50;  ///< MCU 在同步点之后采用的运行分频。
    uint8_t active_level = 1;      ///< MCU 在同步点采用的触发有效电平。
  };

  static_assert(sizeof(SyncCommand) == 5);
  static_assert(sizeof(SyncEvent) == 3);

  /**
   * @brief 构造 CameraSync 模块。
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param camera_pin_name 相机触发 GPIO 名称
   * @param camera_sync_topic_name 同步结果 Topic 名称
   * @param imu_topic_name 作为同步基准的 IMU Topic 名称
   * @param trigger_div 默认每多少个 IMU 样本触发一次相机，必须在 1 到 255 之间
   * @param camera_sync_command_topic_name 上位机同步命令 Topic 名称
   */
  CameraSync(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
             const char* camera_pin_name, const char* camera_sync_topic_name,
             const char* imu_topic_name, uint32_t trigger_div,
             const char* camera_sync_command_topic_name)
      : camera_sync_pin_(
            *hw.template FindOrExit<LibXR::GPIO>({camera_pin_name})),
        imu_topic_(LibXR::Topic::CreateTopic<ImuSample>(imu_topic_name)),
        command_topic_(LibXR::Topic::CreateTopic<SyncCommand>(
            camera_sync_command_topic_name)),
        camera_sync_topic_(
            LibXR::Topic::CreateTopic<SyncEvent>(camera_sync_topic_name)),
        default_trigger_div_(ClampDiv(trigger_div)),
        trigger_div_(ClampDiv(trigger_div)) {
    ASSERT(trigger_div != 0);

    camera_sync_pin_.SetConfig(
        {.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
         .pull = LibXR::GPIO::Pull::NONE});
    camera_sync_pin_.Write(false);

    imu_callback_ = LibXR::Topic::Callback::Create(
        [](bool in_isr, CameraSync* self, LibXR::MicrosecondTimestamp timestamp,
           const ImuSample&) { self->OnImuMessage(in_isr, timestamp); },
        this);
    imu_topic_.RegisterCallback(imu_callback_);

    command_callback_ = LibXR::Topic::Callback::Create(
        [](bool, CameraSync* self, LibXR::MicrosecondTimestamp,
           LibXR::RawData& data) { self->OnCommandData(data); },
        this);
    command_topic_.RegisterCallback(command_callback_);

    app.Register(*this);
  }

  /**
   * @brief CameraSync 当前不输出周期监控。
   */
  void OnMonitor() override {}

 private:
  static constexpr uint8_t default_run_trigger_div =
      50;  ///< 未构造时的保底默认分频。
  static constexpr uint8_t min_pulse_hold_samples =
      1;  ///< 触发脉冲至少保持的 IMU 样本数。
  static constexpr uint8_t known_command_flags =
      RESET_TO_DEFAULT;  ///< 当前支持的 flags 位。

  /**
   * @brief MCU 侧触发状态。
   */
  enum class SyncState : uint8_t {
    NORMAL = 0,           ///< 按 trigger_div_ 周期正常触发。
    WAIT_PROBE_EDGE = 1,  ///< 等待本次同步命令制造的探针触发边沿。
  };

  /**
   * @brief 将外部传入的分频限制到协议可表示范围。
   */
  static uint8_t ClampDiv(uint32_t div) {
    if (div == 0) {
      return 1;
    }
    if (div > UINT8_MAX) {
      return UINT8_MAX;
    }
    return static_cast<uint8_t>(div);
  }

  /**
   * @brief 从 RawData 中解析同步命令。
   */
  void OnCommandData(LibXR::RawData& data) {
    if (data.addr_ == nullptr || data.size_ != sizeof(SyncCommand)) {
      return;
    }

    SyncCommand command;
    LibXR::Memory::FastCopy(&command, data.addr_, sizeof(command));
    OnCommand(command);
  }

  /**
   * @brief 处理上位机命令。
   *
   * reset 命令立即生效；普通同步命令只登记 pending，真正 GPIO 操作仍在 IMU 回调
   * 中执行。
   */
  void OnCommand(const SyncCommand& command) {
    if ((command.flags & static_cast<uint8_t>(~known_command_flags)) != 0) {
      return;
    }

    active_level_ = command.active_level == 0 ? 0U : 1U;
    if ((command.flags & RESET_TO_DEFAULT) != 0) {
      ResetToDefault();
      return;
    }

    // 上位机应等待回执后再发下一条命令；模块只保留最新一条待执行命令。
    if (command.sync_probe_div == 0 || command.run_trigger_div == 0) {
      return;
    }

    pending_command_.sync_probe_div = command.sync_probe_div;
    pending_command_.run_trigger_div = command.run_trigger_div;
    pending_command_.active_level = command.active_level == 0 ? 0U : 1U;
    pending_command_.seq = command.seq;
    pending_command_ready_ = true;
  }

  /**
   * @brief 恢复构造参数给出的默认触发分频。
   *
   * 该命令用于处理 Host 重启但 MCU 未重启的情况。它不制造探针边沿，不发布
   * SyncEvent，只保证后续重新同步从默认触发状态开始。
   */
  void ResetToDefault() {
    pending_command_ready_ = false;
    sync_state_ = SyncState::NORMAL;
    trigger_div_ = default_trigger_div_;
    active_probe_interval_samples_ = default_trigger_div_;
    pending_run_div_ = default_trigger_div_;
    samples_since_trigger_ = 0;
    pulse_hold_samples_ = 0;
    active_seq_ = 0;
    camera_sync_pin_.Write(!active_level_);
  }

  /**
   * @brief 若当前处于正常触发态，则开始执行一条 pending 同步命令。
   */
  void StartPendingCommandIfIdle() {
    if (sync_state_ != SyncState::NORMAL || !pending_command_ready_) {
      return;
    }

    pending_command_ready_ = false;
    active_probe_interval_samples_ =
        static_cast<uint16_t>(trigger_div_) * pending_command_.sync_probe_div;
    pending_run_div_ = pending_command_.run_trigger_div;
    active_level_ = pending_command_.active_level;
    active_seq_ = pending_command_.seq;
    sync_state_ = SyncState::WAIT_PROBE_EDGE;
    camera_sync_pin_.Write(!active_level_);
    pulse_hold_samples_ = 0;
  }

  void OnImuMessage(bool in_isr, LibXR::MicrosecondTimestamp imu_timestamp) {
    if (pulse_hold_samples_ > 0) {
      pulse_hold_samples_--;
      if (pulse_hold_samples_ == 0) {
        camera_sync_pin_.Write(!active_level_);
      }
    }

    StartPendingCommandIfIdle();
    samples_since_trigger_++;

    const uint16_t current_interval = sync_state_ == SyncState::WAIT_PROBE_EDGE
                                          ? active_probe_interval_samples_
                                          : trigger_div_;
    if (samples_since_trigger_ < current_interval) {
      return;
    }

    samples_since_trigger_ = 0;
    const bool publish_sync_event = sync_state_ == SyncState::WAIT_PROBE_EDGE;
    TriggerCamera(in_isr, imu_timestamp, publish_sync_event);
    if (publish_sync_event) {
      trigger_div_ = pending_run_div_;
      sync_state_ = SyncState::NORMAL;
      active_probe_interval_samples_ = trigger_div_;
      pending_run_div_ = trigger_div_;
      active_seq_ = 0;
    }
  }

  void TriggerCamera(bool in_isr, LibXR::MicrosecondTimestamp imu_timestamp,
                     bool publish_sync_event) {
    camera_sync_pin_.Write(active_level_);
    pulse_hold_samples_ = min_pulse_hold_samples;

    if (!publish_sync_event) {
      return;
    }

    SyncEvent event;
    event.seq = active_seq_;
    event.run_trigger_div = pending_run_div_;
    event.active_level = active_level_;
    camera_sync_topic_.PublishFromCallback(event, imu_timestamp, in_isr);
  }

  LibXR::GPIO& camera_sync_pin_;

  LibXR::Topic imu_topic_;
  LibXR::Topic command_topic_;
  LibXR::Topic camera_sync_topic_;
  LibXR::Topic::Callback imu_callback_;
  LibXR::Topic::Callback command_callback_;

  uint8_t default_trigger_div_ = default_run_trigger_div;
  uint8_t trigger_div_ = default_run_trigger_div;
  uint16_t samples_since_trigger_ = 0;
  uint8_t pulse_hold_samples_ = 0;

  bool pending_command_ready_ = false;
  SyncCommand pending_command_;

  SyncState sync_state_ = SyncState::NORMAL;
  uint16_t active_probe_interval_samples_ = default_run_trigger_div;
  uint8_t pending_run_div_ = default_run_trigger_div;
  uint8_t active_level_ = 1;
  uint8_t active_seq_ = 0;
};
