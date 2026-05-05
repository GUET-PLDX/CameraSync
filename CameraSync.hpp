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
   * @brief 上位机同步命令。
   * @details sync_probe_div 是当前运行分频的探针倍率，run_trigger_div 是同步完成后
   *          的正常触发分频，单位是 IMU 样本数。active_level 是相机有效触发电平，
   *          seq 由上位机自增，MCU 在对应同步点回传同一个 seq。
   */
  struct SyncCommand {
    uint8_t version = 1;
    uint8_t seq = 0;
    uint8_t flags = 0;
    uint8_t sync_probe_div = 3;
    uint8_t run_trigger_div = 50;
    uint8_t active_level = 1;
  };

  /**
   * @brief 同步点回执。
   * @details 实际同步时间使用 topic 消息自带 timestamp，不再复制到 payload 里。
   */
  struct SyncEvent {
    uint8_t version = 1;
    uint8_t seq = 0;
    uint8_t run_trigger_div = 50;
    uint8_t active_level = 1;
  };

  static_assert(sizeof(SyncCommand) == 6);
  static_assert(sizeof(SyncEvent) == 4);

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
  CameraSync(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
             const char *camera_pin_name, const char *camera_sync_topic_name,
             const char *imu_topic_name, uint32_t trigger_div,
             const char *camera_sync_command_topic_name)
      : camera_sync_pin_(
            *hw.template FindOrExit<LibXR::GPIO>({camera_pin_name})),
        imu_topic_(imu_topic_name, sizeof(ImuSample)),
        command_topic_(camera_sync_command_topic_name, sizeof(SyncCommand)),
        camera_sync_topic_(camera_sync_topic_name, sizeof(SyncEvent)),
        trigger_div_(ClampDiv(trigger_div)) {
    ASSERT(trigger_div != 0);

    camera_sync_pin_.SetConfig(
        {.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
         .pull = LibXR::GPIO::Pull::NONE});
    camera_sync_pin_.Write(false);

    imu_callback_ = LibXR::Topic::Callback::Create(
        [](bool in_isr, CameraSync *self, LibXR::MicrosecondTimestamp timestamp,
           const ImuSample &) { self->OnImuMessage(in_isr, timestamp); },
        this);
    imu_topic_.RegisterCallback(imu_callback_);

    command_callback_ = LibXR::Topic::Callback::Create(
        [](bool, CameraSync *self, LibXR::MicrosecondTimestamp,
           LibXR::RawData &data) { self->OnCommandData(data); },
        this);
    command_topic_.RegisterCallback(command_callback_);

    app.Register(*this);
  }

  void OnMonitor() override {}

private:
  static constexpr uint8_t command_version = 1;
  static constexpr uint8_t default_run_trigger_div = 50;
  static constexpr uint8_t min_pulse_hold_samples = 1;

  enum class SyncState : uint8_t { NORMAL = 0, WAIT_PROBE_EDGE = 1 };

  static uint8_t ClampDiv(uint32_t div) {
    if (div == 0) {
      return 1;
    }
    if (div > UINT8_MAX) {
      return UINT8_MAX;
    }
    return static_cast<uint8_t>(div);
  }

  void OnCommandData(LibXR::RawData &data) {
    if (data.addr_ == nullptr || data.size_ != sizeof(SyncCommand)) {
      return;
    }

    SyncCommand command;
    LibXR::Memory::FastCopy(&command, data.addr_, sizeof(command));
    OnCommand(command);
  }

  void OnCommand(const SyncCommand &command) {
    // 上位机应等待回执后再发下一条命令；模块只保留最新一条待执行命令。
    if (command.version != command_version || command.sync_probe_div == 0 ||
        command.run_trigger_div == 0) {
      return;
    }

    pending_command_.sync_probe_div = command.sync_probe_div;
    pending_command_.run_trigger_div = command.run_trigger_div;
    pending_command_.active_level = command.active_level == 0 ? 0U : 1U;
    pending_command_.seq = command.seq;
    pending_command_ready_ = true;
  }

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

    const uint16_t current_interval =
        sync_state_ == SyncState::WAIT_PROBE_EDGE ? active_probe_interval_samples_
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
    event.version = command_version;
    event.seq = active_seq_;
    event.run_trigger_div = pending_run_div_;
    event.active_level = active_level_;
    camera_sync_topic_.PublishFromCallback(event, imu_timestamp, in_isr);
  }

  LibXR::GPIO &camera_sync_pin_;

  LibXR::Topic imu_topic_;
  LibXR::Topic command_topic_;
  LibXR::Topic camera_sync_topic_;
  LibXR::Topic::Callback imu_callback_;
  LibXR::Topic::Callback command_callback_;

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
