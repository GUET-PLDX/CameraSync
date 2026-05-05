#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 由带时间戳 IMU 消息驱动的 MCU 侧相机触发同步模块
constructor_args:
  - camera_pin_name: "CAMERA"
  - camera_sync_topic_name: "camera_sync_result"
  - imu_topic_name: "bmi088_gyro"
  - trigger_div: 10
  - camera_sync_command_topic_name: "camera_sync_command"
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstdint>
#include <limits>

#include "app_framework.hpp"
#include "gpio.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "transform.hpp"

/**
 * @brief 相机同步触发模块。
 * @details 模块只使用 IMU topic 的消息时间戳。
 *          正常状态每 trigger_div 个 IMU 样本翻转一次相机 GPIO。同步命令在
 *          下一段反向电平周期开始执行，把这一段反向周期拉长 div 倍。长周期
 *          结束后的第一个有效电平边沿就是同步点。SyncEvent 只回传 seq，
 *          同步时间由 Topic 消息时间戳表示。
 */
class CameraSync : public LibXR::Application {
public:
  using ImuSample = Eigen::Matrix<float, 3, 1>;

  /**
   * @brief 上位机同步命令。
   * @details div 是单次反向电平周期拉长倍率，必须大于 0；active_level 是相机的
   *          有效触发电平，0 表示低有效，非 0 表示高有效；seq 由上位机每次同步
   *          自增，MCU 在对应同步点回传同一个 seq。
   */
  struct SyncCommand {
    uint32_t div = 1;
    uint32_t active_level = 1;
    uint32_t seq = 0;
  };

  /**
   * @brief 同步点回执。
   * @details 只回传 seq。实际同步时间使用 topic 消息自带 timestamp，不再复制到
   *          payload 里。
   */
  struct SyncEvent {
    uint32_t seq = 0;
  };

  /**
   * @brief 构造 CameraSync 模块。
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param camera_pin_name 相机触发 GPIO 名称
   * @param camera_sync_topic_name 同步结果 Topic 名称
   * @param imu_topic_name 作为同步基准的 IMU Topic 名称
   * @param trigger_div 每多少个 IMU 样本触发一次相机，必须大于 0
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
        trigger_div_(trigger_div), toggle_period_samples_(trigger_div) {
    ASSERT(trigger_div_ != 0);

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
  enum class SyncState : uint8_t {
    NORMAL = 0,
    WAIT_REVERSE_PERIOD = 1,
    STRETCH_REVERSE_PERIOD = 2,
  };

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
    if (command.div == 0 ||
        command.div > std::numeric_limits<uint32_t>::max() / trigger_div_) {
      return;
    }

    pending_command_.div = command.div;
    pending_command_.active_level = command.active_level == 0 ? 0U : 1U;
    pending_command_.seq = command.seq;
    pending_command_ready_ = true;
  }

  void StartPendingCommandIfIdle() {
    if (sync_state_ != SyncState::NORMAL || !pending_command_ready_) {
      return;
    }

    pending_command_ready_ = false;
    active_div_ = pending_command_.div;
    active_level_ = pending_command_.active_level;
    active_seq_ = pending_command_.seq;
    sync_state_ = SyncState::WAIT_REVERSE_PERIOD;
  }

  void OnImuMessage(bool in_isr, LibXR::MicrosecondTimestamp imu_timestamp) {
    StartPendingCommandIfIdle();
    samples_since_toggle_++;

    if (samples_since_toggle_ < toggle_period_samples_) {
      return;
    }

    samples_since_toggle_ = 0;
    const uint32_t pin_level = ToggleCameraPin();

    if (sync_state_ == SyncState::WAIT_REVERSE_PERIOD &&
        pin_level != active_level_) {
      sync_state_ = SyncState::STRETCH_REVERSE_PERIOD;
      toggle_period_samples_ = trigger_div_ * active_div_;
      return;
    }

    if (sync_state_ == SyncState::STRETCH_REVERSE_PERIOD) {
      if (pin_level == active_level_) {
        SyncEvent event;
        event.seq = active_seq_;
        camera_sync_topic_.PublishFromCallback(event, imu_timestamp, in_isr);
      }

      sync_state_ = SyncState::NORMAL;
      toggle_period_samples_ = trigger_div_;
      active_div_ = 1;
      active_level_ = 1;
      active_seq_ = 0;
      StartPendingCommandIfIdle();
    }
  }

  uint32_t ToggleCameraPin() {
    camera_sync_state_ = !camera_sync_state_;

    camera_sync_pin_.Write(camera_sync_state_);

    return camera_sync_state_ ? 1 : 0;
  }

  LibXR::GPIO &camera_sync_pin_;
  bool camera_sync_state_ = false;

  LibXR::Topic imu_topic_;
  LibXR::Topic command_topic_;
  LibXR::Topic camera_sync_topic_;
  LibXR::Topic::Callback imu_callback_;
  LibXR::Topic::Callback command_callback_;

  uint32_t trigger_div_ = 1;
  // 当前 GPIO 翻转周期，单位是 IMU 样本数。
  uint32_t toggle_period_samples_ = 1;
  uint32_t samples_since_toggle_ = 0;

  bool pending_command_ready_ = false;
  SyncCommand pending_command_;

  SyncState sync_state_ = SyncState::NORMAL;
  uint32_t active_div_ = 1;
  uint32_t active_level_ = 1;
  uint32_t active_seq_ = 0;
};
