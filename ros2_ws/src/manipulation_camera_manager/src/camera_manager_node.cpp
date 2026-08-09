#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "manipulation_camera_manager/jpeg_decoder.hpp"
#include "manipulation_camera_manager/phase_scheduler.hpp"
#include "manipulation_camera_manager/rolling_statistics.hpp"
#include "manipulation_camera_manager/v4l2_camera.hpp"

namespace manipulation_camera_manager {
namespace {

diagnostic_msgs::msg::KeyValue Value(
    const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string Decimal(double value) {
  std::ostringstream text;
  text << std::fixed << std::setprecision(2) << value;
  return text.str();
}

std::string Statistic(double value, std::size_t count) {
  return count == 0U ? "-1" : Decimal(value);
}

struct DecodeRuntime {
  explicit DecodeRuntime(std::size_t statistics_capacity)
      : frame_age_ms(statistics_capacity),
        decode_time_ms(statistics_capacity) {}

  RollingStatistics frame_age_ms;
  RollingStatistics decode_time_ms;
  std::string frame_id;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
  std::chrono::steady_clock::time_point next_decode_at{};
  std::uint64_t last_generation{0};
  std::uint64_t decoded_frames{0};
  std::uint64_t decode_failures{0};
  std::string last_error;
};

}  // namespace

class CameraManagerNode final : public rclcpp::Node {
 public:
  CameraManagerNode() : Node("camera_manager") {
    const auto names = declare_parameter<std::vector<std::string>>(
        "camera_names",
        std::vector<std::string>{"top", "wrist_a", "wrist_b"});
    const auto width = declare_parameter<std::int64_t>("capture.width", 640);
    const auto height = declare_parameter<std::int64_t>("capture.height", 480);
    const auto fps = declare_parameter<std::int64_t>("capture.fps", 30);
    const auto buffer_count =
        declare_parameter<std::int64_t>("capture.buffer_count", 4);
    const auto disable_dynamic_framerate =
        declare_parameter<bool>("capture.disable_dynamic_framerate", true);
    const auto reconnect_ms =
        declare_parameter<std::int64_t>("reconnect_interval_ms", 500);
    const auto frame_timeout_ms =
        declare_parameter<std::int64_t>("frame_timeout_ms", 1500);
    stale_after_ms_ = declare_parameter<std::int64_t>("stale_after_ms", 500);
    const auto diagnostic_rate =
        declare_parameter<double>("diagnostic_rate_hz", 1.0);
    const auto scheduler_tick_hz =
        declare_parameter<double>("scheduler_tick_hz", 100.0);
    max_frame_age_p95_ms_ =
        declare_parameter<double>("limits.max_frame_age_p95_ms", 200.0);
    max_decode_p95_ms_ =
        declare_parameter<double>("limits.max_decode_p95_ms", 50.0);
    const auto statistics_capacity = declare_parameter<std::int64_t>(
        "statistics_window_samples", 300);
    const auto max_total_inference_hz = declare_parameter<double>(
        "vision.max_total_inference_hz", 12.0);

    if (names.empty() || width <= 0 || height <= 0 || fps <= 0 ||
        buffer_count < 2 || reconnect_ms < 100 || frame_timeout_ms < 500 ||
        stale_after_ms_ < 100 || diagnostic_rate <= 0.0 ||
        scheduler_tick_hz < 10.0 || max_frame_age_p95_ms_ <= 0.0 ||
        max_decode_p95_ms_ <= 0.0 || statistics_capacity <= 0 ||
        max_total_inference_hz <= 0.0) {
      throw std::invalid_argument("invalid camera manager parameter");
    }

    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "camera_diagnostics", 10);

    for (const auto& name : names) {
      CameraConfig config;
      config.name = name;
      config.device_path = declare_parameter<std::string>(
          name + ".device_path", "");
      if (config.device_path.empty()) {
        throw std::invalid_argument(name + ".device_path must not be empty");
      }
      config.width = static_cast<std::uint32_t>(width);
      config.height = static_cast<std::uint32_t>(height);
      config.fps = static_cast<std::uint32_t>(fps);
      config.buffer_count = static_cast<std::uint32_t>(buffer_count);
      config.disable_dynamic_framerate = disable_dynamic_framerate;
      config.reconnect_interval = std::chrono::milliseconds(reconnect_ms);
      config.frame_timeout = std::chrono::milliseconds(frame_timeout_ms);
      cameras_.push_back(std::make_unique<V4l2Camera>(std::move(config)));

      auto runtime = std::make_unique<DecodeRuntime>(
          static_cast<std::size_t>(statistics_capacity));
      runtime->frame_id = declare_parameter<std::string>(
          name + ".frame_id", name + "_optical_frame");
      runtime->publisher = create_publisher<sensor_msgs::msg::Image>(
          "camera/" + name + "/image_raw",
          rclcpp::SensorDataQoS().keep_last(1));
      decode_runtime_.push_back(std::move(runtime));
    }

    LoadSchedule(names, max_total_inference_hz);
    active_phase_ = NormalizePhase(
        declare_parameter<std::string>("initial_phase", "STANDBY"));
    if (!schedule_.HasPhase(active_phase_)) {
      throw std::invalid_argument("initial_phase is not in phase_names");
    }

    phase_subscription_ = create_subscription<std_msgs::msg::String>(
        "camera_phase", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr message) {
          ChangePhase(message->data);
        });

    for (auto& camera : cameras_) {
      camera->Start();
    }
    previous_frames_.resize(cameras_.size(), 0U);
    previous_sample_at_.resize(
        cameras_.size(), std::chrono::steady_clock::now());
    diagnostics_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / diagnostic_rate),
        [this]() { PublishDiagnostics(); });
    scheduler_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / scheduler_tick_hz),
        [this]() { RunDecodeScheduler(); });

    RCLCPP_INFO(
        get_logger(), "camera scheduler ready phase=%s", active_phase_.c_str());
  }

  ~CameraManagerNode() override {
    for (auto& camera : cameras_) {
      camera->Stop();
    }
  }

 private:
  void LoadSchedule(
      const std::vector<std::string>& camera_names,
      double max_total_inference_hz) {
    const auto phases = declare_parameter<std::vector<std::string>>(
        "phase_names", std::vector<std::string>{"STANDBY"});
    if (phases.empty()) {
      throw std::invalid_argument("phase_names must not be empty");
    }
    for (const auto& raw_phase : phases) {
      const auto phase = NormalizePhase(raw_phase);
      double inference_sum = 0.0;
      for (const auto& camera : camera_names) {
        const std::string prefix = "schedule." + phase + "." + camera;
        CameraPhaseRule rule;
        rule.decode_hz = declare_parameter<double>(
            prefix + ".decode_hz", 0.0);
        rule.inference_hz = declare_parameter<double>(
            prefix + ".inference_hz", 0.0);
        if (rule.decode_hz < 0.0 || rule.inference_hz < 0.0 ||
            rule.inference_hz > rule.decode_hz) {
          throw std::invalid_argument(
              prefix + " must satisfy 0 <= inference_hz <= decode_hz");
        }
        inference_sum += rule.inference_hz;
        schedule_.AddRule(phase, camera, rule);
      }
      if (inference_sum > max_total_inference_hz + 1e-9) {
        throw std::invalid_argument(
            "phase " + phase + " exceeds vision.max_total_inference_hz");
      }
    }
  }

  void ChangePhase(const std::string& requested) {
    const auto phase = NormalizePhase(requested);
    if (!schedule_.HasPhase(phase)) {
      ++rejected_phase_commands_;
      RCLCPP_WARN(
          get_logger(), "rejected unknown camera phase: %s",
          requested.c_str());
      return;
    }
    if (phase == active_phase_) {
      return;
    }
    active_phase_ = phase;
    for (auto& runtime : decode_runtime_) {
      runtime->frame_age_ms.Clear();
      runtime->decode_time_ms.Clear();
      runtime->next_decode_at = {};
      runtime->last_generation = 0;
      runtime->decoded_frames = 0;
      runtime->decode_failures = 0;
      runtime->last_error.clear();
    }
    RCLCPP_INFO(get_logger(), "camera phase changed to %s", phase.c_str());
  }

  void RunDecodeScheduler() {
    const auto sampled_at = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < cameras_.size(); ++index) {
      auto& camera = cameras_[index];
      auto& runtime = *decode_runtime_[index];
      const auto& rule = schedule_.Rule(
          active_phase_, camera->Config().name);
      if (rule.decode_hz <= 0.0) {
        continue;
      }
      const auto interval = std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / rule.decode_hz));
      if (runtime.next_decode_at.time_since_epoch().count() == 0) {
        runtime.next_decode_at = sampled_at;
      }
      if (sampled_at < runtime.next_decode_at) {
        continue;
      }
      runtime.next_decode_at += interval;
      if (runtime.next_decode_at <= sampled_at) {
        runtime.next_decode_at = sampled_at + interval;
      }
      const auto frame = camera->LatestFrame();
      if (!frame.has_value() || frame->generation == runtime.last_generation) {
        continue;
      }
      runtime.last_generation = frame->generation;
      const double frame_age_ms =
          std::chrono::duration<double, std::milli>(
              sampled_at - frame->captured_at)
              .count();
      const auto decode_started_at = std::chrono::steady_clock::now();
      try {
        DecodedImage decoded = DecodeJpeg(*frame->bytes);
        const double decode_time_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - decode_started_at)
                .count();

        sensor_msgs::msg::Image message;
        const auto age_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            sampled_at - frame->captured_at);
        message.header.stamp =
            now() - rclcpp::Duration::from_nanoseconds(age_ns.count());
        message.header.frame_id = runtime.frame_id;
        message.height = decoded.height;
        message.width = decoded.width;
        message.encoding = "rgb8";
        message.is_bigendian = false;
        message.step = decoded.width * 3U;
        message.data = std::move(decoded.rgb);
        runtime.publisher->publish(std::move(message));

        runtime.frame_age_ms.Add(std::max(0.0, frame_age_ms));
        runtime.decode_time_ms.Add(std::max(0.0, decode_time_ms));
        ++runtime.decoded_frames;
        runtime.last_error.clear();
      } catch (const std::exception& error) {
        ++runtime.decode_failures;
        runtime.last_error = error.what();
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "%s decode failed: %s",
            camera->Config().name.c_str(), error.what());
      }
    }
  }

  void PublishDiagnostics() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    const auto sampled_at = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < cameras_.size(); ++index) {
      const auto& camera = cameras_[index];
      const auto& runtime = *decode_runtime_[index];
      const CameraStats stats = camera->Stats();
      const double elapsed_seconds =
          std::chrono::duration<double>(sampled_at - previous_sample_at_[index])
              .count();
      const std::uint64_t frame_delta =
          stats.frames_received - previous_frames_[index];
      const double measured_fps =
          elapsed_seconds > 0.0
              ? static_cast<double>(frame_delta) / elapsed_seconds
              : 0.0;
      previous_frames_[index] = stats.frames_received;
      previous_sample_at_[index] = sampled_at;
      const auto age = runtime.frame_age_ms.Summary();
      const auto decode = runtime.decode_time_ms.Summary();
      const auto& rule = schedule_.Rule(
          active_phase_, camera->Config().name);

      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "camera_manager/" + camera->Config().name;
      status.hardware_id = camera->Config().device_path;

      if (stats.state != CameraState::kStreaming) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = CameraStateName(stats.state);
      } else if (stats.latest_age_ms < 0 ||
                 stats.latest_age_ms > stale_after_ms_) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "STALE_FRAME";
      } else if (!runtime.last_error.empty() ||
                 (age.count > 0U && age.p95 > max_frame_age_p95_ms_) ||
                 (decode.count > 0U && decode.p95 > max_decode_p95_ms_)) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "STREAMING_DECODE_WARN";
      } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "STREAMING";
      }

      status.values.push_back(Value("state", CameraStateName(stats.state)));
      status.values.push_back(Value("phase", active_phase_));
      status.values.push_back(
          Value("configured_decode_hz", Decimal(rule.decode_hz)));
      status.values.push_back(
          Value("configured_inference_hz", Decimal(rule.inference_hz)));
      status.values.push_back(
          Value("frames_received", std::to_string(stats.frames_received)));
      status.values.push_back(Value("measured_fps", Decimal(measured_fps)));
      status.values.push_back(Value(
          "driver_frames_dropped",
          std::to_string(stats.driver_frames_dropped)));
      status.values.push_back(
          Value("reconnect_count", std::to_string(stats.reconnect_count)));
      status.values.push_back(
          Value("latest_generation", std::to_string(stats.latest_generation)));
      status.values.push_back(
          Value("latest_bytes", std::to_string(stats.latest_bytes)));
      status.values.push_back(
          Value("latest_age_ms", std::to_string(stats.latest_age_ms)));
      status.values.push_back(
          Value("decoded_frames", std::to_string(runtime.decoded_frames)));
      status.values.push_back(
          Value("decode_failures", std::to_string(runtime.decode_failures)));
      status.values.push_back(Value(
          "decode_frame_age_p50_ms", Statistic(age.p50, age.count)));
      status.values.push_back(Value(
          "decode_frame_age_p95_ms", Statistic(age.p95, age.count)));
      status.values.push_back(Value(
          "decode_frame_age_max_ms", Statistic(age.maximum, age.count)));
      status.values.push_back(Value(
          "decode_time_p50_ms", Statistic(decode.p50, decode.count)));
      status.values.push_back(Value(
          "decode_time_p95_ms", Statistic(decode.p95, decode.count)));
      status.values.push_back(Value(
          "decode_time_max_ms", Statistic(decode.maximum, decode.count)));
      status.values.push_back(Value("capture_error", stats.last_error));
      status.values.push_back(Value("decode_error", runtime.last_error));
      array.status.push_back(std::move(status));
    }

    diagnostic_msgs::msg::DiagnosticStatus scheduler;
    scheduler.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    scheduler.name = "camera_manager/scheduler";
    scheduler.message = "ACTIVE";
    scheduler.hardware_id = "phase_scheduler";
    scheduler.values.push_back(Value("active_phase", active_phase_));
    scheduler.values.push_back(Value(
        "rejected_phase_commands", std::to_string(rejected_phase_commands_)));
    array.status.push_back(std::move(scheduler));
    diagnostics_publisher_->publish(std::move(array));
  }

  std::int64_t stale_after_ms_{500};
  double max_frame_age_p95_ms_{200.0};
  double max_decode_p95_ms_{50.0};
  PhaseSchedule schedule_;
  std::string active_phase_;
  std::uint64_t rejected_phase_commands_{0};
  std::vector<std::unique_ptr<V4l2Camera>> cameras_;
  std::vector<std::unique_ptr<DecodeRuntime>> decode_runtime_;
  std::vector<std::uint64_t> previous_frames_;
  std::vector<std::chrono::steady_clock::time_point> previous_sample_at_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr scheduler_timer_;
};

}  // namespace manipulation_camera_manager

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<
      manipulation_camera_manager::CameraManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
