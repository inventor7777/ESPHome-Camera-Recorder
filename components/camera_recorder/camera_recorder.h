#pragma once

#include <cstdio>
#include <atomic>
#include <string>
#include <dirent.h>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class PedestrianDetect;

namespace esphome::camera_recorder {

class CameraRecorder;

enum class CameraRecorderNumberType : uint8_t { THRESHOLD, HOLD_TIME, MOTION_FPS, MIN_PERSON_DETECTIONS };
enum class CameraRecorderSwitchType : uint8_t { PEOPLE, OTHER_MOTION, AUTO_DELETION, MOTION_DETECTION };
enum class WebDeletionState : uint8_t { IDLE, QUEUED, RUNNING, DONE, FAILED };
struct ClassificationState;

class CameraRecorderNumber final : public number::Number {
 public:
  void set_parent(CameraRecorder *parent) { this->parent_ = parent; }
  void set_type(CameraRecorderNumberType type) { this->type_ = type; }
  void setup_restore(float default_value);

 protected:
  void control(float value) override;
  void apply_(float value);

  CameraRecorder *parent_{nullptr};
  CameraRecorderNumberType type_{CameraRecorderNumberType::THRESHOLD};
  ESPPreferenceObject preference_;
};

class CameraRecorderSwitch final : public switch_::Switch {
 public:
  void set_parent(CameraRecorder *parent) { this->parent_ = parent; }
  void set_type(CameraRecorderSwitchType type) { this->type_ = type; }
  CameraRecorderSwitchType get_type() const { return this->type_; }
  void setup_restore(bool default_value);

 protected:
  void write_state(bool state) override;

  CameraRecorder *parent_{nullptr};
  CameraRecorderSwitchType type_{CameraRecorderSwitchType::PEOPLE};
};

class CameraRecorder final : public Component, public camera::CameraListener, public AsyncWebHandler {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;
  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void on_camera_image(const std::shared_ptr<camera::CameraImage> &image) override;

  void set_camera(camera::Camera *camera) { this->camera_ = camera; }
  void set_time(time::RealTimeClock *clock) { this->time_ = clock; }
  void set_web_server(web_server_base::WebServerBase *web_server) { this->web_server_ = web_server; }
  void set_pins(uint8_t clk, uint8_t cmd, uint8_t data0) {
    this->clk_pin_ = clk;
    this->cmd_pin_ = cmd;
    this->data0_pin_ = data0;
  }
  void set_directory(const std::string &directory) { this->directory_ = directory; }
  void set_light_accent(const std::string &color) { this->light_accent_ = color; }
  void set_dark_accent(const std::string &color) { this->dark_accent_ = color; }
  void set_light_active(const std::string &color) { this->light_active_ = color; }
  void set_dark_active(const std::string &color) { this->dark_active_ = color; }
  void set_format_if_mount_failed(bool format) { this->format_if_mount_failed_ = format; }
  void set_mounted_sensor(binary_sensor::BinarySensor *sensor) { this->mounted_sensor_ = sensor; }
  void set_recording_sensor(binary_sensor::BinarySensor *sensor) { this->recording_sensor_ = sensor; }
  void set_classifying_sensor(binary_sensor::BinarySensor *sensor) { this->classifying_sensor_ = sensor; }
  void set_models_ready_sensor(binary_sensor::BinarySensor *sensor) { this->models_ready_sensor_ = sensor; }
  void set_person_detected_sensor(binary_sensor::BinarySensor *sensor) { this->person_detected_sensor_ = sensor; }
  void set_motion_sensor(binary_sensor::BinarySensor *sensor) { this->motion_sensor_ = sensor; }
  void set_motion_score_sensor(sensor::Sensor *sensor) { this->motion_score_sensor_ = sensor; }
  void set_sd_free_sensor(sensor::Sensor *sensor) { this->sd_free_sensor_ = sensor; }
  void set_threshold_number(CameraRecorderNumber *number) { this->threshold_number_ = number; }
  void set_hold_time_number(CameraRecorderNumber *number) { this->hold_time_number_ = number; }
  void set_motion_fps_number(CameraRecorderNumber *number) { this->motion_fps_number_ = number; }
  void set_min_person_detections_number(CameraRecorderNumber *number) { this->min_person_detections_number_ = number; }

  void set_motion_threshold(float value) { this->motion_threshold_ = value; }
  void set_motion_hold_time(float value) { this->motion_hold_time_s_ = value; }
  void set_motion_fps(float value) { this->motion_interval_ms_ = static_cast<uint32_t>(1000.0f / value + 0.5f); }
  void set_min_person_detections(uint8_t value) { this->min_person_detections_ = value; }
  void add_record_switch(CameraRecorderSwitch *record_switch);
  void set_record_people(bool state) { this->record_people_ = state; }
  void set_record_other_motion(bool state) { this->record_other_motion_ = state; }
  void set_motion_detection(bool state);
  void set_auto_deletion(bool state) {
    this->auto_deletion_ = state;
    if (!state)
      this->auto_delete_pending_ = false;
  }

  void start();
  void stop();
  void stop_and_classify();
  void delete_oldest_days(uint8_t days);

 protected:
  bool mount_();
  bool open_next_file_();
  bool ensure_directory_(const std::string &path);
  std::string unique_filename_(const std::string &stem, const char *suffix) const;
  bool write_avi_header_(const uint8_t *jpeg, size_t length);
  bool finalize_avi_();
  void finish_recording_(bool classify);
  bool models_ready_() const;
  void run_classification_();
  void finish_classification_(bool succeeded, bool person);
  void clear_classification_state_();
  bool queue_motion_(const std::shared_ptr<camera::CameraImage> &image);
  bool process_motion_(const uint8_t *jpeg, size_t length, float *score);
  void publish_motion_result_();
  static void motion_task_(void *arg);
  bool resize_motion_buffers_(size_t rgb_size, size_t pixel_count);
  void publish_motion_(bool state);
  void fail_recording_(const char *message);
  void publish_recording_(bool state);
  void update_storage_();
  void prune_step_();
  void handle_index_(AsyncWebServerRequest *request);
  void handle_file_(AsyncWebServerRequest *request);
  void handle_delete_(AsyncWebServerRequest *request);

  camera::Camera *camera_{nullptr};
  time::RealTimeClock *time_{nullptr};
  web_server_base::WebServerBase *web_server_{nullptr};
  binary_sensor::BinarySensor *mounted_sensor_{nullptr};
  binary_sensor::BinarySensor *recording_sensor_{nullptr};
  binary_sensor::BinarySensor *classifying_sensor_{nullptr};
  binary_sensor::BinarySensor *models_ready_sensor_{nullptr};
  binary_sensor::BinarySensor *person_detected_sensor_{nullptr};
  binary_sensor::BinarySensor *motion_sensor_{nullptr};
  sensor::Sensor *motion_score_sensor_{nullptr};
  sensor::Sensor *sd_free_sensor_{nullptr};
  CameraRecorderNumber *threshold_number_{nullptr};
  CameraRecorderNumber *hold_time_number_{nullptr};
  CameraRecorderNumber *motion_fps_number_{nullptr};
  CameraRecorderNumber *min_person_detections_number_{nullptr};
  CameraRecorderSwitch *record_people_switch_{nullptr};
  CameraRecorderSwitch *record_other_motion_switch_{nullptr};
  CameraRecorderSwitch *auto_deletion_switch_{nullptr};
  CameraRecorderSwitch *motion_detection_switch_{nullptr};
  ::PedestrianDetect *detector_{nullptr};
  sdmmc_card_t *card_{nullptr};
  FILE *file_{nullptr};
  FILE *index_file_{nullptr};
  std::string directory_{"/recordings"};
  std::string light_accent_{"#0878ff"};
  std::string dark_accent_{"#65a8ff"};
  std::string light_active_{"#dcecff"};
  std::string dark_active_{"#1b385c"};
  std::string filename_;
  std::string index_filename_;
  std::string filename_stem_;
  uint8_t clk_pin_{39};
  uint8_t cmd_pin_{38};
  uint8_t data0_pin_{40};
  bool format_if_mount_failed_{false};
  bool mounted_{false};
  std::atomic<bool> recording_{false};
  std::atomic<bool> recording_pending_{false};
  std::atomic<bool> motion_active_{false};
  std::atomic<bool> motion_detection_enabled_{true};
  std::atomic<bool> classification_pending_{false};
  std::atomic<bool> web_active_{false};
  std::atomic<bool> web_toggle_requested_{false};
  std::atomic<WebDeletionState> web_delete_state_{WebDeletionState::IDLE};
  std::string web_delete_date_;
  std::string web_delete_file_;
  std::atomic<uint32_t> sd_total_mib_{0};
  std::atomic<uint32_t> sd_free_mib_{0};
  bool record_people_{true};
  bool record_other_motion_{true};
  bool auto_deletion_{true};
  bool auto_delete_pending_{false};
  uint16_t manual_delete_days_{0};
  DIR *prune_dir_{nullptr};
  std::string prune_date_;
  bool prune_removed_any_{false};
  ClassificationState *classification_state_{nullptr};
  uint32_t classification_queued_ms_{0};
  TaskHandle_t motion_task_handle_{nullptr};
  std::atomic<bool> motion_task_running_{false};
  std::atomic<bool> motion_task_stop_{false};
  std::atomic<bool> motion_busy_{false};
  std::atomic<bool> motion_result_ready_{false};
  std::atomic<bool> motion_reset_pending_{false};
  std::shared_ptr<camera::CameraImage> motion_source_image_;
  uint8_t *motion_jpeg_{nullptr};
  size_t motion_jpeg_size_{0};
  size_t motion_jpeg_length_{0};
  bool motion_result_valid_{false};
  float motion_result_score_{0.0f};
  uint8_t *motion_rgb_{nullptr};
  uint8_t *motion_luma_history_{nullptr};
  size_t motion_rgb_size_{0};
  size_t motion_pixel_count_{0};
  float motion_threshold_{18.0f};
  float motion_hold_time_s_{10.0f};
  uint32_t motion_interval_ms_{1250};
  uint8_t min_person_detections_{2};
  uint8_t motion_history_frames_{0};
  uint8_t motion_history_slot_{0};
  uint32_t last_motion_analysis_ms_{0};
  uint32_t last_motion_detected_ms_{0};
  uint32_t last_motion_request_ms_{0};
  uint32_t last_storage_update_ms_{0};
  uint32_t frame_count_{0};
  uint32_t file_size_{0};
  uint32_t max_frame_size_{0};
  uint32_t first_frame_ms_{0};
  uint32_t last_frame_ms_{0};
  uint32_t last_flush_ms_{0};
  size_t bytes_written_{0};
};

template<typename... Ts> class CameraRecorderStartAction final : public Action<Ts...>, public Parented<CameraRecorder> {
 public:
  void play(const Ts &...x) override { this->parent_->start(); }
};

template<typename... Ts> class CameraRecorderStopAction final : public Action<Ts...>, public Parented<CameraRecorder> {
 public:
  void play(const Ts &...x) override { this->parent_->stop(); }
};

template<typename... Ts>
class CameraRecorderStopAndClassifyAction final : public Action<Ts...>, public Parented<CameraRecorder> {
 public:
  void play(const Ts &...x) override { this->parent_->stop_and_classify(); }
};

template<typename... Ts> class CameraRecorderDeleteDaysAction final : public Action<Ts...>, public Parented<CameraRecorder> {
 public:
  void set_days(uint8_t days) { this->days_ = days; }
  void play(const Ts &...x) override { this->parent_->delete_oldest_days(this->days_); }

 protected:
  uint8_t days_{1};
};

}  // namespace esphome::camera_recorder
