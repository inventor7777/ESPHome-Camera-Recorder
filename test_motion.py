#!/usr/bin/env python3
from pathlib import Path


def score(previous, current):
    return sum(abs(a - b) >= 20 for a, b in zip(previous, current)) * 100 / len(previous)


def classification_frames(frame_count):
    return list(dict.fromkeys(frame_count * marker // 4 for marker in range(4)))


assert score([10] * 100, [10] * 100) == 0
assert score([10] * 100, [40] * 25 + [10] * 75) == 25
assert score([10] * 100, [40] * 25 + [10] * 75) >= 18.0
frames = [[40 if x in range(frame, frame + 4) else 10 for x in range(20)] * 10 for frame in range(5)]
assert score(frames[3], frames[4]) == 10
assert score(frames[0], frames[4]) == 40
assert max(score(frames[3], frames[4]), score(frames[0], frames[4])) >= 18.0
assert classification_frames(1) == [0]
assert classification_frames(4) == [0, 1, 2, 3]
assert classification_frames(100) == [0, 25, 50, 75]

source = (Path(__file__).parent / "components/camera_recorder/camera_recorder.cpp").read_text()
assert "this->motion_result_score_ >= this->motion_threshold_" in source
start = source.index("static uint8_t classification_divisor")
body = " ".join(source[start : source.index("\n}", start)].split())
branches = (
    "if (width / 8 >= CLASSIFICATION_MIN_SIZE && height / 8 >= CLASSIFICATION_MIN_SIZE) return 8;",
    "if (width / 4 >= CLASSIFICATION_MIN_SIZE && height / 4 >= CLASSIFICATION_MIN_SIZE) return 4;",
    "if (width / 2 >= CLASSIFICATION_MIN_SIZE && height / 2 >= CLASSIFICATION_MIN_SIZE) return 2;",
    "return 1;",
)
positions = [body.index(branch) for branch in branches]
assert positions == sorted(positions)
start = source.index("void CameraRecorder::finish_recording_")
finish_body = " ".join(source[start : source.index("\n}", start)].split())
assert "if (this->frame_count_ == 0) { if (remove(this->filename_.c_str()) != 0)" in finish_body
assert "static_cast<uint64_t>(this->frame_count_) * marker / 4" in source
assert "state->detections >= this->min_person_detections_" in source
assert 'name.find("..") == std::string::npos' in source
assert "Accept-Ranges: bytes" in source
assert not any(f"request->send({code}," in source for code in (202, 403, 503))
assert 'send_text(request, "503 Service Unavailable"' in source
assert '"Content-Range: bytes %" PRIu64' in source
assert "end = std::min<uint64_t>(end, start + WEB_RANGE_MAX_BYTES - 1);" in source
assert "send_all(raw, buffer, read)" in source
assert "mount_config.max_files = 4" in source
assert "this->defer([this]() { this->web_server_->init(); });" in source
assert "PedestrianDetect detector;" not in source
assert "new (std::nothrow) PedestrianDetect(PedestrianDetect::PICO_S8_V1, false)" in source
assert "*this->detector_" in source
assert 'xTaskCreatePinnedToCore(&CameraRecorder::motion_task_' in source
assert "this->motion_source_image_ = image" in source
assert "if (!this->motion_detection_enabled_ && !this->recording_)" in source
assert "this->motion_detection_enabled_ && now - this->last_motion_analysis_ms_" in source
assert "this->motion_detection_enabled_ && !this->motion_busy_" in source
assert "if (this->motion_detection_enabled_ && this->motion_result_valid_)" in source
assert "this->motion_history_frames_ = 0" in source
assert "memcpy(recorder->motion_jpeg_" in source
assert "MOTION_JPEG_INITIAL_CAPACITY = 512 * 1024" in source
assert "MOTION_COPY_CHUNK_SIZE = 8 * 1024" in source
assert "offset += MOTION_COPY_CHUNK_SIZE" in source
assert "this->motion_jpeg_size_ * 2" in source
assert "heap_caps_realloc(this->motion_jpeg_" not in source
assert "this->motion_fps_number_->setup_restore(0.8f)" in source
assert source.count("this->motion_interval_ms_") == 2
assert "request_image(camera::IDLE)" not in source
assert source.count("request_image(camera::WEB_REQUESTER)") == 2
assert "MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT" in source
assert "jpeg_dec_process(decoder, &io)" in source
assert 'ESP_LOGW(TAG, "Failed to decode classification JPEG")' in source
assert 'jpeg_decoder.h' not in source
assert 'esp_jpeg_decode(' not in source
assert 'esp_jpeg_get_image_info(' not in source
motion_start = source.index("bool CameraRecorder::process_motion_")
motion_end = source.index("bool CameraRecorder::write_avi_header_", motion_start)
motion_body = source[motion_start:motion_end]
assert "changed_over_four_frames" in motion_body
assert "MOTION_HISTORY_FRAMES = 4" in source
assert "this->motion_luma_history_[history_offset + index]" in motion_body
assert "std::max(changed_recent, changed_over_four_frames) * 100.0f / pixel_count" in motion_body
assert "config.output_type = JPEG_PIXEL_FORMAT_RGB565_LE;" in motion_body
assert "jpeg_dec_process(decoder, &io)" in motion_body
assert 'ESP_LOGW(TAG, "Failed to decode motion JPEG")' in motion_body
assert "Motion decoder A/B" not in source
assert "comparison_luma_" not in source
assert "heap_caps_aligned_alloc(16, rgb_size" in source
assert "motion_result_ready_.store(true, std::memory_order_release)" in source
assert "esp_wifi_sta_get_ap_info" not in source
assert 'url == "/recordings/toggle"' in source
assert "this->web_toggle_requested_.exchange(false)" in source
index_start = source.index("void CameraRecorder::handle_index_")
index_body = source[index_start : source.index("void CameraRecorder::handle_file_", index_start)]
assert "httpd_resp_send(raw, html.data(), html.size())" in index_body
assert "httpd_resp_send_chunk" not in index_body
print("motion and camera recorder source checks passed")
