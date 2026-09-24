# Camera recorder

An ESPHome component for recording an ESP32-S3 camera to a microSD card. Motion detection runs on camera frames; recordings can then be classified for people using a model stored on the card.

## How it works

The component samples camera frames for motion and publishes a **Motion Score**: the percentage of pixels that changed enough since the previous frame or a frame four samples earlier. When the score reaches **Motion Threshold**, the **Motion** binary sensor turns on; it turns off after **Motion Hold Time** without another qualifying frame. Motion detection alone does **not** start recording. Connect the sensor to the recording actions as shown below.

Recordings are MJPEG AVI files under `recordings/YYYY/MM/DD/` on the card. `stop_and_classify` finalizes the AVI, checks up to four frames for people, and labels it `_PERSON`, `_MOTION`, or `_UNCLASSIFIED`. The two **Record** switches decide whether successfully classified person and other-motion clips are kept. Classification failures are kept as `_UNCLASSIFIED`. Plain `stop` saves the AVI without classification. A temporary `.avi.idx` file supports AVI finalization; it is removed after a successful finish and retained if finalization fails.

Open `http://<device-address>/recordings` to browse, play, download, or delete clips. The page and downloads use the configured HTTP username and password.

## Complete example

This example uses the pinout from this repository's ESP32-S3 camera configuration. Change the camera, SD, I²C, and PSRAM settings for your board. Save the YAML at the repository root so the local component path resolves.

```yaml
esphome:
  name: camera-recorder
  min_version: 2026.9.0

esp32:
  variant: ESP32S3
  flash_size: 16MB
  framework:
    type: esp-idf

psram:
  mode: octal
  speed: 80MHz

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

api:
logger:

external_components:
  - source:
      type: local
      path: Components
    components: [camera_recorder]

i2c:
  id: camera_i2c
  sda: GPIO4
  scl: GPIO5

time:
  - platform: sntp
    id: camera_time

esp32_camera:
  id: main_camera
  internal: true
  resolution: 640x480
  jpeg_quality: 15
  max_framerate: 5 fps
  idle_framerate: 0 fps
  external_clock:
    pin: GPIO15
    frequency: 20MHz
  i2c_id: camera_i2c
  data_pins: [GPIO11, GPIO9, GPIO8, GPIO10, GPIO12, GPIO18, GPIO17, GPIO16]
  vsync_pin: GPIO6
  href_pin: GPIO7
  pixel_clock_pin: GPIO13

camera_recorder:
  id: recorder
  camera_id: main_camera
  time_id: camera_time
  clk_pin: GPIO39
  cmd_pin: GPIO38
  data0_pin: GPIO40
  web_username: !secret camera_recorder_web_username
  web_password: !secret camera_recorder_web_password

  directory: /recordings
  format_if_mount_failed: false
  light_accent: "#0878ff"
  dark_accent: "#65a8ff"
  light_active: "#dcecff"
  dark_active: "#1b385c"

  mounted:
    name: SD Card Mounted
  recording:
    name: Recording
  classifying:
    name: Classifying
  models_ready:
    name: Person Model Ready
  person_detected:
    name: Person Detected
  motion:
    name: Motion
    on_press:
      - camera_recorder.start: recorder
    on_release:
      - camera_recorder.stop_and_classify: recorder
  motion_score:
    name: Motion Score
  sd_free:
    name: SD Card Free
  motion_threshold:
    name: Motion Threshold
  motion_hold_time:
    name: Motion Hold Time
  motion_detection_fps:
    name: Motion Detection FPS
  min_person_detections:
    name: Minimum Person Detections
  record_people:
    name: Record People
  record_other_motion:
    name: Record Other Motion
  auto_deletion:
    name: Auto Deletion
  motion_detection:
    name: Motion Detection

button:
  - platform: template
    name: Start Recording
    on_press:
      - camera_recorder.start: recorder
  - platform: template
    name: Stop Recording
    on_press:
      - camera_recorder.stop: recorder
  - platform: template
    name: Stop and Classify Recording
    on_press:
      - camera_recorder.stop_and_classify: recorder
  - platform: template
    name: Delete Oldest Day
    on_press:
      - camera_recorder.delete_oldest_days:
          id: recorder
          days: 1
```

Add `wifi_ssid`, `wifi_password`, `camera_recorder_web_username`, and `camera_recorder_web_password` to `secrets.yaml`.

## Defaults and controls

| Setting | Default | Meaning |
| --- | --- | --- |
| `directory` | `/recordings` | Directory on the SD card; the card itself mounts at `/sdcard`. |
| `format_if_mount_failed` | `false` | If enabled, a failed mount may format the card and erase its contents. |
| `light_accent`, `dark_accent` | `#0878ff`, `#65a8ff` | Browser page accent colors. |
| `light_active`, `dark_active` | `#dcecff`, `#1b385c` | Browser page active-state colors. |
| Motion Threshold | `18%` | Minimum Motion Score to trigger Motion; adjustable from 0.1% to 100%. |
| Motion Hold Time | `10 s` | Time before Motion clears; adjustable from 1 to 300 seconds. |
| Motion Detection FPS | `0.8` | Motion samples per second; adjustable from 0.1 to 2.0. |
| Minimum Person Detections | `2` | Positive samples required among up to four AVI frames; adjustable from 1 to 4. |
| Record People / Record Other Motion | On / On | Keep clips after successful classification. |
| Auto Deletion | On | After a recording, delete the oldest complete recording days when card free space is below 10%; today's clips are retained. |
| Motion Detection | On | When off, stops motion frame processing, clears Motion, and sets Motion Score to 0%. |

Number and switch values are restored after reboot. `mounted` and `recording` sensors are optional; the other listed controls and status entities are created by default. Without valid time, recordings go into `recordings/unsynced/`.

## Person model

Before first boot, mount the microSD card on your computer and run:

```sh
Components/camera_recorder/download_models.sh /path/to/sd-card
```

Pass the **card root**, not its `models` directory. The script downloads and verifies `models/s3/pedestrian_detect_pico_s8_v1.espdl`. **Person Model Ready** indicates whether it loaded. Motion detection and recording can still run without the model, but person classification cannot.
