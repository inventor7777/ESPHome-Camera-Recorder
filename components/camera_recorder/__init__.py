import re

from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import (
    binary_sensor,
    esp32_camera,
    number,
    sensor,
    switch,
    time,
    web_server_base,
)
from esphome.components.web_server_base import CONF_WEB_SERVER_BASE_ID
from esphome.components.esp32 import (
    add_idf_component,
    add_idf_sdkconfig_option,
    get_esp32_variant,
    include_builtin_idf_component,
    require_fatfs,
    require_vfs_dir,
)
from esphome.components.esp32.const import VARIANT_ESP32S3
import esphome.config_validation as cv
from esphome.const import (
    CONF_CLK_PIN,
    CONF_ID,
    CONF_NAME,
    CONF_TIME_ID,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_MOTION,
    DEVICE_CLASS_RUNNING,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
    UNIT_SECOND,
)
from esphome.core import CORE

CODEOWNERS = []
DEPENDENCIES = ["esp32_camera", "time", "psram"]
AUTO_LOAD = ["binary_sensor", "number", "sensor", "switch", "web_server_base"]

CONF_CAMERA_ID = "camera_id"
CONF_CMD_PIN = "cmd_pin"
CONF_DATA0_PIN = "data0_pin"
CONF_DIRECTORY = "directory"
CONF_LIGHT_ACCENT = "light_accent"
CONF_DARK_ACCENT = "dark_accent"
CONF_LIGHT_ACTIVE = "light_active"
CONF_DARK_ACTIVE = "dark_active"
CONF_FORMAT_IF_MOUNT_FAILED = "format_if_mount_failed"
CONF_MOUNTED = "mounted"
CONF_MOTION = "motion"
CONF_MOTION_DETECTION = "motion_detection"
CONF_MOTION_HOLD_TIME = "motion_hold_time"
CONF_MOTION_FPS = "motion_detection_fps"
CONF_MIN_PERSON_DETECTIONS = "min_person_detections"
CONF_MOTION_SCORE = "motion_score"
CONF_MOTION_THRESHOLD = "motion_threshold"
CONF_RECORDING = "recording"
CONF_CLASSIFYING = "classifying"
CONF_MODELS_READY = "models_ready"
CONF_PERSON_DETECTED = "person_detected"
CONF_RECORD_PEOPLE = "record_people"
CONF_RECORD_OTHER_MOTION = "record_other_motion"
CONF_AUTO_DELETION = "auto_deletion"
CONF_DAYS = "days"
CONF_SD_FREE = "sd_free"
CONF_WEB_PASSWORD = "web_password"
CONF_WEB_USERNAME = "web_username"

camera_recorder_ns = cg.esphome_ns.namespace("camera_recorder")
CameraRecorder = camera_recorder_ns.class_("CameraRecorder", cg.Component)
CameraRecorderNumber = camera_recorder_ns.class_("CameraRecorderNumber", number.Number)
CameraRecorderNumberType = camera_recorder_ns.enum(
    "CameraRecorderNumberType", is_class=True
)
CameraRecorderSwitch = camera_recorder_ns.class_("CameraRecorderSwitch", switch.Switch)
CameraRecorderSwitchType = camera_recorder_ns.enum(
    "CameraRecorderSwitchType", is_class=True
)
CameraRecorderStartAction = camera_recorder_ns.class_(
    "CameraRecorderStartAction", automation.Action
)
CameraRecorderStopAction = camera_recorder_ns.class_(
    "CameraRecorderStopAction", automation.Action
)
CameraRecorderStopAndClassifyAction = camera_recorder_ns.class_(
    "CameraRecorderStopAndClassifyAction", automation.Action
)
CameraRecorderDeleteDaysAction = camera_recorder_ns.class_(
    "CameraRecorderDeleteDaysAction", automation.Action
)


def _validate_directory(value):
    value = cv.string_strict(value)
    if not value.startswith("/") or ".." in value.split("/"):
        raise cv.Invalid("directory must be an absolute SD-card path without '..'")
    return value.rstrip("/") or "/"


def _validate_color(value):
    value = cv.string_strict(value)
    if re.fullmatch(r"#[0-9A-Fa-f]{6}", value) is None:
        raise cv.Invalid("colors must use quoted #RRGGBB format")
    return value


def _final_validate(_config):
    if CORE.target_framework != "esp-idf":
        raise cv.Invalid("camera_recorder requires the ESP-IDF framework")
    if get_esp32_variant() != VARIANT_ESP32S3:
        raise cv.Invalid("camera_recorder currently supports ESP32-S3 only")


CONFIG_SCHEMA = cv.All(
    cv.require_esphome_version(2026, 9, 0),
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CameraRecorder),
            cv.Required(CONF_CAMERA_ID): cv.use_id(esp32_camera.ESP32Camera),
            cv.Required(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.GenerateID(CONF_WEB_SERVER_BASE_ID): cv.use_id(
                web_server_base.WebServerBase
            ),
            cv.Required(CONF_CLK_PIN): pins.internal_gpio_output_pin_number,
            cv.Required(CONF_CMD_PIN): pins.internal_gpio_output_pin_number,
            cv.Required(CONF_DATA0_PIN): pins.internal_gpio_pin_number,
            cv.Optional(CONF_DIRECTORY, default="/recordings"): _validate_directory,
            cv.Optional(CONF_LIGHT_ACCENT, default="#0878ff"): _validate_color,
            cv.Optional(CONF_DARK_ACCENT, default="#65a8ff"): _validate_color,
            cv.Optional(CONF_LIGHT_ACTIVE, default="#dcecff"): _validate_color,
            cv.Optional(CONF_DARK_ACTIVE, default="#1b385c"): _validate_color,
            cv.Optional(CONF_FORMAT_IF_MOUNT_FAILED, default=False): cv.boolean,
            cv.Optional(CONF_MOUNTED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_RECORDING): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_RUNNING,
                icon="mdi:record-circle-outline",
            ),
            cv.Optional(
                CONF_CLASSIFYING, default={CONF_NAME: "Classifying"}
            ): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_RUNNING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(
                CONF_MODELS_READY, default={CONF_NAME: "Person Model Ready"}
            ): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(
                CONF_PERSON_DETECTED, default={CONF_NAME: "Person Detected"}
            ): binary_sensor.binary_sensor_schema(),
            cv.Optional(
                CONF_MOTION, default={CONF_NAME: "Motion"}
            ): binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_MOTION),
            cv.Optional(
                CONF_MOTION_SCORE, default={CONF_NAME: "Motion Score"}
            ): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                icon="mdi:animation",
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(
                CONF_SD_FREE, default={CONF_NAME: "SD Card Free"}
            ): sensor.sensor_schema(
                unit_of_measurement="GiB",
                icon="mdi:micro-sd",
                accuracy_decimals=2,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Required(CONF_WEB_USERNAME): cv.All(
                cv.string_strict, cv.Length(min=1)
            ),
            cv.Required(CONF_WEB_PASSWORD): cv.sensitive(
                cv.All(cv.string_strict, cv.Length(min=1))
            ),
            cv.Optional(
                CONF_MOTION_THRESHOLD,
                default={CONF_NAME: "Motion Threshold"},
            ): number.number_schema(
                CameraRecorderNumber,
                unit_of_measurement=UNIT_PERCENT,
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(
                CONF_MOTION_HOLD_TIME, default={CONF_NAME: "Motion Hold Time"}
            ): number.number_schema(
                CameraRecorderNumber,
                unit_of_measurement=UNIT_SECOND,
                device_class=DEVICE_CLASS_DURATION,
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(
                CONF_MOTION_FPS, default={CONF_NAME: "Motion Detection FPS"}
            ): number.number_schema(
                CameraRecorderNumber,
                unit_of_measurement="fps",
                icon="mdi:camera-burst",
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(
                CONF_MIN_PERSON_DETECTIONS,
                default={CONF_NAME: "Minimum Person Detections"},
            ): number.number_schema(
                CameraRecorderNumber,
                icon="mdi:account-group",
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(
                CONF_RECORD_PEOPLE, default={CONF_NAME: "Record People"}
            ): switch.switch_schema(
                CameraRecorderSwitch,
                entity_category=ENTITY_CATEGORY_CONFIG,
                default_restore_mode="RESTORE_DEFAULT_ON",
            ),
            cv.Optional(
                CONF_RECORD_OTHER_MOTION,
                default={CONF_NAME: "Record Other Motion"},
            ): switch.switch_schema(
                CameraRecorderSwitch,
                entity_category=ENTITY_CATEGORY_CONFIG,
                default_restore_mode="RESTORE_DEFAULT_ON",
            ),
            cv.Optional(
                CONF_AUTO_DELETION, default={CONF_NAME: "Auto Deletion"}
            ): switch.switch_schema(
                CameraRecorderSwitch,
                icon="mdi:delete-clock",
                entity_category=ENTITY_CATEGORY_CONFIG,
                default_restore_mode="RESTORE_DEFAULT_ON",
            ),
            cv.Optional(
                CONF_MOTION_DETECTION, default={CONF_NAME: "Motion Detection"}
            ): switch.switch_schema(
                CameraRecorderSwitch,
                entity_category=ENTITY_CATEGORY_CONFIG,
                default_restore_mode="RESTORE_DEFAULT_ON",
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)

FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    camera = await cg.get_variable(config[CONF_CAMERA_ID])
    cg.add(var.set_camera(camera))
    clock = await cg.get_variable(config[CONF_TIME_ID])
    cg.add(var.set_time(clock))
    web_server = await cg.get_variable(config[CONF_WEB_SERVER_BASE_ID])
    cg.add(var.set_web_server(web_server))
    cg.add_define("USE_WEBSERVER_AUTH")
    cg.add(web_server.set_auth_username(config[CONF_WEB_USERNAME]))
    cg.add(web_server.set_auth_password(config[CONF_WEB_PASSWORD]))
    cg.add(var.set_pins(config[CONF_CLK_PIN], config[CONF_CMD_PIN], config[CONF_DATA0_PIN]))
    cg.add(var.set_directory(config[CONF_DIRECTORY]))
    cg.add(var.set_light_accent(config[CONF_LIGHT_ACCENT]))
    cg.add(var.set_dark_accent(config[CONF_DARK_ACCENT]))
    cg.add(var.set_light_active(config[CONF_LIGHT_ACTIVE]))
    cg.add(var.set_dark_active(config[CONF_DARK_ACTIVE]))
    cg.add(var.set_format_if_mount_failed(config[CONF_FORMAT_IF_MOUNT_FAILED]))

    if mounted_config := config.get(CONF_MOUNTED):
        mounted = await binary_sensor.new_binary_sensor(mounted_config)
        cg.add(var.set_mounted_sensor(mounted))
    if recording_config := config.get(CONF_RECORDING):
        recording = await binary_sensor.new_binary_sensor(recording_config)
        cg.add(var.set_recording_sensor(recording))

    classifying = await binary_sensor.new_binary_sensor(config[CONF_CLASSIFYING])
    cg.add(var.set_classifying_sensor(classifying))
    models_ready = await binary_sensor.new_binary_sensor(config[CONF_MODELS_READY])
    cg.add(var.set_models_ready_sensor(models_ready))
    person_detected = await binary_sensor.new_binary_sensor(config[CONF_PERSON_DETECTED])
    cg.add(var.set_person_detected_sensor(person_detected))

    motion = await binary_sensor.new_binary_sensor(config[CONF_MOTION])
    cg.add(var.set_motion_sensor(motion))

    motion_score = await sensor.new_sensor(config[CONF_MOTION_SCORE])
    cg.add(var.set_motion_score_sensor(motion_score))

    sd_free = await sensor.new_sensor(config[CONF_SD_FREE])
    cg.add(var.set_sd_free_sensor(sd_free))

    threshold = await number.new_number(
        config[CONF_MOTION_THRESHOLD], min_value=0.1, max_value=100, step=0.1
    )
    cg.add(threshold.set_parent(var))
    cg.add(threshold.set_type(CameraRecorderNumberType.THRESHOLD))
    cg.add(var.set_threshold_number(threshold))

    hold_time = await number.new_number(
        config[CONF_MOTION_HOLD_TIME], min_value=1, max_value=300, step=1
    )
    cg.add(hold_time.set_parent(var))
    cg.add(hold_time.set_type(CameraRecorderNumberType.HOLD_TIME))
    cg.add(var.set_hold_time_number(hold_time))

    motion_fps = await number.new_number(
        config[CONF_MOTION_FPS], min_value=0.1, max_value=2.0, step=0.1
    )
    cg.add(motion_fps.set_parent(var))
    cg.add(motion_fps.set_type(CameraRecorderNumberType.MOTION_FPS))
    cg.add(var.set_motion_fps_number(motion_fps))

    min_person_detections = await number.new_number(
        config[CONF_MIN_PERSON_DETECTIONS], min_value=1, max_value=4, step=1
    )
    cg.add(min_person_detections.set_parent(var))
    cg.add(
        min_person_detections.set_type(
            CameraRecorderNumberType.MIN_PERSON_DETECTIONS
        )
    )
    cg.add(var.set_min_person_detections_number(min_person_detections))

    for key, switch_type in (
        (CONF_RECORD_PEOPLE, CameraRecorderSwitchType.PEOPLE),
        (CONF_RECORD_OTHER_MOTION, CameraRecorderSwitchType.OTHER_MOTION),
        (CONF_AUTO_DELETION, CameraRecorderSwitchType.AUTO_DELETION),
        (CONF_MOTION_DETECTION, CameraRecorderSwitchType.MOTION_DETECTION),
    ):
        record_switch = await switch.new_switch(config[key])
        cg.add(record_switch.set_parent(var))
        cg.add(record_switch.set_type(switch_type))
        cg.add(var.add_record_switch(record_switch))

    require_fatfs()
    require_vfs_dir()
    for component in ("fatfs", "sdmmc", "esp_driver_sdmmc"):
        include_builtin_idf_component(component)

    add_idf_component(name="espressif/pedestrian_detect", ref="0.3.2")
    for option in (
        "CONFIG_PEDESTRIAN_DETECT_MODEL_IN_SDCARD",
        "CONFIG_PEDESTRIAN_DETECT_PICO_S8_V1",
        "CONFIG_FATFS_LFN_HEAP",
    ):
        add_idf_sdkconfig_option(option, True)


ACTION_SCHEMA = automation.maybe_simple_id(
    {cv.GenerateID(): cv.use_id(CameraRecorder)}
)


@automation.register_action(
    "camera_recorder.start", CameraRecorderStartAction, ACTION_SCHEMA, synchronous=True
)
async def camera_recorder_start_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "camera_recorder.stop", CameraRecorderStopAction, ACTION_SCHEMA, synchronous=True
)
async def camera_recorder_stop_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "camera_recorder.stop_and_classify",
    CameraRecorderStopAndClassifyAction,
    ACTION_SCHEMA,
    synchronous=True,
)
async def camera_recorder_stop_and_classify_to_code(
    config, action_id, template_arg, args
):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "camera_recorder.delete_oldest_days",
    CameraRecorderDeleteDaysAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(CameraRecorder),
            cv.Required(CONF_DAYS): cv.int_range(min=1, max=255),
        }
    ),
    synchronous=True,
)
async def camera_recorder_delete_oldest_days_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_days(config[CONF_DAYS]))
    return var
