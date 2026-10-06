"""Somfy RTS awning with closed-loop positioning from a TF-Luna lidar.

One `somfy_lidar:` block sets up everything: the CC1101 radio (ESPHome's own
cc1101 and remote_transmitter drivers), a virtual RTS remote, the TF-Luna on
I2C, the cover and its setup buttons and diagnostic sensors.
"""

from esphome import automation
import esphome.codegen as cg
from esphome.components import (
    button,
    cc1101,
    cover,
    i2c,
    remote_base,
    remote_transmitter,
    sensor,
)
import esphome.config_validation as cv
from esphome.const import (
    CONF_COMMAND,
    CONF_ID,
    CONF_OUTPUT_POWER,
    CONF_REPEAT,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_CENTIMETER,
    UNIT_SECOND,
)
from esphome.core import CORE

CODEOWNERS = ["@d-asensio"]
DEPENDENCIES = ["spi", "i2c"]
AUTO_LOAD = ["remote_base", "sensor", "button", "cover"]

CONF_REMOTE_ADDRESS = "remote_address"
CONF_ROLLING_CODE = "rolling_code"
CONF_CS_PIN = "cs_pin"
CONF_GDO0_PIN = "gdo0_pin"
CONF_LIDAR_ADDRESS = "lidar_address"
CONF_MIN_SIGNAL_STRENGTH = "min_signal_strength"
CONF_OPEN_COMMAND = "open_command"
CONF_CLOSED_DISTANCE = "closed_distance"
CONF_OPEN_DISTANCE = "open_distance"
CONF_STOP_LATENCY = "stop_latency"
CONF_POSITION_TOLERANCE = "position_tolerance"
CONF_MIN_TRAVEL = "min_travel"
CONF_MAX_TRAVEL_TIME = "max_travel_time"

# Internal sub-configurations generated from the options above.
_RADIO = "_radio"
_TRANSMITTER = "_transmitter"
_REMOTE = "_remote"
_LIDAR = "_lidar"
_COVER = "_cover"
_SENSORS = "_sensors"
_BUTTONS = "_buttons"

ns = cg.esphome_ns.namespace("somfy_lidar")
SomfyAwning = ns.class_("SomfyAwning", cover.Cover, cg.Component)
SomfyRTSRemote = ns.class_("SomfyRTSRemote", cg.Component)
TFLuna = ns.class_("TFLuna", cg.PollingComponent, i2c.I2CDevice)
AwningButton = ns.class_("AwningButton", button.Button)
ButtonAction = ns.enum("ButtonAction", True)
SendCommandAction = ns.class_("SendCommandAction", automation.Action)
Command = ns.enum("Command", True)
COMMANDS = {
    "MY": Command.MY,
    "UP": Command.UP,
    "DOWN": Command.DOWN,
    "PROG": Command.PROG,
}


def _distance_cm(value):
    # Plain numbers are cm; values with a unit such as "2.4m" are converted.
    if isinstance(value, (int, float)):
        return float(value)
    return cv.distance(value) * 100.0


def _validate_calibration(config):
    closed = config.get(CONF_CLOSED_DISTANCE)
    opened = config.get(CONF_OPEN_DISTANCE)
    if (closed is None) != (opened is None):
        raise cv.Invalid(
            f"Set both {CONF_CLOSED_DISTANCE} and {CONF_OPEN_DISTANCE}, or neither"
        )
    if closed is not None and abs(closed - opened) < 20:
        raise cv.Invalid(
            f"{CONF_CLOSED_DISTANCE} and {CONF_OPEN_DISTANCE} must be at least 20cm apart"
        )
    return config


# (key, schema, fixed entity config)
SENSORS = [
    (
        # Fast internal feed for the control loop.
        "distance_raw",
        sensor.sensor_schema(unit_of_measurement=UNIT_CENTIMETER, accuracy_decimals=1),
        {"name": "Raw distance", "internal": True},
    ),
    (
        "distance",
        sensor.sensor_schema(
            unit_of_measurement=UNIT_CENTIMETER,
            icon="mdi:arrow-expand-horizontal",
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_DISTANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        {"name": "Distance", "filters": [{"throttle_average": "5s"}]},
    ),
    (
        "signal_strength",
        sensor.sensor_schema(
            icon="mdi:signal",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        {"name": "Lidar signal strength", "filters": [{"throttle_average": "30s"}]},
    ),
    (
        "temperature",
        sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        {"name": "Lidar temperature", "filters": [{"throttle_average": "60s"}]},
    ),
    (
        "stop_latency",
        sensor.sensor_schema(
            unit_of_measurement=UNIT_SECOND,
            icon="mdi:timer-outline",
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_DURATION,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        {"name": "Stop latency"},
    ),
]

BUTTONS = [
    ("CALIBRATE", {"name": "Calibrate", "icon": "mdi:ruler"}),
    (
        "SET_CLOSED_HERE",
        {"name": "Set closed position here", "disabled_by_default": True},
    ),
    ("SET_OPEN_HERE", {"name": "Set open position here", "disabled_by_default": True}),
    # Pairing: hold PROG on a remote that already controls the awning until it
    # jogs, then press this within a few seconds. Pressing it again unpairs.
    (
        "PROG",
        {"name": "Send PROG", "icon": "mdi:remote", "disabled_by_default": True},
    ),
]


def _expand(config):
    """Builds the internal component configurations from the user options."""
    config = dict(config)
    config[_RADIO] = cc1101.CONFIG_SCHEMA(
        {
            CONF_CS_PIN: config[CONF_CS_PIN],
            "frequency": "433.42MHz",  # Somfy RTS, not the usual 433.92MHz
            "modulation_type": "ASK/OOK",
            CONF_OUTPUT_POWER: config[CONF_OUTPUT_POWER],
        }
    )
    config[_TRANSMITTER] = remote_transmitter.CONFIG_SCHEMA(
        {
            "pin": config[CONF_GDO0_PIN],
            "carrier_duty_percent": "100%",
            # somfy_rts switches the radio to TX around each blocking transmission.
            "non_blocking": False,
        }
    )
    config[_REMOTE] = cv.Schema(
        {cv.GenerateID(): cv.declare_id(SomfyRTSRemote)}
    ).extend(cv.COMPONENT_SCHEMA)({})
    config[_LIDAR] = (
        cv.Schema({cv.GenerateID(): cv.declare_id(TFLuna)})
        .extend(cv.polling_component_schema("100ms"))
        .extend(i2c.i2c_device_schema(0x10))
    )({"address": config[CONF_LIDAR_ADDRESS]})
    config[_COVER] = cover.cover_schema(SomfyAwning, device_class="awning")(
        {"name": "None"}  # the device name
    )
    config[_SENSORS] = {
        key: schema(dict(entity)) for key, schema, entity in SENSORS
    }
    config[_BUTTONS] = {
        action: button.button_schema(
            AwningButton, entity_category=ENTITY_CATEGORY_CONFIG
        )(dict(entity))
        for action, entity in BUTTONS
    }
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SomfyAwning),
            # 24-bit address of the ESP's virtual remote. Pick a random value;
            # it must differ from every other remote paired with the motor.
            cv.Required(CONF_REMOTE_ADDRESS): cv.hex_int_range(min=1, max=0xFFFFFF),
            # Only used while no rolling code is stored in flash yet, e.g. to
            # take over a remote that is already paired from ESPSomfy RTS.
            cv.Optional(CONF_ROLLING_CODE, default=1): cv.int_range(
                min=1, max=0xFFFF
            ),
            # Extra frames per command, like holding a button briefly.
            cv.Optional(CONF_REPEAT, default=2): cv.int_range(min=0, max=20),
            # CC1101 pins outside the SPI bus, which comes from the `spi:` block.
            cv.Required(CONF_CS_PIN): cv.valid,
            cv.Required(CONF_GDO0_PIN): cv.valid,
            cv.Optional(CONF_OUTPUT_POWER, default=10): cv.float_range(
                min=-30.0, max=11.0
            ),
            # TF-Luna on the `i2c:` bus (its pin 5 tied to GND).
            cv.Optional(CONF_LIDAR_ADDRESS, default=0x10): cv.i2c_address,
            # The datasheet treats readings below 100 as unreliable.
            cv.Optional(CONF_MIN_SIGNAL_STRENGTH, default=100): cv.int_range(
                min=0, max=65534
            ),
            # Which button extends the awning. Most Somfy awnings extend on DOWN.
            cv.Optional(CONF_OPEN_COMMAND, default="DOWN"): cv.one_of(
                "UP", "DOWN", upper=True
            ),
            # Starting calibration in cm; ignored once a calibration is stored.
            cv.Optional(CONF_CLOSED_DISTANCE): _distance_cm,
            cv.Optional(CONF_OPEN_DISTANCE): _distance_cm,
            # Time from MY to standstill; learned after every positioning move.
            cv.Optional(
                CONF_STOP_LATENCY, default="600ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_POSITION_TOLERANCE, default="2%"): cv.percentage,
            cv.Optional(CONF_MIN_TRAVEL, default="3%"): cv.percentage,
            cv.Optional(
                CONF_MAX_TRAVEL_TIME, default="120s"
            ): cv.positive_time_period_milliseconds,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_calibration,
    _expand,
)


async def to_code(config):
    # The radio drivers are configured here rather than by the user, so make
    # sure their sources are part of the build as if they had their own block.
    for domain in ("cc1101", "remote_transmitter"):
        CORE.config.setdefault(domain, [])

    await cc1101.to_code(config[_RADIO])
    radio = await cg.get_variable(config[_RADIO][CONF_ID])
    await remote_transmitter.to_code(config[_TRANSMITTER])
    transmitter = await cg.get_variable(config[_TRANSMITTER][CONF_ID])

    remote = cg.new_Pvariable(config[_REMOTE][CONF_ID])
    await cg.register_component(remote, config[_REMOTE])
    cg.add(remote.set_transmitter(transmitter))
    cg.add(remote.set_radio(radio))
    cg.add(remote.set_address(config[CONF_REMOTE_ADDRESS]))
    cg.add(remote.set_initial_rolling_code(config[CONF_ROLLING_CODE]))
    cg.add(remote.set_repeat(config[CONF_REPEAT]))

    sensors = {}
    for key, _, _ in SENSORS:
        sensors[key] = await sensor.new_sensor(config[_SENSORS][key])

    lidar = cg.new_Pvariable(config[_LIDAR][CONF_ID])
    await cg.register_component(lidar, config[_LIDAR])
    await i2c.register_i2c_device(lidar, config[_LIDAR])
    cg.add(lidar.set_distance_sensor(sensors["distance_raw"]))
    cg.add(lidar.set_signal_strength_sensor(sensors["signal_strength"]))
    cg.add(lidar.set_temperature_sensor(sensors["temperature"]))
    cg.add(lidar.set_min_signal_strength(config[CONF_MIN_SIGNAL_STRENGTH]))

    awning = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(awning, config)
    await cover.register_cover(awning, config[_COVER])
    cg.add(awning.set_remote(remote))
    cg.add(awning.set_distance_sensor(sensors["distance_raw"]))
    cg.add(awning.set_reported_distance_sensor(sensors["distance"]))
    cg.add(awning.set_stop_latency_sensor(sensors["stop_latency"]))
    open_cmd = config[CONF_OPEN_COMMAND]
    close_cmd = "UP" if open_cmd == "DOWN" else "DOWN"
    cg.add(awning.set_commands(COMMANDS[open_cmd], COMMANDS[close_cmd]))
    if CONF_CLOSED_DISTANCE in config:
        cg.add(
            awning.set_default_calibration(
                config[CONF_CLOSED_DISTANCE], config[CONF_OPEN_DISTANCE]
            )
        )
    cg.add(awning.set_stop_latency(config[CONF_STOP_LATENCY].total_milliseconds / 1000.0))
    cg.add(awning.set_position_tolerance(config[CONF_POSITION_TOLERANCE]))
    cg.add(awning.set_min_travel(config[CONF_MIN_TRAVEL]))
    cg.add(awning.set_max_travel_time(config[CONF_MAX_TRAVEL_TIME].total_milliseconds))

    for action, conf in config[_BUTTONS].items():
        btn = await button.new_button(conf)
        cg.add(btn.set_parent(awning))
        cg.add(btn.set_action(getattr(ButtonAction, action)))


@automation.register_action(
    "somfy_lidar.send_command",
    SendCommandAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(SomfyAwning),
            cv.Required(CONF_COMMAND): cv.enum(COMMANDS, upper=True),
            cv.Optional(CONF_REPEAT, default=2): cv.int_range(min=0, max=20),
        }
    ),
    synchronous=True,
)
async def send_command_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_command(config[CONF_COMMAND]))
    cg.add(var.set_repeat(config[CONF_REPEAT]))
    return var
