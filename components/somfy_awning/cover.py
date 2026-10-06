import esphome.codegen as cg
from esphome.components import cover, sensor
from esphome.components.somfy_rts import COMMANDS, SomfyRTSRemote
import esphome.config_validation as cv

CODEOWNERS = ["@d-asensio"]
DEPENDENCIES = ["somfy_rts"]
AUTO_LOAD = ["sensor"]

CONF_SOMFY_RTS_ID = "somfy_rts_id"
CONF_DISTANCE_SENSOR = "distance_sensor"
CONF_OPEN_COMMAND = "open_command"
CONF_CLOSED_DISTANCE = "closed_distance"
CONF_OPEN_DISTANCE = "open_distance"
CONF_STOP_LATENCY = "stop_latency"
CONF_POSITION_TOLERANCE = "position_tolerance"
CONF_MAX_TRAVEL_TIME = "max_travel_time"
CONF_MIN_TRAVEL = "min_travel"

somfy_awning_ns = cg.esphome_ns.namespace("somfy_awning")
SomfyAwning = somfy_awning_ns.class_("SomfyAwning", cover.Cover, cg.Component)


def _validate(config):
    closed = config.get(CONF_CLOSED_DISTANCE)
    opened = config.get(CONF_OPEN_DISTANCE)
    if (closed is None) != (opened is None):
        raise cv.Invalid(
            f"Set both {CONF_CLOSED_DISTANCE} and {CONF_OPEN_DISTANCE}, or neither"
        )
    if closed is not None and abs(closed - opened) < 20:
        raise cv.Invalid("closed_distance and open_distance must be at least 20cm apart")
    return config


def _distance_cm(value):
    # Accept plain numbers (cm) or values with a unit such as "2.4m".
    if isinstance(value, (int, float)):
        return float(value)
    return cv.distance(value) * 100.0


CONFIG_SCHEMA = cv.All(
    cover.cover_schema(SomfyAwning, device_class="awning")
    .extend(
        {
            cv.GenerateID(CONF_SOMFY_RTS_ID): cv.use_id(SomfyRTSRemote),
            cv.Required(CONF_DISTANCE_SENSOR): cv.use_id(sensor.Sensor),
            # Which button extends the awning. Most Somfy awning motors extend on DOWN.
            cv.Optional(CONF_OPEN_COMMAND, default="DOWN"): cv.one_of(
                "UP", "DOWN", upper=True
            ),
            # Starting calibration in cm; ignored once a calibration is stored.
            cv.Optional(CONF_CLOSED_DISTANCE): _distance_cm,
            cv.Optional(CONF_OPEN_DISTANCE): _distance_cm,
            # Time between deciding to stop and the awning standing still (radio
            # frame, motor run-down, sensor filtering). Learned after every move.
            cv.Optional(
                CONF_STOP_LATENCY, default="600ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_POSITION_TOLERANCE, default="2%"): cv.percentage,
            # Moves shorter than this are not attempted.
            cv.Optional(CONF_MIN_TRAVEL, default="3%"): cv.percentage,
            cv.Optional(
                CONF_MAX_TRAVEL_TIME, default="120s"
            ): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    _validate,
)


async def to_code(config):
    var = await cover.new_cover(config)
    await cg.register_component(var, config)

    remote = await cg.get_variable(config[CONF_SOMFY_RTS_ID])
    cg.add(var.set_remote(remote))
    sens = await cg.get_variable(config[CONF_DISTANCE_SENSOR])
    cg.add(var.set_distance_sensor(sens))

    open_cmd = config[CONF_OPEN_COMMAND]
    close_cmd = "UP" if open_cmd == "DOWN" else "DOWN"
    cg.add(var.set_commands(COMMANDS[open_cmd], COMMANDS[close_cmd]))
    if CONF_CLOSED_DISTANCE in config:
        cg.add(
            var.set_default_calibration(
                config[CONF_CLOSED_DISTANCE], config[CONF_OPEN_DISTANCE]
            )
        )
    cg.add(var.set_stop_latency(config[CONF_STOP_LATENCY].total_milliseconds / 1000.0))
    cg.add(var.set_position_tolerance(config[CONF_POSITION_TOLERANCE]))
    cg.add(var.set_min_travel(config[CONF_MIN_TRAVEL]))
    cg.add(var.set_max_travel_time(config[CONF_MAX_TRAVEL_TIME].total_milliseconds))
