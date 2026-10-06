from esphome import automation
import esphome.codegen as cg
from esphome.components import remote_base
import esphome.config_validation as cv
from esphome.const import CONF_ADDRESS, CONF_COMMAND, CONF_ID, CONF_REPEAT

CODEOWNERS = ["@d-asensio"]
DEPENDENCIES = ["remote_transmitter"]
MULTI_CONF = True

CONF_ROLLING_CODE = "rolling_code"

somfy_rts_ns = cg.esphome_ns.namespace("somfy_rts")
SomfyRTSRemote = somfy_rts_ns.class_("SomfyRTSRemote", cg.Component)
SendAction = somfy_rts_ns.class_("SendAction", automation.Action)
Command = somfy_rts_ns.enum("Command", True)
COMMANDS = {
    "MY": Command.MY,
    "UP": Command.UP,
    "DOWN": Command.DOWN,
    "PROG": Command.PROG,
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SomfyRTSRemote),
        cv.GenerateID(remote_base.CONF_TRANSMITTER_ID): cv.use_id(
            remote_base.RemoteTransmitterBase
        ),
        # 24-bit address of this virtual remote. Pick a random value; every
        # remote paired to the motor needs a different one.
        cv.Required(CONF_ADDRESS): cv.hex_int_range(min=1, max=0xFFFFFF),
        # Only used when nothing is stored in flash yet, e.g. to take over a
        # remote that is already paired from ESPSomfy RTS.
        cv.Optional(CONF_ROLLING_CODE, default=1): cv.int_range(min=1, max=0xFFFF),
        # Extra frames after the first one, like holding a button briefly.
        cv.Optional(CONF_REPEAT, default=2): cv.int_range(min=0, max=20),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    transmitter = await cg.get_variable(config[remote_base.CONF_TRANSMITTER_ID])
    cg.add(var.set_transmitter(transmitter))
    cg.add(var.set_address(config[CONF_ADDRESS]))
    cg.add(var.set_initial_rolling_code(config[CONF_ROLLING_CODE]))
    cg.add(var.set_repeat(config[CONF_REPEAT]))


@automation.register_action(
    "somfy_rts.send",
    SendAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(SomfyRTSRemote),
            cv.Required(CONF_COMMAND): cv.enum(COMMANDS, upper=True),
            cv.Optional(CONF_REPEAT): cv.int_range(min=0, max=20),
        }
    ),
    synchronous=True,
)
async def send_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_command(config[CONF_COMMAND]))
    if CONF_REPEAT in config:
        cg.add(var.set_repeat(config[CONF_REPEAT]))
    return var
