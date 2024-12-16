import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from . import CONF_USB_VCP_ID, UsbVcp, CONF_USB_VCP

AUTO_LOAD = [CONF_USB_VCP]

CONF_RECEIVED = "received"

TYPES = [
    CONF_RECEIVED,
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_USB_VCP_ID): cv.use_id(UsbVcp),
        cv.Optional(CONF_RECEIVED): text_sensor.text_sensor_schema(),
    }
).extend(cv.COMPONENT_SCHEMA)


async def setup_conf(config, key, hub):
    if key in config:
        conf = config[key]
        var = await text_sensor.new_text_sensor(conf)
        cg.add(getattr(hub, f"set_{key}_text_sensor")(var))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_USB_VCP_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)