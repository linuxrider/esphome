import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.core import CORE
from esphome.const import (
    CONF_BAUD_RATE,
    CONF_ID,
    CONF_NUMBER,
    CONF_RX_PIN,
    CONF_TX_PIN,
    CONF_PORT,
    CONF_DATA,
    CONF_RX_BUFFER_SIZE,
    CONF_INVERTED,
    CONF_INVERT,
    CONF_TRIGGER_ID,
    CONF_SEQUENCE,
    CONF_TIMEOUT,
    CONF_DEBUG,
    CONF_DIRECTION,
    CONF_AFTER,
    CONF_BYTES,
    CONF_DELIMITER,
    CONF_DUMMY_RECEIVER,
    CONF_DUMMY_RECEIVER_ID,
    CONF_LAMBDA,
    PLATFORM_HOST,
)
from esphome.components.esp32.const import (
    KEY_ESP32,
    KEY_VARIANT,
    VARIANT_ESP32S2,
    VARIANT_ESP32S3,
)

CODEOWNERS = ["@linuxrider"]
CONF_USB_VCP_ID = "usb_vcp_id"


def _validate_variant(value):
    variant = CORE.data[KEY_ESP32][KEY_VARIANT]
    if variant not in [VARIANT_ESP32S2, VARIANT_ESP32S3]:
        raise cv.Invalid(f"USB VCP is unsupported by ESP32 variant {variant}")
    return value


CONF_USB_VCP = "usb_vcp"

usb_vcp_ns = cg.esphome_ns.namespace(CONF_USB_VCP)
UsbVcp = usb_vcp_ns.class_("UsbVcp", cg.Component)

def validate_phy(value):
    value = pins.internal_gpio_input_pin_schema(value)
    if CORE.is_esp8266 and value[CONF_NUMBER] >= 16:
        raise cv.Invalid("Pins GPIO16 and GPIO17 cannot be used as RX pins on ESP8266.")
    return value

VCPParityOptions = usb_vcp_ns.enum("VCPParityOptions")
VCP_PARITY_OPTIONS = {
    "NONE": VCPParityOptions.VCP_CONFIG_PARITY_NONE,
    "EVEN": VCPParityOptions.VCP_CONFIG_PARITY_EVEN,
    "ODD": VCPParityOptions.VCP_CONFIG_PARITY_ODD,
}

VCP_HANDSHAKE_OPTIONS = {
    "NONE": VCPParityOptions.VCP_CONFIG_HANDSHAKE_NONE,
    "RTS/CTS": VCPParityOptions.VCP_CONFIG_HANDSHAKE_RTSCTS,
    "DTR/DSR": VCPParityOptions.VCP_CONFIG_HANDSHAKE_DTRDSR,
}
CONF_STOP_BITS = "stop_bits"
CONF_DATA_BITS = "data_bits"
CONF_PARITY = "parity"
CONF_PHY = "phy"
CONF_HANDSHAKE = "handshake"

VCPDirection = usb_vcp_ns.enum("VCPDirection")
VCP_DIRECTIONS = {
    "RX": VCPDirection.VCP_DIRECTION_RX,
    "TX": VCPDirection.VCP_DIRECTION_TX,
    "BOTH": VCPDirection.VCP_DIRECTION_BOTH,
}

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UsbVcp),
            cv.Required(CONF_BAUD_RATE): cv.int_range(min=1),
            cv.Optional(CONF_PHY): validate_phy,
            cv.Optional(CONF_RX_BUFFER_SIZE, default=256): cv.validate_bytes,
            cv.Optional(CONF_STOP_BITS, default=1): cv.one_of(1, 2, int=True),
            cv.Optional(CONF_DATA_BITS, default=8): cv.int_range(min=5, max=8),
            cv.Optional(CONF_PARITY, default="NONE"): cv.enum(
                VCP_PARITY_OPTIONS, upper=True
            ),
            cv.Optional(CONF_HANDSHAKE, default="NONE"): cv.enum(
                VCP_HANDSHAKE_OPTIONS, upper=True
            ),
            # cv.Optional(CONF_DEBUG): maybe_empty_debug,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    _validate_variant,
)


async def to_code(config):
    cg.add_global(usb_vcp_ns.using)
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_baud_rate(config[CONF_BAUD_RATE]))

    if CONF_PHY in config:
        tx_pin = await cg.gpio_pin_expression(config[CONF_TX_PIN])
        cg.add(var.set_phy(tx_pin))
    cg.add(var.set_rx_buffer_size(config[CONF_RX_BUFFER_SIZE]))
    cg.add(var.set_stop_bits(config[CONF_STOP_BITS]))
    cg.add(var.set_data_bits(config[CONF_DATA_BITS]))
    cg.add(var.set_parity(config[CONF_PARITY]))

    if CONF_DEBUG in config:
        await debug_to_code(config[CONF_DEBUG], var)
