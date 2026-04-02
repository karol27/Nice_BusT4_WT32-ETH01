import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover
from esphome.const import CONF_ADDRESS, CONF_ID, CONF_UPDATE_INTERVAL, CONF_USE_ADDRESS

CONF_TX_PIN  = 'tx_pin'
CONF_RX_PIN  = 'rx_pin'
CONF_UART_NUM = 'uart_num'

bus_t4_ns = cg.esphome_ns.namespace('bus_t4')
Nice = bus_t4_ns.class_('NiceBusT4', cover.Cover, cg.Component)

CONFIG_SCHEMA = cover.cover_schema(Nice).extend({
    cv.GenerateID(): cv.declare_id(Nice),
    cv.Optional(CONF_ADDRESS): cv.hex_uint16_t,
    cv.Optional(CONF_USE_ADDRESS): cv.hex_uint16_t,
    # UART hardware configuration — override for ESP32-C3 (default matches original WT32-ETH01 wiring)
    cv.Optional(CONF_TX_PIN,  default=17): cv.int_,
    cv.Optional(CONF_RX_PIN,  default=5):  cv.int_,
    cv.Optional(CONF_UART_NUM, default=1): cv.int_range(min=0, max=2),
#    cv.Optional(CONF_UPDATE_INTERVAL): cv.positive_time_period_milliseconds,
}).extend(cv.COMPONENT_SCHEMA)


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)

    yield cover.register_cover(var, config)

    if CONF_ADDRESS in config:
        address = config[CONF_ADDRESS]
        cg.add(var.set_to_address(address))

    if CONF_USE_ADDRESS in config:
        use_address = config[CONF_USE_ADDRESS]
        cg.add(var.set_from_address(use_address))

    cg.add(var.set_uart_nr(config[CONF_UART_NUM]))
    cg.add(var.set_tx_pin(config[CONF_TX_PIN]))
    cg.add(var.set_rx_pin(config[CONF_RX_PIN]))

 #   if CONF_UPDATE_INTERVAL in config:
 #       update_interval = config[CONF_UPDATE_INTERVAL]
 #       cg.add(var.set_update_interval(update_interval))
