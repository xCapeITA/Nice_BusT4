import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover
from esphome.const import CONF_ADDRESS, CONF_ID, CONF_USE_ADDRESS

# Definizione del namespace e della classe C++ associata al template lupanosix_blu
bus_t4_ns = cg.esphome_ns.namespace('bus_t4')
NiceBusT4 = bus_t4_ns.class_('NiceBusT4', cover.Cover, cg.Component)

CONFIG_SCHEMA = (
    cover.cover_schema(NiceBusT4)
    .extend(
        {
            cv.Optional(CONF_ADDRESS): cv.hex_uint16_t,
            cv.Optional(CONF_USE_ADDRESS): cv.hex_uint16_t,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield cover.register_cover(var, config)

    # Scomposizione dell'indirizzo di destinazione (addr_to) nei due byte costituenti
    if CONF_ADDRESS in config:
        address = config[CONF_ADDRESS]
        addr_hi = (address >> 8) & 0xFF
        addr_lo = address & 0xFF
        cg.add(var.addr_to[0], addr_hi)
        cg.add(var.addr_to[1], addr_lo)

    # Scomposizione dell'indirizzo sorgente del gateway (addr_from) nei due byte costituenti
    if CONF_USE_ADDRESS in config:
        use_address = config[CONF_USE_ADDRESS]
        from_hi = (use_address >> 8) & 0xFF
        from_lo = use_address & 0xFF
        cg.add(var.addr_from[0], from_hi)
        cg.add(var.addr_from[1], from_lo)
