import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import mqtt
from esphome.const import CONF_ID

# Define the ESPHome namespace
AUTOLOAD = ["mqtt"]
DEPENDENCIES = ["mqtt"]
CODEOWNERS = ["@formatBCE"]

airbnk_gateway_ns = cg.esphome_ns.namespace("airbnk_gateway")
AirbnkGatewayClass = airbnk_gateway_ns.class_("AirbnkGateway", mqtt.MQTTComponent)

CONF_MAC_ADDRESS = "mac_address"
CONF_MQTT_TOPIC = "mqtt_topic"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(AirbnkGatewayClass),
        cv.Required(CONF_MAC_ADDRESS): cv.string,
        cv.Required(CONF_MQTT_TOPIC): cv.string,
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    #await cg.register_component(var, config)
    await mqtt.register_mqtt_component(var, config)

    cg.add(var.set_mac_address(config[CONF_MAC_ADDRESS]))
    cg.add(var.set_mqtt_topic(config[CONF_MQTT_TOPIC]))