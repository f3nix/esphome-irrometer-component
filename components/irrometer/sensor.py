import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_PRESSURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_OHM,
    UNIT_EMPTY,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

# Define your own constants for custom configuration keys.
# It's good practice to prefix them with your component name.
CONF_RESISTANCE = "resistance"
CONF_RESISTANCE_A = "resistance_a"
CONF_RESISTANCE_B = "resistance_b"
CONF_SOIL_WATER_TENSION = "soil_water_tension"
CONF_SOIL_TEMPERATURE = "soil_temperature"
CONF_WM = "watermarks"

# Define a namespace for the C++ code
irrometer_ns = cg.esphome_ns.namespace("irrometer")
IrrometerComponent = irrometer_ns.class_("Irrometer", cg.PollingComponent)

# Define the configuration schema for each watermark sensor
WM_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_SOIL_WATER_TENSION): sensor.sensor_schema(
            unit_of_measurement="cbar",
            device_class=DEVICE_CLASS_PRESSURE,
            state_class=STATE_CLASS_MEASUREMENT,
            accuracy_decimals=0,
        ),
        cv.Optional(CONF_SOIL_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
            accuracy_decimals=1,
        ),
        cv.Optional(CONF_RESISTANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_OHM,
            icon="mdi:omega",
            state_class=STATE_CLASS_MEASUREMENT,
            accuracy_decimals=1,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_RESISTANCE_A): sensor.sensor_schema(
            unit_of_measurement=UNIT_OHM,
            icon="mdi:omega",
            state_class=STATE_CLASS_MEASUREMENT,
            accuracy_decimals=1,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_RESISTANCE_B): sensor.sensor_schema(
            unit_of_measurement=UNIT_OHM,
            icon="mdi:omega",
            state_class=STATE_CLASS_MEASUREMENT,
            accuracy_decimals=1,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)

# Define the main configuration schema for the irrometer component
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(IrrometerComponent),
            cv.Optional(CONF_WM): cv.ensure_list(WM_SCHEMA),
        }
    )
    .extend(cv.polling_component_schema("15s")) # Default update interval
)


async def to_code(config):
    """Generate the C++ code for this component."""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_WM in config:
        for i, wm_config in enumerate(config[CONF_WM]):
            # Assign a C++ variable for each watermark sensor group
            cg.add(var.set_wm_index(i))

            if CONF_SOIL_WATER_TENSION in wm_config:
                sens = await sensor.new_sensor(wm_config[CONF_SOIL_WATER_TENSION])
                cg.add(var.set_tension_sensor(i, sens))

            if CONF_SOIL_TEMPERATURE in wm_config:
                sens = await sensor.new_sensor(wm_config[CONF_SOIL_TEMPERATURE])
                cg.add(var.set_temperature_sensor(i, sens))

            if CONF_RESISTANCE in wm_config:
                sens = await sensor.new_sensor(wm_config[CONF_RESISTANCE])
                cg.add(var.set_resistance_sensor(i, sens))

            if CONF_RESISTANCE_A in wm_config:
                sens = await sensor.new_sensor(wm_config[CONF_RESISTANCE_A])
                cg.add(var.set_resistance_a_sensor(i, sens))

            if CONF_RESISTANCE_B in wm_config:
                sens = await sensor.new_sensor(wm_config[CONF_RESISTANCE_B])
                cg.add(var.set_resistance_b_sensor(i, sens))
