# Load cell module setup
#
# Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from klippy.configfile import ConfigWrapper
from klippy.printer import Printer, SubsystemComponentCollection

<<<<<<< HEAD
from . import ads131m0x, ads1220, hx71x
=======
from . import ads1220, fusion, hx71x
>>>>>>> 5f9fabbd ([load_cell] Implement a generic fusion mechanism)
from .load_cell import LoadCell


# register sensors that implement BulkAdcSensor
def register_components(subsystem: SubsystemComponentCollection):
<<<<<<< HEAD
    sensors = (
        hx71x.HX71X_SENSOR_TYPES
        | ads1220.ADS1220_SENSOR_TYPE
        | ads131m0x.ADS131M0X_SENSOR_TYPES
    )
=======
    sensors = (hx71x.HX71X_SENSOR_TYPES | ads1220.ADS1220_SENSOR_TYPE
               | fusion.LOAD_CELL_FUSION_SENSOR_TYPE)
>>>>>>> 5f9fabbd ([load_cell] Implement a generic fusion mechanism)
    for name, sensor in sensors.items():
        subsystem.register_component("load_cell_sensors", name, sensor)


def load_config(config: ConfigWrapper):
    printer: Printer = config.get_printer()
    sensors = printer.lookup_components("load_cell_sensors")
    sensor_class = config.getchoice("sensor_type", sensors)
    return LoadCell(config, sensor_class(config))


def load_config_prefix(config):
    return load_config(config)
