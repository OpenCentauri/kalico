# Named HX71x sensor sections for load-cell fusion
#
# Copyright (C) 2026 Timo V
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from klippy.extras.load_cell import hx71x as load_cell_hx71x


SENSOR_TYPES = {
    "hx711": load_cell_hx71x.HX711,
    "hx717": load_cell_hx71x.HX717,
}


def load_config(config):
    return load_config_prefix(config)


def load_config_prefix(config):
    sensor_class = config.getchoice("chip", SENSOR_TYPES)
    return sensor_class(config)