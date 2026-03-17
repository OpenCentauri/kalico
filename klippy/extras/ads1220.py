# Named ADS1220 sensor sections for load-cell fusion
#
# Copyright (C) 2026 Timo V
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from klippy.extras.load_cell import ads1220 as load_cell_ads1220


def load_config(config):
    return load_config_prefix(config)


def load_config_prefix(config):
    return load_cell_ads1220.ADS1220(config)