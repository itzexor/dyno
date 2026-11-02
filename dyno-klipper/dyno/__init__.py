"""Dyno load-cell pressure-advance measurement for Kalico/Klipper.

Installed as klippy/extras/dyno; configured by a [dyno] section.
"""


def load_config(config):
    from .extra import load_config as load
    return load(config)
