import logging
from pprint import pformat

from .robot import Robot
from .config import RobotConfig

def make_robot_from_config(config: RobotConfig) -> Robot:
    if config.type == "unix_robot":
        from .unix_robot import UnixRobot

        return UnixRobot(config)
    elif config.type == "x1_robot":
        from .x1_robot import X1Robot

        return X1Robot(config)
    else:
        raise ValueError(config.type)