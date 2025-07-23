#!/usr/bin/env python

import logging
import time
from functools import cached_property
from typing import Any

from lerobot.common.cameras.utils import make_cameras_from_configs
from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from .config_unix_robot import UnixRobotConfig

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

logger = logging.getLogger(__name__)


class UnixRobot(Robot, Node):
    config_class = UnixRobotConfig
    name = "unix_robot"

    def __init__(self, config: UnixRobotConfig):
        Robot.__init__(self, config)
        Node.__init__(self, "unix_robot_node")
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)

        self._is_connected = False

        self.joint_states = None

        self.joint_actions = None

        self.motors = [
            "la0", "la1", "la2", "la3", "la4", "la5", "la6", "la7",
            "left_gripper",
            "ra0", "ra1", "ra2", "ra3", "ra4", "ra5", "ra6", "ra7",
            "right_gripper"
        ]

        self.action_publisher = None

        self.create_subscription(JointState, "unix/recorded_joint_states", self._joint_states_callback, 10)

        if not config.teleop:
            self.action_publisher = self.create_publisher(JointState, "unix/sent_actions", 10)

    def _joint_states_callback(self, msg: JointState):
        if not self._is_connected:
            return
        # logger.info(f"Received joint states: {msg.name} with positions {msg.position}")
        # logger.info(f"Received joint velocities: {msg.name} with velocities {msg.velocity}")
        
        self.joint_actions = {f"{name}.pos": position for name, position in zip(msg.name, msg.position)}
        self.joint_states = {f"{name}.pos": position for name, position in zip(msg.name, msg.velocity)}

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self._is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        for cam in self.cameras.values():
            cam.connect()

        self._is_connected = True
        logger.info(f"{self} connected.")

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        for cam in self.cameras.values():
            cam.disconnect()

        self._is_connected = False
        logger.info(f"{self} disconnected.")

    def get_observation(self) -> dict[str, Any]:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.joint_states is None:
            return {}

        obs_dict = self.joint_states.copy()

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict

    def get_action(self) -> dict[str, Any]:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.joint_actions is None:
            return {}

        action_dict = self.joint_actions.copy()

        return action_dict

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # 发布 JointState（除 gripper 外的关节动作）
        joint_state_msg = JointState()
        joint_state_msg.name = list(goal_pos.keys())
        # joint_state_msg.position = list(goal_pos.values())
        joint_state_msg.position = [float(val) for val in goal_pos.values()]
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        if self.action_publisher is not None:
            self.action_publisher.publish(joint_state_msg)


        # 返回合并动作
        result = {f"{motor}.pos": val for motor, val in goal_pos.items()}

        return result
