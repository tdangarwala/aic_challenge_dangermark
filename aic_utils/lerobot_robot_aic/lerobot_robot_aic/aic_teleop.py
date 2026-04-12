#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

from dataclasses import dataclass, field
from threading import Thread
from typing import Any, cast

import pyspacemouse
import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig
from lerobot.teleoperators.keyboard import (
    KeyboardEndEffectorTeleop,
    KeyboardEndEffectorTeleopConfig,
)
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot_teleoperator_devices import KeyboardJointTeleop, KeyboardJointTeleopConfig
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import Buffer, TransformListener

from .aic_robot import arm_joint_names
from .types import JointMotionUpdateActionDict, MotionUpdateActionDict


@TeleoperatorConfig.register_subclass("aic_keyboard_joint")
@dataclass
class AICKeyboardJointTeleopConfig(KeyboardJointTeleopConfig):
    arm_action_keys: list[str] = field(
        default_factory=lambda: [f"{x}" for x in arm_joint_names]
    )
    high_command_scaling: float = 0.05
    low_command_scaling: float = 0.02


class AICKeyboardJointTeleop(KeyboardJointTeleop):
    def __init__(self, config: AICKeyboardJointTeleopConfig):
        super().__init__(config)

        self.config = config
        self._low_scaling = config.low_command_scaling
        self._high_scaling = config.high_command_scaling
        self._current_scaling = self._high_scaling

        self.curr_joint_actions: JointMotionUpdateActionDict = {
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": 0.0,
            "elbow_joint": 0.0,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
        }

    @property
    def action_features(self) -> dict:
        return {"names": JointMotionUpdateActionDict.__annotations__}

    def _get_action_value(self, is_pressed: bool) -> float:
        return self._current_scaling if is_pressed else 0.0

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        self._drain_pressed_keys()

        for key, is_pressed in self.current_pressed.items():

            if key == "u" and is_pressed:
                is_low_scaling = self._current_scaling == self._low_scaling
                self._current_scaling = (
                    self._high_scaling if is_low_scaling else self._low_scaling
                )
                print(f"Command scaling toggled to: {self._current_scaling}")
                continue

            val = self._get_action_value(is_pressed)

            if key == "q":
                self.curr_joint_actions["shoulder_pan_joint"] = val
            elif key == "a":
                self.curr_joint_actions["shoulder_pan_joint"] = -val
            elif key == "w":
                self.curr_joint_actions["shoulder_lift_joint"] = val
            elif key == "s":
                self.curr_joint_actions["shoulder_lift_joint"] = -val
            elif key == "e":
                self.curr_joint_actions["elbow_joint"] = val
            elif key == "d":
                self.curr_joint_actions["elbow_joint"] = -val
            elif key == "r":
                self.curr_joint_actions["wrist_1_joint"] = val
            elif key == "f":
                self.curr_joint_actions["wrist_1_joint"] = -val
            elif key == "t":
                self.curr_joint_actions["wrist_2_joint"] = val
            elif key == "g":
                self.curr_joint_actions["wrist_2_joint"] = -val
            elif key == "y":
                self.curr_joint_actions["wrist_3_joint"] = val
            elif key == "h":
                self.curr_joint_actions["wrist_3_joint"] = -val
            elif is_pressed:
                # If the key is pressed, add it to the misc_keys_queue
                # this will record key presses that are not part of the delta_x, delta_y, delta_z
                # this is useful for retrieving other events like interventions for RL, episode success, etc.
                self.misc_keys_queue.put(key)

        self.current_pressed.clear()

        return cast(dict, self.curr_joint_actions)


@TeleoperatorConfig.register_subclass("aic_keyboard_ee")
@dataclass(kw_only=True)
class AICKeyboardEETeleopConfig(KeyboardEndEffectorTeleopConfig):
    high_command_scaling: float = 0.1
    low_command_scaling: float = 0.02


class AICKeyboardEETeleop(KeyboardEndEffectorTeleop):
    def __init__(self, config: AICKeyboardEETeleopConfig):
        super().__init__(config)
        self.config = config

        self._high_scaling = config.high_command_scaling
        self._low_scaling = config.low_command_scaling
        self._current_scaling = self._high_scaling

        self._current_actions: MotionUpdateActionDict = {
            "linear.x": 0.0,
            "linear.y": 0.0,
            "linear.z": 0.0,
            "angular.x": 0.0,
            "angular.y": 0.0,
            "angular.z": 0.0,
        }

    @property
    def action_features(self) -> dict:
        return MotionUpdateActionDict.__annotations__

    def _get_action_value(self, is_pressed: bool) -> float:
        return self._current_scaling if is_pressed else 0.0

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        self._drain_pressed_keys()

        for key, is_pressed in self.current_pressed.items():

            if key == "t" and is_pressed:
                is_low_speed = self._current_scaling == self._low_scaling
                self._current_scaling = (
                    self._high_scaling if is_low_speed else self._low_scaling
                )
                print(f"Command scaling toggled to: {self._current_scaling}")
                continue

            val = self._get_action_value(is_pressed)

            if key == "w":
                self._current_actions["linear.y"] = -val
            elif key == "s":
                self._current_actions["linear.y"] = val
            elif key == "a":
                self._current_actions["linear.x"] = -val
            elif key == "d":
                self._current_actions["linear.x"] = val
            elif key == "r":
                self._current_actions["linear.z"] = -val
            elif key == "f":
                self._current_actions["linear.z"] = val
            elif key == "W":
                self._current_actions["angular.x"] = val
            elif key == "S":
                self._current_actions["angular.x"] = -val
            elif key == "A":
                self._current_actions["angular.y"] = -val
            elif key == "D":
                self._current_actions["angular.y"] = val
            elif key == "q":
                self._current_actions["angular.z"] = -val
            elif key == "e":
                self._current_actions["angular.z"] = val
            elif is_pressed:
                # If the key is pressed, add it to the misc_keys_queue
                # this will record key presses that are not part of the delta_x, delta_y, delta_z
                # this is useful for retrieving other events like interventions for RL, episode success, etc.
                self.misc_keys_queue.put(key)

        self.current_pressed.clear()

        return cast(dict, self._current_actions)


@TeleoperatorConfig.register_subclass("aic_spacemouse")
@dataclass(kw_only=True)
class AICSpaceMouseTeleopConfig(TeleoperatorConfig):
    operator_position_front: bool = True
    device: str | None = None  # only needed for multiple space mice
    command_scaling: float = 0.1


class AICSpaceMouseTeleop(Teleoperator):
    def __init__(self, config: AICSpaceMouseTeleopConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._device: pyspacemouse.SpaceMouseDevice | None = None

        self._current_actions: MotionUpdateActionDict = {
            "linear.x": 0.0,
            "linear.y": 0.0,
            "linear.z": 0.0,
            "angular.x": 0.0,
            "angular.y": 0.0,
            "angular.z": 0.0,
        }

    @property
    def name(self) -> str:
        return "aic_spacemouse"

    @property
    def action_features(self) -> dict:
        return MotionUpdateActionDict.__annotations__

    @property
    def feedback_features(self) -> dict:
        # TODO
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError()

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node("spacemouse_teleop")
        if calibrate:
            self._node.get_logger().warn(
                "Calibration not supported, ensure the robot is calibrated before running teleop."
            )

        self._device = pyspacemouse.open(
            dof_callback=None,
            # button_callback_arr=[
            #     pyspacemouse.ButtonCallback([0], self._button_callback),  # Button 1
            #     pyspacemouse.ButtonCallback([1], self._button_callback),  # Button 2
            # ],
            device=self.config.device,
        )

        if self._device is None:
            raise RuntimeError("Failed to open SpaceMouse device")

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = Thread(target=self._executor.spin)
        self._executor_thread.start()
        self._is_connected = True

    @property
    def is_calibrated(self) -> bool:
        # Calibration not supported
        return True

    def calibrate(self) -> None:
        # Calibration not supported
        pass

    def configure(self) -> None:
        pass

    def apply_deadband(self, value, threshold=0.02):
        return value if abs(value) > threshold else 0.0

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected or not self._device:
            raise DeviceNotConnectedError()

        state = self._device.read()

        clean_x = self.apply_deadband(float(state.x))
        clean_y = self.apply_deadband(float(state.y))
        clean_z = self.apply_deadband(float(state.z))
        clean_roll = self.apply_deadband(float(state.roll))
        clean_pitch = self.apply_deadband(float(state.pitch))
        clean_yaw = self.apply_deadband(float(state.yaw))

        twist_msg = Twist()
        twist_msg.linear.x = clean_x**1 * self.config.command_scaling
        twist_msg.linear.y = -(clean_y**1) * self.config.command_scaling
        twist_msg.linear.z = -(clean_z**1) * self.config.command_scaling
        twist_msg.angular.x = -(clean_pitch**1) * self.config.command_scaling
        twist_msg.angular.y = clean_roll**1 * self.config.command_scaling  #
        twist_msg.angular.z = clean_yaw**1 * self.config.command_scaling

        if not self.config.operator_position_front:
            twist_msg.linear.x *= -1
            twist_msg.linear.y *= -1
            twist_msg.angular.x *= -1
            twist_msg.angular.y *= -1

        self._current_actions = {
            "linear.x": twist_msg.linear.x,
            "linear.y": twist_msg.linear.y,
            "linear.z": twist_msg.linear.z,
            "angular.x": twist_msg.angular.x,
            "angular.y": twist_msg.angular.y,
            "angular.z": twist_msg.angular.z,
        }

        return cast(dict, self._current_actions)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        if self._device:
            self._device.close()
        self._is_connected = False
        pass


@TeleoperatorConfig.register_subclass("cheatcodeteleop")
@dataclass(kw_only=True)
class CheatCodeTeleopConfig(TeleoperatorConfig):
    cable_name: str = "cable_0"
    plug_name: str = "sfp_tip"
    module_name: str = "nic_card_mount_0"
    port_name: str = "sfp_port_0"

    kp: float = 1.0
    ki: float = 0.15
    max_windup: float = 1.0
    kp_ang: float = 1.5


class CheatCodeTeleop(Teleoperator):
    def __init__(self, config: CheatCodeTeleopConfig):
        super().__init__(config)
        self.config = config

        self.integrator = np.zeros(3)
        self.stage = "approach"
        self.z_offset = 0.2

        self._is_connected = False

    @property
    def name(self):
        return "cheatcodeteleop"
    
    @property
    def action_features(self):
        return MotionUpdateActionDict.__annotations__
    
    @property
    def feedback_features(self):
        return {}
    
    @property
    def is_connected(self):
        return self._is_connected
    
    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def send_feedback(self, feedback) -> None:
        pass

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise DeviceAlreadyConnectedError()

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node("cheatcodeteleop")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._is_connected = True



    # for calculate_velocity and calculate_angular_velocity
    # Lerobot train effectively samples the cheatcode policy path
    # when it samples (get_action) we need to provide x,y,z linear vel and x,y,z angular vel to store for training
    # velocity is better for learning because ....
    # 
    def calculate_velocity(self, gripper_pos, plug_tip_pos, port_pos):
        xy_error = np.linalg.norm(plug_tip_pos[:2] - port_pos[:2])
        z_error = abs(plug_tip_pos[2] - (port_pos[2] + self.z_offset))
        if xy_error < 0.005 and z_error < 0.01 and self.stage == "approach":
            self.stage = "insert"

        if self.stage == "insert":
            self.z_offset = max(-0.015, self.z_offset - 0.0005)

        if self.z_offset <= -0.015 and self.stage == "insert":
            return np.array([0.0, 0.0, 0.0])  # stop moving once fully inserted
        
        #offset between gripper and plug tip
        offset = gripper_pos - plug_tip_pos

        #hover above the port before insertion
        hover_target = port_pos + np.array([0.0, 0.0, self.z_offset])  # hover above the port by z_offset

        #desired gripper position 
        desired_gripper_pos = hover_target + offset  # maintain the same offset from the plug tip while hovering

        #error between current gripper position and desired position
        error = desired_gripper_pos - gripper_pos
        
        #add error to integrator for integral term
        self.integrator = np.clip(self.integrator + error, -self.config.max_windup, self.config.max_windup)  # limit integrator to prevent windup

        #velocity command proportional to error
        velocity_command = self.config.kp * error + self.config.ki * self.integrator

        return velocity_command
    
    def calculate_angular_velocity(self, q_port, q_plug_tip, q_gripper):
        # how much to rotate plug to match port
        r_port = R.from_quat(q_port) # convert to scipy Rotation format
        r_plug_tip = R.from_quat(q_plug_tip)
        r_gripper = R.from_quat(q_gripper)

        r_correction = r_port * r_plug_tip.inv()  # desired plug tip orientation to match port

        r_desired_gripper = r_correction * r_gripper  # desired gripper orientation to maintain plug tip orientation
        
        r_error = r_desired_gripper * r_gripper.inv()  # error between current gripper orientation and desired orientation

        angular_velocity = self.config.kp_ang * r_error.as_rotvec()  # proportional control on orientation error
        
        return angular_velocity
    
    #requests RO
    def _lookup_tf(self, target_frame, source_frame):
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except Exception:
            return None

    def reset(self):
        self.integrator = np.zeros(3)
        self.stage = "approach"
        self.z_offset = 0.2

    def get_action(self):
        # is called by Lerobot at each timestep
        # should return a 6D velocity command for the gripper (3D linear + 3D angular)
        gripper_tf = self._lookup_tf("base_link", "gripper/tcp")
        plug_tip_tf = self._lookup_tf("base_link", f"{self.config.cable_name}/{self.config.plug_name}_link")
        port_tf = self._lookup_tf("base_link", f"task_board/{self.config.module_name}/{self.config.port_name}_link")

        if gripper_tf is None or plug_tip_tf is None or port_tf is None:
            return {"linear.x": 0.0, "linear.y": 0.0, "linear.z": 0.0,
                "angular.x": 0.0, "angular.y": 0.0, "angular.z": 0.0}

        gripper_pos = np.array([
            gripper_tf.transform.translation.x,
            gripper_tf.transform.translation.y,
            gripper_tf.transform.translation.z
        ])

        plug_tip_pos = np.array([
            plug_tip_tf.transform.translation.x,
            plug_tip_tf.transform.translation.y,
            plug_tip_tf.transform.translation.z
        ])

        port_pos = np.array([
            port_tf.transform.translation.x,
            port_tf.transform.translation.y,
            port_tf.transform.translation.z
        ])

        gripper_q = np.array([
            gripper_tf.transform.rotation.x,
            gripper_tf.transform.rotation.y,
            gripper_tf.transform.rotation.z,
            gripper_tf.transform.rotation.w
        ])

        plug_tip_q = np.array([
            plug_tip_tf.transform.rotation.x,
            plug_tip_tf.transform.rotation.y,
            plug_tip_tf.transform.rotation.z,
            plug_tip_tf.transform.rotation.w
        ])

        port_q = np.array([
            port_tf.transform.rotation.x,
            port_tf.transform.rotation.y,
            port_tf.transform.rotation.z,
            port_tf.transform.rotation.w
        ])

        # get positions and orientations from TF

        # call calculate_velocity and calculate_angular_velocity to get linear and angular velocity commands
        linear_velocity = self.calculate_velocity(gripper_pos, plug_tip_pos, port_pos)
        # combine linear and angular velocity into a single 6D command
        angular_velocity = self.calculate_angular_velocity(port_q, plug_tip_q, gripper_q)

        action_vector = np.concatenate([linear_velocity, angular_velocity])
        return {
            "linear.x": float(action_vector[0]),
            "linear.y": float(action_vector[1]),
            "linear.z": float(action_vector[2]),
            "angular.x": float(action_vector[3]),
            "angular.y": float(action_vector[4]),
            "angular.z": float(action_vector[5]),
        }

    def disconnect(self):
        self._executor.shutdown()
        self._spin_thread.join(timeout=2.0)
        self._node.destroy_node()
        self._is_connected = False
        
