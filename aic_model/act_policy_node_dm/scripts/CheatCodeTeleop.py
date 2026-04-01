
import numpy as np
import rclpy

from threading import Thread
from dataclasses import dataclass, field
from scipy.spatial.transform import Rotation as R
from lerobot.common.robot_devices.teleoperation.utils import Teleoperator, TeleoperatorConfig
from lerobot.common.exceptions import DeviceAlreadyConnectedError
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import Buffer, TransformListener

from .types import JointMotionUpdateActionDict, MotionUpdateActionDict

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

    def connect(self):
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
        #offset between gripper and plug tip
        offset = gripper_pos - plug_tip_pos

        #hover above the port before insertion
        hover_target = port_pos + np.array([0.0, 0.0, self.z_offset])  # hover above the port by z_offset

        #desired gripper position 
        desired_gripper_pos = hover_target + offset  # maintain the same offset from the plug tip while hovering

        #error between current gripper position and desired position
        error = desired_gripper_pos - gripper_pos

        if abs(error[2]) < 0.01 and self.stage == "approach":
            self.stage = "insert"

        if self.stage == "insert":
            self.z_offset = max(-0.015, self.z_offset - 0.0005)

        if self.z_offset == -0.015 and self.stage == "insert":
            return np.array([0.0, 0.0, 0.0])  # stop moving once fully inserted
        
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

        return np.concatenate([linear_velocity, angular_velocity])
    

    def disconnect(self):
        self._is_connected = False
        self._node.destroy_node()
        