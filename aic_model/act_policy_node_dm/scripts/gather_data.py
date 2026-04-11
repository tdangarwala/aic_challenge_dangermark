
import subprocess
import time
import os

import random
import yaml

from pathlib import Path


PROGRESS_FILE = Path("collection_progress.txt")

NIC_RAIL_LIMITS    = (-0.0215, 0.0234)
NIC_YAW_LIMITS     = (-0.1745, 0.1745)  # -10 to +10 degrees in radians
SC_RAIL_LIMITS     = (-0.06, 0.055)
BOARD_X_LIMITS     = (0.13, 0.20)
BOARD_Y_LIMITS     = (-0.25, 0.25)
BOARD_YAW_LIMITS   = (2.8, 3.4)

TRIALS = [
    {"name": "sfp_port0", "type": "sfp", "target_port": "SFP_PORT_0", "num_episodes": 1},
    {"name": "sfp_port1", "type": "sfp", "target_port": "SFP_PORT_1", "num_episodes": 1},
    {"name": "sc_port0",  "type": "sc",  "target_port": "SC_PORT_0",  "num_episodes": 1},
    {"name": "sc_port1",  "type": "sc",  "target_port": "SC_PORT_1",  "num_episodes": 1},
]

SCORING_CONFIG = {
    "scoring": {
        "topics": [
            {"topic": {"name": "/joint_states", "type": "sensor_msgs/msg/JointState"}},
            {"topic": {"name": "/tf", "type": "tf2_msgs/msg/TFMessage"}},
            {"topic": {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "latched": True}},
            {"topic": {"name": "/scoring/tf", "type": "tf2_msgs/msg/TFMessage"}},
            {"topic": {"name": "/aic/gazebo/contacts/off_limit", "type": "ros_gz_interfaces/msg/Contacts"}},
            {"topic": {"name": "/fts_broadcaster/wrench", "type": "geometry_msgs/msg/WrenchStamped"}},
            {"topic": {"name": "/aic_controller/joint_commands", "type": "aic_control_interfaces/msg/JointMotionUpdate"}},
            {"topic": {"name": "/aic_controller/pose_commands", "type": "aic_control_interfaces/msg/MotionUpdate"}},
            {"topic": {"name": "/scoring/insertion_event", "type": "std_msgs/msg/String"}},
            {"topic": {"name": "/aic_controller/controller_state", "type": "aic_control_interfaces/msg/ControllerState"}},
        ]
    }
}

TASK_BOARD_LIMITS = {
    "task_board_limits": {
        "nic_rail":   {"min_translation": -0.0215, "max_translation": 0.0234},
        "sc_rail":    {"min_translation": -0.06,   "max_translation": 0.055},
        "mount_rail": {"min_translation": -0.09425, "max_translation": 0.09425},
    }
}

MOUNT_RAILS = {
    "lc_mount_rail_0":  {"entity_present": True,  "entity_name": "lc_mount_0",  "entity_pose": {"translation": 0.02,  "roll": 0.0, "pitch": 0.0, "yaw": 0.0}},
    "sfp_mount_rail_0": {"entity_present": True,  "entity_name": "sfp_mount_0", "entity_pose": {"translation": 0.03,  "roll": 0.0, "pitch": 0.0, "yaw": 0.0}},
    "sc_mount_rail_0":  {"entity_present": True,  "entity_name": "sc_mount_0",  "entity_pose": {"translation": -0.02, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}},
    "lc_mount_rail_1":  {"entity_present": True,  "entity_name": "lc_mount_1",  "entity_pose": {"translation": -0.01, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}},
    "sfp_mount_rail_1": {"entity_present": False},
    "sc_mount_rail_1":  {"entity_present": False},
}

def get_nic_true_dict(i):
    return {"entity_present": True,
        "entity_name": f"nic_card_{i}",
            "entity_pose": {
                "translation": random.uniform(*NIC_RAIL_LIMITS),
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": random.uniform(*NIC_YAW_LIMITS)
            },
    }

def get_nic_dict(nic_rail):
    nic_dict = {}
    for i in range(5):
        rail_name = f"nic_rail_{i}"
        is_chosen = (i == nic_rail)
        if is_chosen:
            nic_dict[rail_name] = get_nic_true_dict(i)
        else:
            nic_dict[rail_name] = {"entity_present": False}
    return nic_dict

def get_sc_true_dict(i):
    return {"entity_present": True,
        "entity_name": f"sc_mount_{i}",
            "entity_pose": {
                "translation": random.uniform(*SC_RAIL_LIMITS),
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.1
            },
    }
def get_sc_dict():
    sc_dict = {}
    sc_rail = random.randint(0,1)
    for i in range(2):
        rail_name = f"sc_rail_{i}"
        is_chosen = (i == sc_rail)
        if is_chosen:
            sc_dict[rail_name] = get_sc_true_dict(i)
        else:
            sc_dict[rail_name] = {"entity_present": False}
    
    return sc_dict


def get_task_board_dict(board_x, board_y, board_yaw):
    return {
        "task_board": {
            "pose" : {
                "x": board_x,
                "y": board_y,
                "z": 1.14,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": board_yaw
            }
        }
    }

def generate_cable_dict(cable_name, cable_type, gripper_z):
    return {
        "cables": {
            cable_name: {
                "pose": {
                    "gripper_offset": {
                        "x": 0.0 + random.uniform(-0.002, 0.002),
                        "y": 0.015385 + random.uniform(-0.002, 0.002),
                        "z": gripper_z + random.uniform(-0.002, 0.002),
                    },
                    "roll": 0.4432 + random.uniform(-0.04, 0.04),
                    "pitch": -0.4838 + random.uniform(-0.04, 0.04),
                    "yaw": 1.3303 + random.uniform(-0.04, 0.04),
                },
                "attach_cable_to_gripper": True,
                "cable_type": cable_type,
            }
        },
    }  

def generate_task_dict(cable_name, plug_type, plug_name, port_type, port_name, module_name):
    return {
        "tasks": {
            "task_1": {
                "cable_type": "sfp_sc",
                "cable_name": cable_name,
                "plug_type":  plug_type,
                "plug_name":  plug_name,
                "port_type":  port_type,
                "port_name":  port_name,
                "target_module_name": module_name,
                "time_limit": 180,
            }
        },
    }

def generate_scene_dict(board_x, board_y, board_yaw, rails, cable_name, cable_type, gripper_z):
    return {
        "scene": {
            "task_board": {
                "pose": {
                    "x": board_x, "y": board_y, "z": 1.14,
                    "roll": 0.0, "pitch": 0.0, "yaw": board_yaw,
                },
                **rails,
                **MOUNT_RAILS,
            },
            **generate_cable_dict(cable_name, cable_type, gripper_z),
        }
    }     

def generate_config(trial, idx):
    config = {}
    # random board pose
    board_x   = random.uniform(*BOARD_X_LIMITS)
    board_y   = random.uniform(*BOARD_Y_LIMITS)
    board_yaw = random.uniform(*BOARD_YAW_LIMITS)

    if trial['type'] == "sfp":
        nic_rail = random.randint(0, 4)
        rails = get_nic_dict(nic_rail)
        rails.update({"sc_rail_0": {"entity_present": False},
                      "sc_rail_1": {"entity_present": False}})
        cable_name    = "cable_0"
        cable_type    = "sfp_sc_cable"
        plug_type     = "sfp"
        plug_name     = "sfp_tip"
        port_type     = "sfp"
        port_name     = trial["target_port"].lower()   # sfp_port_0 or sfp_port_1
        module_name   = f"nic_card_mount_{nic_rail}"
        gripper_z     = 0.04245
    else:
        sc_dict = get_sc_dict()
        # figure out which sc_rail was chosen so we can name the module
        sc_rail = next(i for i in range(2) if sc_dict[f"sc_rail_{i}"]["entity_present"])
        rails = {"nic_rail_0": {"entity_present": False},
                "nic_rail_1": {"entity_present": False},
                "nic_rail_2": {"entity_present": False},
                "nic_rail_3": {"entity_present": False},
                "nic_rail_4": {"entity_present": False}}
        rails.update(sc_dict)
        cable_name    = "cable_1"
        cable_type    = "sfp_sc_cable_reversed"
        plug_type     = "sc"
        plug_name     = "sc_tip"
        port_type     = "sc"
        port_name     = "sc_port_base"
        module_name   = f"sc_port_{sc_rail}"
        gripper_z     = 0.04045

    config_path = f"/tmp/aic_trial_{idx}.yaml"

    config = {
        **SCORING_CONFIG,
        **TASK_BOARD_LIMITS,
        "trials": {
            "trial_1": {
                **generate_scene_dict(board_x, board_y, board_yaw, rails, cable_name, cable_type, gripper_z),
                **generate_task_dict(cable_name, plug_type, plug_name, port_type, port_name, module_name),
            }
        },
        "robot": {
            "home_joint_positions": {
                "shoulder_pan_joint":  -0.1597,
                "shoulder_lift_joint": -1.3542,
                "elbow_joint":         -1.6648,
                "wrist_1_joint":       -1.6933,
                "wrist_2_joint":        1.5710,
                "wrist_3_joint":        1.4110,
            }
        },
    }

    with open(config_path, "w") as f:
        yaml.dump(config, f)

    return config_path, module_name, plug_name

def load_progress():
    if not PROGRESS_FILE.exists():
        return set()
    return set(PROGRESS_FILE.read_text().splitlines())

def save_progress(key):
    with PROGRESS_FILE.open("a") as f:
        f.write(key + "\n")

def run_episode(trial, idx):
    repo_id = f"tapan/aic_{trial['name']}_dataset"

    # generate randomised config for this episode
    config_path, module_name, plug_name = generate_config(trial, idx)
   
    engine_cmd = (
        f"distrobox enter -r aic_eval -- /bin/bash -c "
        f"'. /entrypoint.sh; " # Use '.' for sourcing if 'source' behaves oddly
        f"ros2 run aic_engine aic_engine --ros-args "
        f"-p config_file_path:={config_path} "
        f"-p ground_truth:=true "
        f"-p use_sim_time:=true "
        f"-p headless:=true'" # Forces the engine to skip GUI dependencies
    )
    
    engine_proc = subprocess.Popen(engine_cmd, shell=True)
    time.sleep(30)

    record_cmd = (
        f"pixi run lerobot-record "
        f"--dataset.repo_id {repo_id} "
        f"--dataset.num_episodes 1 " # We are doing 1 at a time
        f"--teleop.type=cheatcodeteleop "
        f"--teleop.cable_name={('sfp_sc_cable' if trial['type'] == 'sfp' else 'sfp_sc_cable_reversed')} "
        f"--teleop.plug_name={plug_name} "
        f"--teleop.module_name={module_name} "
        f"--teleop.port_name={trial['target_port'].lower()} "
        f"--dataset.fps 20" # 'ups' is now often 'fps' in newer configs
    )

    record_proc = subprocess.Popen(record_cmd, shell=True)
    record_proc.wait()

    record_proc.terminate()
    engine_proc.terminate()
    subprocess.run("distrobox enter -r aic_eval -- pkill -9 -f gz", shell=True)
    subprocess.run("distrobox enter -r aic_eval -- pkill -9 -f ros", shell=True)
    time.sleep(5)


completed = load_progress()
for trial in TRIALS:
    for i in range(trial["num_episodes"]):
        key = f"{trial['name']}_{i}"
        if key in completed:
            print(f"Skipping {key} (already completed)")
            continue
        run_episode(trial, i)
        save_progress(key)
