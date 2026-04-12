import subprocess
import time
import random
from pathlib import Path

PROGRESS_FILE = Path("collection_progress.txt")

NIC_RAIL_LIMITS  = (-0.0215, 0.0234)
NIC_YAW_LIMITS   = (-0.1745, 0.1745)
SC_RAIL_LIMITS   = (-0.06, 0.055)
BOARD_X_LIMITS   = (0.13, 0.20)
BOARD_Y_LIMITS   = (-0.25, 0.25)
BOARD_YAW_LIMITS = (2.8, 3.4)

TRIALS = [
    {"name": "sfp_port0", "type": "sfp", "target_port": "sfp_port_0", "num_episodes": 1},
    {"name": "sfp_port1", "type": "sfp", "target_port": "sfp_port_1", "num_episodes": 1},
    {"name": "sc_port0",  "type": "sc",  "target_port": "sc_port_0",  "num_episodes": 1},
    {"name": "sc_port1",  "type": "sc",  "target_port": "sc_port_1",  "num_episodes": 1},
]

def load_progress():
    if not PROGRESS_FILE.exists():
        return set()
    return set(PROGRESS_FILE.read_text().splitlines())

def save_progress(key):
    with PROGRESS_FILE.open("a") as f:
        f.write(key + "\n")

def run_episode(trial, idx):
    board_x   = random.uniform(*BOARD_X_LIMITS)
    board_y   = random.uniform(*BOARD_Y_LIMITS)
    board_yaw = random.uniform(*BOARD_YAW_LIMITS)

    if trial["type"] == "sfp":
        nic_rail    = random.randint(0, 4)
        module_name = f"nic_card_mount_{nic_rail}"
        plug_name   = "sfp_tip"
        cable_type  = "sfp_sc_cable"
        rail_params = (
            f"nic_card_mount_{nic_rail}_present:=true "
            f"nic_card_mount_{nic_rail}_translation:={random.uniform(*NIC_RAIL_LIMITS):.4f} "
            f"nic_card_mount_{nic_rail}_yaw:={random.uniform(*NIC_YAW_LIMITS):.4f} "
            f"sfp_mount_rail_0_present:=true "
        )
    else:
        sc_rail     = int(trial["target_port"][-1])
        module_name = f"sc_port_{sc_rail}"
        plug_name   = "sc_tip"
        cable_type  = "sfp_sc_cable_reversed"
        rail_params = (
            f"sc_port_rail_{sc_rail}_present:=true "
            f"sc_port_rail_{sc_rail}_translation:={random.uniform(*SC_RAIL_LIMITS):.4f} "
        )

    engine_cmd = (
        "distrobox enter -r aic_eval -- /bin/bash -c "
        f"'. /entrypoint.sh "
        f"ground_truth:=true "
        f"spawn_task_board:=true "
        f"spawn_cable:=true "
        f"attach_cable_to_gripper:=true "
        f"cable_type:={cable_type} "
        f"task_board_x:={board_x:.4f} "
        f"task_board_y:={board_y:.4f} "
        f"task_board_z:=1.14 "
        f"task_board_yaw:={board_yaw:.4f} "
        f"gazebo_gui:=false "
        f"{rail_params}'"
    )

    record_cmd = (
        "cd ~/aic_challenge_dangermark &&"
        "pixi run lerobot-record "
        "--robot.type=aic_controller "
        "--robot.id=aic "
        "--robot.teleop_target_mode=cartesian "
        "--robot.teleop_frame_id=base_link "
        "--teleop.type=cheatcodeteleop "
        "--teleop.id=aic "
        f"--teleop.cable_name={cable_type} "
        f"--teleop.plug_name={plug_name} "
        f"--teleop.module_name={module_name} "
        f"--teleop.port_name={trial['target_port']} "
        f"--dataset.repo_id=tapan/aic_{trial['name']}_dataset "
        f"--dataset.single_task='Insert {plug_name} into {trial['target_port']}' "
        "--dataset.num_episodes=1 "
        "--dataset.fps=20 "
        "--dataset.push_to_hub=false "
        "--dataset.private=true "
        "--display_data=false "
        "--play_sounds=false"
    )

    print(f"\n{'='*60}")
    print(f"Episode {idx} — {trial['name']} | module: {module_name} | plug: {plug_name}")
    print(f"{'='*60}")

    engine_proc = subprocess.Popen(engine_cmd, shell=True)
    print("Waiting for sim to initialise...")
    time.sleep(120)

    record_proc = subprocess.Popen(record_cmd, shell=True)
    record_proc.wait()

    engine_proc.terminate()
    subprocess.run("distrobox enter -r aic_eval -- pkill -9 -f gz", shell=True)
    subprocess.run("distrobox enter -r aic_eval -- pkill -9 -f ros", shell=True)
    time.sleep(5)

if __name__ == "__main__":
    completed = load_progress()
    for trial in TRIALS:
        for i in range(trial["num_episodes"]):
            key = f"{trial['name']}_{i}"
            if key in completed:
                print(f"Skipping {key} (already completed)")
                continue
            run_episode(trial, i)
            save_progress(key)