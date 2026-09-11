"""Unified sim2sim script supporting multiple robot configurations.

Usage Examples:
    python models/nix2_policy/ampwalk_0204084348/sim2sim_keyboard.py --robot nix25_joint21_run
"""

import argparse
import time
import os
from datetime import datetime

import mujoco
import mujoco.viewer
import numpy as np
import onnx
import onnxruntime
from scipy.spatial.transform import Rotation as R
from pynput import keyboard

# Simulation parameters
simulation_duration = 200.0
simulation_dt = 0.002
control_decimation = 10

# Robot configurations
ROBOT_CONFIGS = {
    "nix25_joint21_run": {
        "num_actions": 21,
        "num_obs_per_step": 78,
        "actor_obs_history_length":10,
        "default_xml": "/home/congzz/workspace/lumos_ws/st_gym/third_party/lumos_assets/nix2-5/mjcf/nix2-5.xml",  # Must be provided
        "joint_names": [
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", 
            "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", 
            "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "torso_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint"
        ],
        "observation_structure": {
            "ang_vel": 3,
            "projected_gravity": 3,
            "command": 3,
            "joint_pos": 21,
            "joint_vel": 21,
            "actions": 21,
            "sin": 2,
            "cos": 2,
            "phase": 2,
        },
        "gait_air_ratio_l": 0.6,
        "gait_air_ratio_r": 0.6,
        "gait_phase_offset_l":0.6,
        "gait_phase_offset_r":0.1,
        "gait_cycle": 0.5,
        "clip_observations": 100.0,
        "clip_actions": 100.0,
        "action_scale": 0.25,
        "joint_default_positions": np.array([-0.1, 0.0, 0.0, 0.2, -0.1, 0.0, 
                                         -0.1, 0.0, 0.0, 0.2, -0.1, 0.0,
                                         0.0,
                                         0.15, 0.12, 0.0, 1.2,
                                         0.15, -0.12, 0.0, 1.2]),
        "joint_stiffness": np.array([55, 55, 55, 55, 20, 20, 55, 55, 55, 55, 20, 20, 55, 55, 55, 55, 55, 55, 55, 55, 55]),
        "joint_damping": np.array([3, 3, 3, 3, 2, 2, 3, 3, 3, 3, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3]),
    },
}

def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd

def set_cmd_overlay(viewer, cmd_vx, cmd_vy, cmd_wz, base_vx, base_vy, base_wz):
    """Overlay the commanded and measured base-frame velocity on the viewer.

    Layout (two text columns):
             x       y       z
        cmd +0.00   +0.00   +0.00
        vel +0.00   +0.00   +0.00
    """
    left_col = "\ncmd\nvel"
    right_col = (
        "   x       y       z\n"
        f"{cmd_vx:+.2f}   {cmd_vy:+.2f}   {cmd_wz:+.2f}\n"
        f"{base_vx:+.2f}   {base_vy:+.2f}   {base_wz:+.2f}"
    )
    viewer.set_texts((None, None, left_col, right_col))

def run_simulation(robot_type: str,  policy_path: str):
    global time_step, line_x_vel, line_y_vel, ang_z_vel 
    time_step, line_x_vel, line_y_vel, ang_z_vel = 0, 0.0, 0.0, 0.0

    def on_press(key):
        global line_x_vel, line_y_vel, ang_z_vel, time_step
        try:
            if key.char == 'w':
                line_x_vel = min(line_x_vel + 0.1, 4.0)
                print(f"X velocity: {line_x_vel:.2f}")
            elif key.char == 's':
                line_x_vel = max(line_x_vel - 0.1, -4.0)
                print(f"X velocity: {line_x_vel:.2f}")
            elif key.char == 'a':
                line_y_vel = min(line_y_vel + 0.1, 0.5)
                print(f"Y velocity: {line_y_vel:.2f}")
            elif key.char == 'd':
                line_y_vel = max(line_y_vel - 0.1, -0.5)
                print(f"Y velocity: {line_y_vel:.2f}")
            elif key.char == 'q':
                ang_z_vel = min(ang_z_vel + 0.1, 1.5)
                print(f"Angular Z velocity: {ang_z_vel:.2f}")
            elif key.char == 'e':
                ang_z_vel = max(ang_z_vel - 0.1, -1.5)
                print(f"Angular Z velocity: {ang_z_vel:.2f}")
            elif key.char == 'r':
                line_x_vel = 0.0
                line_y_vel = 0.0
                ang_z_vel = 0.0
                time_step = 0
                print("Reset velocities")
        except AttributeError:
            if key == keyboard.Key.esc:
                return False  # 停止监听

    """Run the sim2sim simulation."""
    config = ROBOT_CONFIGS[robot_type]
    print(f"[INFO]: Using robot configuration: {robot_type}")
    
    # Load ONNX model and extract metadata
    model = onnx.load(policy_path)
    isaac_joint_names = ["left_hip_pitch_joint", "right_hip_pitch_joint", "torso_joint", "left_hip_roll_joint", "right_hip_roll_joint", 
                            "left_shoulder_pitch_joint", "right_shoulder_pitch_joint", "left_hip_yaw_joint", "right_hip_yaw_joint", 
                            "left_shoulder_roll_joint", "right_shoulder_roll_joint", "left_knee_joint", "right_knee_joint", 
                            "left_shoulder_yaw_joint", "right_shoulder_yaw_joint", "left_ankle_pitch_joint", "right_ankle_pitch_joint", 
                            "left_elbow_joint", "right_elbow_joint", "left_ankle_roll_joint", "right_ankle_roll_joint"]
    print("isaac_joint_names", isaac_joint_names)
       
    mujoco_joint_names = config["joint_names"]
    print("mujoco_joint_names", mujoco_joint_names)
    joint_default_pos_robot_idx = config["joint_default_positions"]
    # Precompute joint index mappings between MuJoCo (robot) order and policy (Isaac)
    # order once, instead of doing O(n) list.index() lookups every control step.
    isaac_to_mujoco_idx = np.array([mujoco_joint_names.index(joint) for joint in isaac_joint_names])
    mujoco_to_isaac_idx = np.array([isaac_joint_names.index(joint) for joint in mujoco_joint_names])
    joint_default_pos_policy_idx = joint_default_pos_robot_idx[isaac_to_mujoco_idx]
    stiffness_array = config["joint_stiffness"]
    damping_array = config["joint_damping"]
    
    print("joint_default_pos_robot_idx", joint_default_pos_robot_idx)
    print("joint_default_pos_policy_idx", joint_default_pos_policy_idx)
    print("stiffness_array", stiffness_array)
    print("damping_array", damping_array)
    
    # Initialize variables
    num_actions = config["num_actions"]
    clip_actions = config["clip_actions"]
    action_scale = config["action_scale"]
    clip_observations = config["clip_observations"]
    num_obs = config["num_obs_per_step"] * config["actor_obs_history_length"]
    num_obs_per_step = config["num_obs_per_step"]
    xml_path = config["default_xml"]
    action = np.zeros(num_actions, dtype=np.float32)
    obs_history = np.zeros(num_obs, dtype=np.float32)
    counter = 0
    gait_phase = np.zeros(2)
    gait_cycle = config["gait_cycle"]
    phase_ratio = np.array([config["gait_air_ratio_l"], config["gait_air_ratio_r"]])
    phase_offset = np.array([config["gait_phase_offset_l"], config["gait_phase_offset_r"]])
    dt= simulation_dt * control_decimation
    print(f"[INFO]: Actions: {num_actions}, Observations: {num_obs}")
    
    # Load robot model
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt

    # Cache sensor data addresses to avoid per-step string lookups
    orient_adr = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "orientation")]
    gyro_adr = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "angular-velocity")]
    
    # Load policy
    policy = onnxruntime.InferenceSession(policy_path)
    
    action_buffer = np.zeros((num_actions,), dtype=np.float32)
    
    target_dof_pos = joint_default_pos_robot_idx.copy()
    zero_target_dq = np.zeros(num_actions, dtype=np.double)
    
    # 启动键盘监听线程
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    with mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False) as viewer:

        # 获取当前相机
        cam = viewer.cam  

        # 设置跟踪相机
        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING  # 跟踪模式
        cam.trackbodyid = 1  # 要跟踪的 body ID（如机器人基座）
        cam.lookat[:] = [0, 0, 0]  # 相机看向的偏移量
        cam.distance = 2.0  # 相机距离
        cam.azimuth = 135 # 水平旋转角度 (0~360°)
        cam.elevation = -10  # 俯仰角度 (-90°~90°)
        cam.trackbodyid = 0  # 跟踪的body id，-1表示不跟踪任何body
        viewer.opt.geomgroup[:] = [0, 1, 1, 1, 0, 0] # 显示碰撞几何体

        start = time.time()
        
        while viewer.is_running() and time.time() - start < simulation_duration:
            cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING  # 跟踪模式
            step_start = time.time()
            command_vel = np.array([line_x_vel, line_y_vel, ang_z_vel], dtype=np.float32)
            # print(f"command_vel: {command_vel}")

            mujoco.mj_step(m, d)

            # Calculate control torques
            tau = pd_control(target_dof_pos, d.qpos[7:], stiffness_array,
                           zero_target_dq, d.qvel[6:], damping_array)
            d.ctrl[:] = tau
            counter += 1

            if counter % control_decimation == 0:
                # IMU readings — only computed at control rate (they are unused
                # on the other physics substeps)
                quat = d.sensordata[orient_adr:orient_adr + 4][[1, 2, 3, 0]]
                r = R.from_quat(quat)
                omega = d.sensordata[gyro_adr:gyro_adr + 3]
                gvec = r.apply([0.0, 0.0, -1.0], inverse=True)
                # Base-frame linear velocity (for the real-time cmd/vel overlay)
                base_lin_vel = r.apply(d.qvel[:3], inverse=True)

                # Reorder joints from MuJoCo order to policy order via fancy indexing
                qpos_policy_idx = d.qpos[7:7 + num_actions][isaac_to_mujoco_idx]
                qvel_policy_idx = d.qvel[6:6 + num_actions][isaac_to_mujoco_idx]

                obs_per_step = np.zeros(num_obs_per_step, dtype=np.float32)
                offset = 0
                obs_per_step[offset:offset + 3] = omega * 0.25
                offset += 3
                obs_per_step[offset:offset + 3] = gvec
                offset += 3
                obs_per_step[offset:offset + 3] = command_vel
                offset += 3
                obs_per_step[offset:offset + num_actions] = qpos_policy_idx - joint_default_pos_policy_idx
                offset += num_actions
                obs_per_step[offset:offset + num_actions] = qvel_policy_idx * 0.05
                offset += num_actions
                obs_per_step[offset:offset + num_actions] = action_buffer
                offset += num_actions
                obs_per_step[offset:offset + 2] = np.sin(2 * np.pi * gait_phase)
                offset += 2
                obs_per_step[offset:offset + 2] = np.cos(2 * np.pi * gait_phase)
                offset += 2
                obs_per_step[offset:offset + 2] = phase_ratio

                # Real-time cmd/velocity overlay on the viewer (updated at control rate)
                set_cmd_overlay(viewer, command_vel[0], command_vel[1], command_vel[2],
                                base_lin_vel[0], base_lin_vel[1], omega[2])

                # Shift history in place instead of np.roll (avoids an allocation)
                obs_history[:-num_obs_per_step] = obs_history[num_obs_per_step:]
                obs_history[-num_obs_per_step:] = obs_per_step
                obs = obs_history

                # Run policy inference (feed numpy directly, no torch round-trip)
                action = policy.run(['actions'], {'obs': obs[None, :]})[0]

                action = np.asarray(action).reshape(-1)
                action_buffer = action.copy()
                target_dof_pos = (action * action_scale + joint_default_pos_policy_idx)[mujoco_to_isaac_idx]

                # Advance time step
                time_step += 1
                temp = time_step * dt / gait_cycle

                # Only advance gait phase when there is a non-negligible commanded velocity
                cmd_xy_norm = np.linalg.norm(command_vel[:2])
                if cmd_xy_norm > 0.1:
                    gait_phase[0] = (temp + phase_offset[0]) % 1.0
                    gait_phase[1] = (temp + phase_offset[1]) % 1.0
                else:
                    # If not moving, keep gait phases at zero
                    gait_phase[0] = 0.0
                    gait_phase[1] = 0.0

            viewer.sync()

            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    
    listener.stop()


def main():
    parser = argparse.ArgumentParser(description="Unified sim2sim script for multiple robots.")
    parser.add_argument("--robot", type=str, choices=["nix25_joint21_run"], default="nix25_joint21_run", help="Robot type: nix25_joint21_run")
    args = parser.parse_args()

    policy_path = os.path.join(f"models/nix2_policy/ampwalk_0204084348/policy.onnx")

    print(f"[INFO]: Robot: {args.robot}")
    print(f"[INFO]: Policy path: {policy_path}")
    
    run_simulation(args.robot, policy_path)


if __name__ == "__main__":
    main()