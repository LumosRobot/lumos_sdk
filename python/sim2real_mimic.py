#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NIX2 远程策略执行脚本 — Mimic 版（对齐 state_mimic.cpp 部署逻辑）

与 sim2real_lumos.py 的差异
---------------------------
1. 参考动作从纯文本文件 ``store_ref_motion.txt`` 读取（每帧 45 维：
   3 维 cmd + 21 维 dof_pos + 21 维 dof_vel），不依赖 .npz / body_quat。
2. Observation 中 base 朝向 **不用** torso 四元数，而是 **projected_gravity**
   （把 (0,0,-1) 用 pelvis IMU 四元数旋转后得到）—— 与 state_mimic.cpp
   ``input_ob_[3..5]`` 一致。
3. **关节顺序两套**：
       robot 顺序 (= SDK / 电机 / 状态读取顺序)  ── 来自 ``joint_names.yaml`` 中
            ``robot_joint_names``，本工程恰好等于 ``JOINT_NAMES_XML``。
       policy 顺序 (= ONNX 输入输出顺序)         ── 来自 ``joint_names.yaml`` 中
            ``policy_joint_names``，与 robot 顺序不同。
   所以电机读到的数组要 **重排到 policy 顺序** 喂给策略，
   策略输出的 action 要 **按 policy_idx 取值** 再下发到对应的 robot_idx。
4. ``last_actions`` 直接是上一次策略输出（policy 顺序原样），不做 remap，
   与 state_mimic.cpp 中 ``last_actions_[idx] = actions_[idx]`` 完全一致。
5. kp / kd / action_scales / default_dof 全部 **robot 顺序**，从 yaml/txt 读，
   下发与 lumos_controller 一致。

Observation layout (114) — 与 state_mimic.cpp::rl_inference 对应
-----------------------------------------------------------------
    [ 0: 3 )  omega                       (pelvis IMU)
    [ 3: 6 )  projected_gravity           (R(q) @ (0,0,-1))
    [ 6:27 )  joint_pos    (policy 顺序)
    [27:48 )  joint_vel    (policy 顺序)
    [48:69 )  last_actions (policy 输出原样, 无 remap)
    [69:114)  ref_motion_frame (45 维: 3 cmd + 21 dof_pos + 21 dof_vel)

Target
------
    target_q_robot[i] = action_policy[ policy_idx_of(robot_name[i]) ]
                          * action_scale_robot[i] + default_dof_robot[i]
    通过 LCM 按 robot 顺序下发。

用法
----
    python3 python/sim2real_mimic.py \
        --model_dir /home/foolyc/Downloads/suchao_04281718_resim \
        [--start_frame 0] [--end_frame -1] [--loop] [--dry_run]
"""

import argparse
import os
import sys
import time
import threading

import numpy as np
try:
    import onnxruntime
except ImportError:
    onnxruntime = None
try:
    import yaml
except ImportError:
    yaml = None

# ── LCM 类型加载 ─────────────────────────────────────────────────
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(THIS_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, "lcm_typedef", "python"))

import lcm  # noqa: E402


def _load_lcm_class(modname: str):
    import importlib
    m = importlib.import_module(modname)
    cls = getattr(m, modname)
    sys.modules[modname] = cls
    return cls


joint_cmd_lcmt      = _load_lcm_class("joint_cmd_lcmt")
joint_data_lcmt     = _load_lcm_class("joint_data_lcmt")
joint_cmds_lcmt     = _load_lcm_class("joint_cmds_lcmt")
joint_datasets_lcmt = _load_lcm_class("joint_datasets_lcmt")
imu_data_lcmt       = _load_lcm_class("imu_data_lcmt")


# ── 常量 ─────────────────────────────────────────────────────────
LCM_URL       = "udpm://239.255.76.67:7667?ttl=255"
CH_JOINT_CMDS = "lcm_joint_cmd"
CH_JOINT_DATA = "lcm_joint_data"
CH_IMU        = "lcm_imu_data"

CONTROL_HZ  = 125          # 与 lumos_controller 中 epoch_time*inference_interval 对应
NUM_ACTIONS = 21
REF_FIELDS  = 45          # ref_motion 每帧字段数（3 cmd + 21 pos + 21 vel）
NUM_OBS     = 6 + 3 * NUM_ACTIONS + REF_FIELDS    # 6 + 63 + 45 = 114

# ── 关节名 → SDK (component_type, joint_id) 映射 ─────────────────
# component: 1=ARM_L, 2=ARM_R, 7=WAIST, 8=LEG_L, 9=LEG_R
# 字典 key 顺序 == robot/SDK/mujoco 顺序（与 joint_names.yaml 中
# robot_joint_names 完全一致，启动时会校验）。
JOINT_NAME_TO_SDK = {
    # LEG_L
    "left_hip_pitch_joint":   (8, 0),
    "left_hip_roll_joint":    (8, 1),
    "left_hip_yaw_joint":     (8, 2),
    "left_knee_joint":        (8, 3),
    "left_ankle_pitch_joint": (8, 4),
    "left_ankle_roll_joint":  (8, 5),
    # LEG_R
    "right_hip_pitch_joint":   (9, 0),
    "right_hip_roll_joint":    (9, 1),
    "right_hip_yaw_joint":     (9, 2),
    "right_knee_joint":        (9, 3),
    "right_ankle_pitch_joint": (9, 4),
    "right_ankle_roll_joint":  (9, 5),
    # WAIST
    "torso_joint":             (7, 0),
    # ARM_L
    "left_shoulder_pitch_joint": (1, 0),
    "left_shoulder_roll_joint":  (1, 1),
    "left_shoulder_yaw_joint":   (1, 2),
    "left_elbow_joint":          (1, 3),
    # ARM_R
    "right_shoulder_pitch_joint": (2, 0),
    "right_shoulder_roll_joint":  (2, 1),
    "right_shoulder_yaw_joint":   (2, 2),
    "right_elbow_joint":          (2, 3),
}
JOINT_NAMES_XML = list(JOINT_NAME_TO_SDK.keys())


# ═══════════════════════════════════════════════════════════════════
# 状态接收（IMU + 关节）
# ═══════════════════════════════════════════════════════════════════
class RobotState:
    def __init__(self):
        self.lock = threading.Lock()
        self.quat  = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)  # wxyz
        self.omega = np.zeros(3, dtype=np.float64)
        self.joint_pos = {}   # (ctype, jid) -> pos
        self.joint_vel = {}
        self.imu_cnt   = 0
        self.joint_cnt = 0

    def update_imu(self, msg):
        with self.lock:
            self.quat  = np.array(msg.navQuat[:4], dtype=np.float64)
            self.omega = np.array(msg.omega[:3],   dtype=np.float64)
            self.imu_cnt += 1

    def update_joint(self, msg):
        with self.lock:
            for i in range(msg.datasets_num):
                d = msg.datasets[i]
                key = (int(d.component_type), int(d.joint_id))
                self.joint_pos[key] = float(d.pos_high)
                self.joint_vel[key] = float(d.vel)
            self.joint_cnt += 1

    def snapshot_imu(self):
        with self.lock:
            return self.quat.copy(), self.omega.copy()

    def snapshot_joints_by_names(self, joint_names):
        with self.lock:
            n = len(joint_names)
            qpos = np.zeros(n, dtype=np.float64)
            qvel = np.zeros(n, dtype=np.float64)
            for i, name in enumerate(joint_names):
                key = JOINT_NAME_TO_SDK[name]
                qpos[i] = self.joint_pos.get(key, 0.0)
                qvel[i] = self.joint_vel.get(key, 0.0)
            return qpos, qvel


def setup_lcm(lc, state: RobotState):
    lc.subscribe(CH_IMU,        lambda ch, data: state.update_imu(imu_data_lcmt.decode(data)))
    lc.subscribe(CH_JOINT_DATA, lambda ch, data: state.update_joint(joint_datasets_lcmt.decode(data)))


# ═══════════════════════════════════════════════════════════════════
# 四元数 → projected gravity（与 controller::rotate_vector 等价）
# ═══════════════════════════════════════════════════════════════════
def quat_rotate_vector(q, v):
    """wxyz 四元数旋转 3D 向量 v。等价于 R(q) @ v。"""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    # v' = v + w*t + cross(q.xyz, t)
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return np.array([rx, ry, rz])


def projected_gravity_from_quat(q):
    """与 controller 中 ``rotate_vector(q, [0,0,-1])`` 等价。"""
    return quat_rotate_vector(q, np.array([0.0, 0.0, -1.0]))


# ═══════════════════════════════════════════════════════════════════
# 文件加载
# ═══════════════════════════════════════════════════════════════════
def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def load_floats_file(path):
    """读取一个空白分隔的浮点文件 → 1D ndarray。"""
    vals = []
    with open(path, "r") as f:
        for line in f:
            for tok in line.split():
                vals.append(float(tok))
    return np.asarray(vals, dtype=np.float64)


def load_ref_motion(path, n_fields):
    """读取 ref_motion 文件 → (N, n_fields)。"""
    rows = []
    with open(path, "r") as f:
        for line in f:
            toks = line.split()
            if not toks:
                continue
            if len(toks) != n_fields:
                raise ValueError(f"ref_motion 期望 {n_fields} 列，实际 {len(toks)}：{path}")
            rows.append([float(x) for x in toks])
    return np.asarray(rows, dtype=np.float64)


# ═══════════════════════════════════════════════════════════════════
# 策略推理
# ═══════════════════════════════════════════════════════════════════
class PolicyRunner:
    """支持 MLP / LSTM 策略：自动识别 h_in/c_in 并在内部维护隐藏态。"""

    def __init__(self, onnx_path):
        self.session = onnxruntime.InferenceSession(onnx_path)
        self.input_names  = [i.name for i in self.session.get_inputs()]
        self.input_shapes = {i.name: i.shape for i in self.session.get_inputs()}
        self.output_names = [o.name for o in self.session.get_outputs()]

        self.is_lstm = ("h_in" in self.input_names) and ("c_in" in self.input_names)
        if self.is_lstm:
            self.h_shape = [d if isinstance(d, int) and d > 0 else 1
                            for d in self.input_shapes["h_in"]]
            self.c_shape = [d if isinstance(d, int) and d > 0 else 1
                            for d in self.input_shapes["c_in"]]
            self.h = np.zeros(self.h_shape, dtype=np.float32)
            self.c = np.zeros(self.c_shape, dtype=np.float32)
            # 输出索引
            self.idx_actions = self.output_names.index("actions") \
                if "actions" in self.output_names else 0
            self.idx_h_out = self.output_names.index("h_out")
            self.idx_c_out = self.output_names.index("c_out")
            print(f"[POLICY] LSTM detected. h={self.h_shape} c={self.c_shape}")
        else:
            print(f"[POLICY] MLP. inputs={self.input_names}")

    def reset(self):
        if self.is_lstm:
            self.h.fill(0.0)
            self.c.fill(0.0)

    def infer(self, obs_np):
        """obs_np: (1, NUM_OBS) float32 → action: (NUM_ACTIONS,)"""
        feed = {"obs": obs_np.astype(np.float32)}
        if self.is_lstm:
            feed["h_in"] = self.h
            feed["c_in"] = self.c
        outs = self.session.run(None, feed)
        if self.is_lstm:
            self.h = outs[self.idx_h_out]
            self.c = outs[self.idx_c_out]
            action = outs[self.idx_actions]
        else:
            action = outs[0]
        return action.reshape(-1)


# ═══════════════════════════════════════════════════════════════════
# Observation 构造（layout 与 state_mimic.cpp 一致）
# ═══════════════════════════════════════════════════════════════════
def build_observation(omega, proj_gravity,
                      qpos_policy, qvel_policy,
                      last_actions_policy, ref_motion_frame):
    obs = np.zeros(NUM_OBS, dtype=np.float32)
    off = 0
    obs[off:off + 3] = omega;                              off += 3
    obs[off:off + 3] = proj_gravity;                       off += 3
    obs[off:off + NUM_ACTIONS] = qpos_policy;              off += NUM_ACTIONS
    obs[off:off + NUM_ACTIONS] = qvel_policy;              off += NUM_ACTIONS
    obs[off:off + NUM_ACTIONS] = last_actions_policy;      off += NUM_ACTIONS
    obs[off:off + REF_FIELDS]  = ref_motion_frame
    return obs


# ═══════════════════════════════════════════════════════════════════
# 关节指令下发（robot/xml 顺序）
# ═══════════════════════════════════════════════════════════════════
def send_joint_targets(lc, joint_names, target_pos, kp, kd):
    cmds = joint_cmds_lcmt()
    cmds.cmds_num = len(joint_names)
    cmds.cmds = []
    for i, name in enumerate(joint_names):
        ctype, jid = JOINT_NAME_TO_SDK[name]
        c = joint_cmd_lcmt()
        c.component_type = ctype
        c.joint_id       = jid
        c.ctrlWord       = 3
        c.tarPos         = float(target_pos[i])
        c.tarVel         = 0.0
        c.tarCur         = 0.0
        c.tarTor         = 0.0
        c.res1           = float(kp[i])
        c.res2           = float(kd[i])
        c.res3           = 0.0
        c.res4           = 0.0
        cmds.cmds.append(c)
    lc.publish(CH_JOINT_CMDS, cmds.encode())


# ═══════════════════════════════════════════════════════════════════
# 主流程
# ═══════════════════════════════════════════════════════════════════
def run(model_dir, start_frame=0, end_frame=-1, loop_motion=False, dry_run=False):
    if onnxruntime is None or yaml is None:
        missing = []
        if onnxruntime is None:
            missing.append("onnxruntime")
        if yaml is None:
            missing.append("PyYAML")
        print(f"[ERROR] 缺少 Python 依赖: {', '.join(missing)}。请先安装后再运行 mimic 策略回放。")
        return

    # ── 解析路径 ────────────────────────────────────────────────
    p_joint_names = os.path.join(model_dir, "joint_names.yaml")
    p_kp_kd       = os.path.join(model_dir, "kp_kd.yaml")
    p_default     = os.path.join(model_dir, "store_ref_init_dof_pos.txt")
    p_ref_motion  = os.path.join(model_dir, "store_ref_motion.txt")
    p_policy      = os.path.join(model_dir, "policy.onnx")
    for p in (p_joint_names, p_kp_kd, p_default, p_ref_motion, p_policy):
        if not os.path.exists(p):
            print(f"[ERROR] 文件不存在: {p}")
            return

    # ── 关节顺序 ────────────────────────────────────────────────
    jn = load_yaml(p_joint_names)
    policy_joint_names = list(jn["policy_joint_names"])
    robot_joint_names  = list(jn["robot_joint_names"])
    assert len(policy_joint_names) == NUM_ACTIONS
    assert len(robot_joint_names)  == NUM_ACTIONS

    # 校验 robot_joint_names == JOINT_NAMES_XML（顺序+集合）
    if robot_joint_names != JOINT_NAMES_XML:
        print("[ERROR] robot_joint_names 与脚本内 JOINT_NAMES_XML 顺序不一致：")
        for i, (a, b) in enumerate(zip(robot_joint_names, JOINT_NAMES_XML)):
            mark = "" if a == b else "  <<< MISMATCH"
            print(f"  [{i:2d}]  yaml={a:<32s}  script={b}{mark}")
        return
    if set(policy_joint_names) != set(robot_joint_names):
        print("[ERROR] policy/robot joint name 集合不一致")
        return

    # 排序映射：
    #   idx_robot2policy[r] = p   表示 robot 顺序第 r 个关节，对应 policy 第 p 个
    #     → policy_arr[idx_robot2policy[r]] 是 robot[r] 的策略值
    #   idx_policy2robot[p] = r   表示 policy 顺序第 p 个关节，对应 robot 第 r 个
    #     → robot_arr[idx_policy2robot[p]] 是 policy[p] 对应的电机值
    idx_robot2policy = np.array(
        [policy_joint_names.index(n) for n in robot_joint_names], dtype=np.int64
    )
    idx_policy2robot = np.array(
        [robot_joint_names.index(n)  for n in policy_joint_names], dtype=np.int64
    )

    # ── kp/kd/scales/default（全部 robot 顺序）──────────────────
    cp = load_yaml(p_kp_kd)
    kp_robot     = np.asarray(cp["kps"],    dtype=np.float64)
    kd_robot     = np.asarray(cp["kds"],    dtype=np.float64)
    scale_robot  = np.asarray(cp["scales"], dtype=np.float64)
    default_robot = load_floats_file(p_default)
    for arr, name in [(kp_robot, "kps"), (kd_robot, "kds"),
                      (scale_robot, "scales"), (default_robot, "default_dof")]:
        if arr.shape[0] != NUM_ACTIONS:
            print(f"[ERROR] {name} 长度 {arr.shape[0]} != {NUM_ACTIONS}")
            return

    # ── ref_motion ─────────────────────────────────────────────
    ref_motion = load_ref_motion(p_ref_motion, REF_FIELDS)
    total_frames = ref_motion.shape[0]
    if end_frame < 0 or end_frame > total_frames:
        end_frame = total_frames
    if start_frame < 0 or start_frame >= end_frame:
        start_frame = 0
    print(f"[INFO] ref_motion frames total={total_frames}, "
          f"range=[{start_frame},{end_frame}), loop={loop_motion}")

    # ── 打印关节映射 ────────────────────────────────────────────
    print(f"[INFO] robot order (= xml/SDK)        | policy order")
    for i in range(NUM_ACTIONS):
        rn  = robot_joint_names[i]
        pn  = policy_joint_names[i]
        c, j = JOINT_NAME_TO_SDK[rn]
        print(f"  r[{i:2d}] {rn:<30s}  SDK({c},{j})  kp={kp_robot[i]:6.2f} "
              f"kd={kd_robot[i]:5.2f} scale={scale_robot[i]:.2f} def={default_robot[i]:+.3f} "
              f" | p[{i:2d}] {pn}")

    # ── 加载 ONNX 策略 ─────────────────────────────────────────
    policy = PolicyRunner(p_policy)

    # ── 建立 LCM ───────────────────────────────────────────────
    lc = lcm.LCM(LCM_URL)
    state = RobotState()
    setup_lcm(lc, state)

    print("[INFO] Waiting for IMU + lcm_joint_data ...")
    t0 = time.time()
    while time.time() - t0 < 5.0:
        lc.handle_timeout(10)
        if state.imu_cnt > 0 and state.joint_cnt > 0:
            break
    if state.imu_cnt == 0 or state.joint_cnt == 0:
        print(f"[ERROR] no data. imu={state.imu_cnt} joint={state.joint_cnt}")
        return
    print(f"[INFO] OK. imu={state.imu_cnt} joint={state.joint_cnt}")

    if dry_run:
        print("[WARN] DRY-RUN: 仅推理策略，不下发关节指令")
    else:
        print("[WARN] 即将下发关节指令，请确认机器人已 STAND 且进入 DEBUG 状态。3s 后开始 ...")
        time.sleep(3.0)

    # ── 控制循环（与 sim2real_lumos.py 相同的时间门 + LCM 紧循环范式）──
    INNER_TIMEOUT_MS = 2
    dt = 1.0 / CONTROL_HZ

    # last_actions_policy: 上一次策略输出原样（policy 顺序），对齐
    # state_mimic.cpp 中 last_actions_[idx] = actions_[idx]
    last_actions_policy = np.zeros(NUM_ACTIONS, dtype=np.float32)

    step = 0
    rl_iter = start_frame
    loop_iters = 0
    infer_times = []
    t_start = time.perf_counter()
    next_infer_time = t_start

    try:
        while True:
            lc.handle_timeout(INNER_TIMEOUT_MS)
            loop_iters += 1

            now = time.perf_counter()
            if now < next_infer_time:
                continue

            # 1) 取 ref_motion 帧索引（与 controller rl_iter_ % frame_num 一致）
            if loop_motion:
                frame_idx = rl_iter % total_frames
                if frame_idx >= end_frame:
                    rl_iter = start_frame
                    frame_idx = start_frame
            else:
                if rl_iter >= end_frame:
                    print("[INFO] ref_motion finished.")
                    break
                frame_idx = rl_iter

            ref_frame = ref_motion[frame_idx]   # (45,)

            # 2) 取机器人状态
            quat, omega = state.snapshot_imu()
            qpos_robot, qvel_robot = state.snapshot_joints_by_names(robot_joint_names)

            proj_g = projected_gravity_from_quat(quat)

            # 3) robot → policy 排序
            qpos_policy = qpos_robot[idx_policy2robot]
            qvel_policy = qvel_robot[idx_policy2robot]

            # 4) 构造 observation
            obs = build_observation(
                omega=omega,
                proj_gravity=proj_g,
                qpos_policy=qpos_policy,
                qvel_policy=qvel_policy,
                last_actions_policy=last_actions_policy,
                ref_motion_frame=ref_frame,
            )

            # 5) 推理
            t_infer_start = time.perf_counter()
            action_policy = policy.infer(obs.reshape(1, -1))
            t_infer_ms = (time.perf_counter() - t_infer_start) * 1000.0
            infer_times.append(t_infer_ms)

            # 更新 last_actions（policy 顺序原样，与 controller 一致）
            last_actions_policy = action_policy.astype(np.float32).copy()

            # 6) target：policy → robot 排序
            #    target_robot[i] = action_policy[idx_robot2policy[i]] * scale_robot[i] + default_robot[i]
            action_for_robot = action_policy[idx_robot2policy]
            # target_robot = action_for_robot * scale_robot + default_robot
            target_robot = action_for_robot * scale_robot
            # 7) 下发（robot 顺序）
            if not dry_run:
                send_joint_targets(lc, robot_joint_names, target_robot, kp_robot, kd_robot)

            # 8) 打印
            if step > 0 and step % 50 == 0:
                avg_infer = sum(infer_times) / len(infer_times)
                max_infer = max(infer_times)
                elapsed   = time.perf_counter() - t_start
                actual_hz = step / max(elapsed, 1e-6)
                iters_per_step = loop_iters / step
                print(f"[STEP] step={step:4d} frame={frame_idx}/{end_frame}  "
                      f"hz={actual_hz:.1f}  "
                      f"infer_avg={avg_infer:.2f}ms infer_max={max_infer:.2f}ms  "
                      f"lcm_iters/step={iters_per_step:.1f}  "
                      f"target_robot[:3]={target_robot[:3]}")
                infer_times.clear()

            step    += 1
            rl_iter += 1

            next_infer_time += dt
            if time.perf_counter() > next_infer_time + dt:
                overrun_ms = (time.perf_counter() - next_infer_time) * 1000.0
                print(f"[WARN] control loop overrun by {overrun_ms:.1f} ms, resetting tick")
                next_infer_time = time.perf_counter()

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt, exiting.")


# ═══════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model_dir", required=True,
                    help="包含 policy.onnx / joint_names.yaml / kp_kd.yaml / "
                         "store_ref_init_dof_pos.txt / store_ref_motion.txt 的目录")
    ap.add_argument("--start_frame", type=int, default=0)
    ap.add_argument("--end_frame",   type=int, default=-1,
                    help="不含；-1 表示到末尾")
    ap.add_argument("--loop", action="store_true", help="循环播放 ref_motion")
    ap.add_argument("--dry_run", action="store_true",
                    help="只推理不下发关节指令（首次测试请加）")
    args = ap.parse_args()

    run(
        model_dir=args.model_dir,
        start_frame=args.start_frame,
        end_frame=args.end_frame,
        loop_motion=args.loop,
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    main()
