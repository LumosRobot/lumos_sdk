#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NIX2 远程策略执行脚本（LCM 版，去除 MuJoCo）

流程：
    1. 通过 LCM 订阅 IMU + JointsData，作为机器人状态
    2. 加载 ONNX 策略 + 参考动作 npz
    3. 按 50Hz 构造 observation，跑策略，得到 action
    4. 通过 LCM 把 21 个关节的 target_pos = default_pos + action*scale 下发

前置：
    - 机器人端已运行 lumos_controller
    - 已 source config_network_lcm.sh <网卡> 配置多播
    - 已通过 sdk_debug.py 完成 RESET→STAND→进入 SDK 模式：
          python3 python/sdk_debug.py state 1
          python3 python/sdk_debug.py state 2     # 等约 11s 站稳
          python3 python/sdk_debug.py mode 1

用法：
    python3 python/sim2real_lumos.py \
        --motion_file path/to/box.npz \
        --policy_path path/to/policy.onnx \
        [--loop] [--dry_run]

注意：
    - --dry_run 只跑策略不下发关节指令，建议第一次使用先 dry_run。
    - 关节顺序：策略 metadata 里的 joint_names 必须与 SDK 组件顺序兼容。
      本脚本按 lumos_nix2 的 21 关节布局映射到 SDK (component_type, joint_id)。
"""

import argparse
import os
import sys
import time
import threading

import numpy as np
import onnx
import onnxruntime

# ── LCM 类型加载（绕过生成代码的 module-vs-class 问题） ─────────────
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
imu_data_lcmt        = _load_lcm_class("imu_data_lcmt")


# ── 常量 ─────────────────────────────────────────────────────────────
LCM_URL = "udpm://239.255.76.67:7667?ttl=255"
CH_JOINT_CMDS = "lcm_joint_cmd"
CH_JOINT_DATA = "lcm_joint_data"
CH_IMU        = "lcm_imu_data"

CONTROL_HZ    = 50          # 与 sim2sim 中 1/(simulation_dt*control_decimation) 一致
NUM_ACTIONS   = 21
NUM_OBS       = 114

# ── 关节名 → SDK (component_type, joint_id) 映射 ─────────────────────
# component: 1=ARM_L, 2=ARM_R, 7=WAIST, 8=LEG_L, 9=LEG_R
# 字典 key 顺序 == 「mujoco/xml 排序」(= sim2sim 中 config["joint_names"]
# for lumos_nix2)。这是电机下发用的顺序，也是 SDK / 真实机器人侧的顺序。
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

# mujoco/xml 排序的名字列表（用于状态读取 & 电机下发）
JOINT_NAMES_XML = list(JOINT_NAME_TO_SDK.keys())


# ═══════════════════════════════════════════════════════════════════
# 状态接收（IMU + 关节）
# ═══════════════════════════════════════════════════════════════════
class RobotState:
    """线程安全的机器人最新状态缓存。"""

    def __init__(self):
        self.lock = threading.Lock()
        # IMU
        self.quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)  # wxyz
        self.omega = np.zeros(3, dtype=np.float64)
        # 关节：用 (component_type, joint_id) → (pos, vel)
        self.joint_pos = {}
        self.joint_vel = {}
        self.imu_cnt = 0
        self.joint_cnt = 0

    def update_imu(self, msg):
        with self.lock:
            self.quat = np.array(msg.navQuat[:4], dtype=np.float64)  # wxyz
            self.omega = np.array(msg.omega[:3], dtype=np.float64)
            self.imu_cnt += 1

    def update_joint(self, msg):
        with self.lock:
            for i in range(msg.datasets_num):
                d = msg.datasets[i]
                key = (int(d.component_type), int(d.joint_id))
                self.joint_pos[key] = float(d.pos_high)
                self.joint_vel[key] = float(d.vel)
            self.joint_cnt += 1

    def snapshot_joints_by_names(self, joint_names):
        """按给定关节名顺序返回 (qpos, qvel)，每个名字通过 JOINT_NAME_TO_SDK 查 SDK 索引。"""
        with self.lock:
            n = len(joint_names)
            qpos = np.zeros(n, dtype=np.float64)
            qvel = np.zeros(n, dtype=np.float64)
            for i, name in enumerate(joint_names):
                key = JOINT_NAME_TO_SDK[name]
                qpos[i] = self.joint_pos.get(key, 0.0)
                qvel[i] = self.joint_vel.get(key, 0.0)
            return qpos, qvel

    def snapshot_imu(self):
        with self.lock:
            return self.quat.copy(), self.omega.copy()


def setup_lcm(lc, state: RobotState):
    """订阅 LCM 通道，在主线程 handle_timeout 循环中消费。"""
    lc.subscribe(CH_IMU,        lambda ch, data: state.update_imu(imu_data_lcmt.decode(data)))
    lc.subscribe(CH_JOINT_DATA, lambda ch, data: state.update_joint(joint_datasets_lcmt.decode(data)))


# ═══════════════════════════════════════════════════════════════════
# 四元数 / 旋转工具（从 sim2sim 抽取的 NumPy 版本）
# ═══════════════════════════════════════════════════════════════════
def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_inv(q, eps=1e-9):
    return quat_conjugate(q) / max(np.sum(q * q), eps)


def extract_yaw_quat(q):
    """从 wxyz 四元数中提取仅含 yaw 分量的四元数 (绕世界 Z 轴)。

    与 controller 端 extractYawQuaternion 等价：先把 quat 转成 yaw 角，
    再用 yaw 角构造新四元数 [cos(yaw/2), 0, 0, sin(yaw/2)]。
    """
    w, x, y, z = q
    # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    half = 0.5 * yaw
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)])


def quat_from_yaw(yaw):
    """绕 z 轴 yaw 旋转 → wxyz 四元数。"""
    half = 0.5 * yaw
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)])


def transform_pelvis_to_torso(pelvis_quat, torso_joint_angle):
    """与 controller 端 transformPelvisToTorso(q, q_[12], 0, 0) 等价。

    IMU 装在 pelvis，但训练时 obs 的 base 朝向是 torso_link，
    需要左乘腰 yaw 旋转把 pelvis 四元数变到 torso 系。
    """
    q_waist = quat_from_yaw(torso_joint_angle)
    q_torso = quat_mul(pelvis_quat, q_waist)
    n = np.linalg.norm(q_torso)
    return q_torso / max(n, 1e-9)


def matrix_from_quat(q):
    """wxyz -> 3x3 旋转矩阵。"""
    r, i, j, k = q
    two_s = 2.0 / (r * r + i * i + j * j + k * k)
    return np.array([
        [1 - two_s * (j * j + k * k), two_s * (i * j - k * r),       two_s * (i * k + j * r)],
        [two_s * (i * j + k * r),     1 - two_s * (i * i + k * k),   two_s * (j * k - i * r)],
        [two_s * (i * k - j * r),     two_s * (j * k + i * r),       1 - two_s * (i * i + j * j)],
    ])


# ═══════════════════════════════════════════════════════════════════
# 策略推理
# ═══════════════════════════════════════════════════════════════════
def run_onnx_policy(session, obs_np, time_step_np=None):
    input_dict = {}
    for inp in session.get_inputs():
        if inp.name == "obs":
            input_dict[inp.name] = obs_np.astype(np.float32)
        elif inp.name == "time_step":
            if time_step_np is None:
                input_dict[inp.name] = np.zeros((obs_np.shape[0], 1), dtype=np.float32)
            else:
                input_dict[inp.name] = time_step_np.astype(np.float32)
        else:
            input_dict[inp.name] = obs_np.astype(np.float32)
    return session.run(None, input_dict)[0]


def load_policy_metadata(policy_path):
    """从 onnx 中读取 joint_names / default_pos / kp / kd / action_scale。"""
    model = onnx.load(policy_path)
    meta = {p.key: p.value for p in model.metadata_props}

    joint_seq = meta["joint_names"].split(",")
    default_pos_seq = np.array([float(x) for x in meta["default_joint_pos"].split(",")])
    kp_seq = np.array([float(x) for x in meta["joint_stiffness"].split(",")])
    kd_seq = np.array([float(x) for x in meta["joint_damping"].split(",")])
    scale_seq = np.array([float(x) for x in meta["action_scale"].split(",")])
    return joint_seq, default_pos_seq, kp_seq, kd_seq, scale_seq


def remap_array(values, src_names, dst_names):
    """将 values 按 src_names 顺序重排为 dst_names 顺序。"""
    idx = [src_names.index(n) for n in dst_names]
    return np.asarray(values)[idx]


def build_observation(motion_pos, motion_vel, motion_quat,
                      robot_quat, robot_omega,
                      qpos_seq, qvel_seq, default_pos_seq,
                      action_buffer):
    """与 sim2sim 中 create_observation 的 else 分支等价。

    Layout (114):
      omega(3) | motion_ref_ori_b(6) | qpos-default(21) | qvel(21) | actions(21) | motioninput(42)
    """
    # motion_ref_ori_b: 用相对四元数旋转矩阵的前两列 (6 维)
    q_robot_inv = quat_inv(robot_quat)
    q_rel = quat_mul(q_robot_inv, motion_quat)
    mat = matrix_from_quat(q_rel)
    motion_ref_ori_b = mat[:, :2].reshape(6)

    obs = np.zeros(NUM_OBS, dtype=np.float32)
    off = 0
    obs[off:off + 3] = robot_omega;                              off += 3
    obs[off:off + 6] = motion_ref_ori_b;                         off += 6
    obs[off:off + NUM_ACTIONS] = qpos_seq - default_pos_seq;     off += NUM_ACTIONS
    obs[off:off + NUM_ACTIONS] = qvel_seq;                       off += NUM_ACTIONS
    obs[off:off + NUM_ACTIONS] = action_buffer;                  off += NUM_ACTIONS
    obs[off:off + NUM_ACTIONS] = motion_pos;                     off += NUM_ACTIONS
    obs[off:off + NUM_ACTIONS] = motion_vel
    return obs


# ═══════════════════════════════════════════════════════════════════
# 关节指令下发
# ═══════════════════════════════════════════════════════════════════
def send_joint_targets(lc, joint_names, target_pos, kp, kd):
    """按 joint_names 顺序下发；target_pos/kp/kd 都是与 joint_names 同长的数组。"""
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
def run(motion_file, policy_path, loop_motion=False, dry_run=False,
        start_frame=0, end_frame=None, motion_fps=50.0):
    # ── 加载参考动作 ───────────────────────────────────────────────
    motion = np.load(motion_file)
    ref_pos  = motion["joint_pos"]   # (T, 21) 列顺序 = 策略 joint_seq
    ref_vel  = motion["joint_vel"]   # (T, 21)
    ref_bquat = motion["body_quat_w"]  # (T, n_body, 4)  wxyz
    motion_body_idx = 3              # torso_link
    total_frames = min(ref_pos.shape[0], ref_vel.shape[0], ref_bquat.shape[0])

    # 解析 [start_frame, end_frame) 区间
    if start_frame < 0:
        start_frame = 0
    if end_frame is None or end_frame > total_frames:
        end_frame = total_frames
    if end_frame <= start_frame:
        print(f"[ERROR] invalid frame range [{start_frame}, {end_frame})")
        return
    num_frames = end_frame - start_frame

    # 每个控制步前进多少 npz 帧：motion_fps / CONTROL_HZ
    # 例如 motion=200Hz, control=50Hz → frame_step=4
    frame_step = motion_fps / CONTROL_HZ
    print(f"[INFO] Motion total={total_frames}, range=[{start_frame}, {end_frame}),"
          f" len={num_frames}, motion_fps={motion_fps}, control_hz={CONTROL_HZ},"
          f" frame_step={frame_step:.3f}")

    # ── 加载策略 + 元数据 ──────────────────────────────────────────
    joint_seq, default_pos_seq, kp_seq, kd_seq, scale_seq = load_policy_metadata(policy_path)
    # 校验所有策略关节都在 SDK 映射表中
    missing = [j for j in joint_seq if j not in JOINT_NAME_TO_SDK]
    if missing:
        print(f"[ERROR] 以下关节没有 SDK 映射：{missing}")
        return
    # 校验 xml 名字集合与 policy 名字集合一致
    if set(joint_seq) != set(JOINT_NAMES_XML):
        only_in_seq = set(joint_seq) - set(JOINT_NAMES_XML)
        only_in_xml = set(JOINT_NAMES_XML) - set(joint_seq)
        print(f"[ERROR] joint name 集合不一致 only_in_policy={only_in_seq} only_in_xml={only_in_xml}")
        return

    # ── 排序映射：seq ↔ xml ────────────────────────────────────────
    # idx_seq2xml[i] : xml 顺序中第 i 个关节，对应 policy 顺序的下标
    #     → 取 policy 数组到 xml 顺序: arr_xml = arr_seq[idx_seq2xml]
    # idx_xml2seq[i] : policy 顺序中第 i 个关节，对应 xml 顺序的下标
    #     → 取 xml 数组到 policy 顺序: arr_seq = arr_xml[idx_xml2seq]
    idx_seq2xml = np.array([joint_seq.index(n) for n in JOINT_NAMES_XML], dtype=np.int64)
    idx_xml2seq = np.array([JOINT_NAMES_XML.index(n) for n in joint_seq], dtype=np.int64)

    # 把策略侧的 default/kp/kd/scale 提前 remap 到 xml 顺序（电机下发用）
    default_pos_xml = default_pos_seq[idx_seq2xml]
    kp_xml          = kp_seq[idx_seq2xml]
    kd_xml          = kd_seq[idx_seq2xml]

    print(f"[INFO] joint_seq (policy order) = {joint_seq}")
    print(f"[INFO] JOINT_NAMES_XML (mujoco order) =")
    for i, name in enumerate(JOINT_NAMES_XML):
        ctype, jid = JOINT_NAME_TO_SDK[name]
        seq_idx = joint_seq.index(name)
        print(f"  xml[{i:2d}] = {name:<32s} → SDK(comp={ctype}, jid={jid})  "
              f"policy_idx={seq_idx:2d}  kp={kp_xml[i]:.1f}  kd={kd_xml[i]:.2f}  "
              f"default={default_pos_xml[i]:+.3f}")

    policy = onnxruntime.InferenceSession(policy_path)

    # ── 建立 LCM ─────────────────────────────────────────────────
    lc = lcm.LCM(LCM_URL)
    state = RobotState()
    setup_lcm(lc, state)

    print("[INFO] Waiting for IMU + JointsData ...")
    t0 = time.time()
    while time.time() - t0 < 5.0:
        lc.handle_timeout(10)   # 10ms，等数据
        if state.imu_cnt > 0 and state.joint_cnt > 0:
            break
    if state.imu_cnt == 0 or state.joint_cnt == 0:
        print(f"[ERROR] no data. imu={state.imu_cnt} joint={state.joint_cnt}. "
              f"检查多播路由和机器人侧 lumos_controller / SDK 模式是否就绪。")
        return
    print(f"[INFO] OK. imu={state.imu_cnt} joint={state.joint_cnt}")

    if dry_run:
        print("[WARN] DRY-RUN: 仅推理策略，不下发关节指令")
    else:
        print("[WARN] 即将下发关节指令，请确认已进入 SDK 模式且机器人已 STAND。3s 后开始 ...")
        time.sleep(3.0)

    # ── 控制循环：外层紧循环消费 LCM，内层按时间门触发 50Hz 推理 ─────
    # 关键：不要在这里 time.sleep()，否则 LCM 消息会堆积，
    # 醒来后被瞬间排干，导致一次性连发多帧 send_joint_targets（机器人乱动）。
    # `lc.handle_timeout(INNER_TIMEOUT_MS)` 已经提供了「最多等 ms」的天然节流。
    INNER_TIMEOUT_MS = 2   # 每次最多阻塞 2ms 等消息（来消息会更早返回）

    dt = 1.0 / CONTROL_HZ
    action_buffer = np.zeros(NUM_ACTIONS, dtype=np.float32)
    yaw_offset_quat = None
    step = 0

    # 推理速度统计
    infer_times = []
    loop_iters = 0

    t_start = time.perf_counter()
    next_infer_time = t_start   # 下一次允许推理的时刻（绝对 perf_counter）

    try:
        while True:
            # ── 紧循环消费 LCM 消息（保持状态新鲜） ──
            lc.handle_timeout(INNER_TIMEOUT_MS)
            loop_iters += 1

            # ── 时间门：只有到 50Hz tick 才推理 + 下发 ──
            now = time.perf_counter()
            if now < next_infer_time:
                continue

            # 计算当前应使用的 npz 帧
            local_frame = step * frame_step
            if not loop_motion and local_frame >= num_frames:
                print("[INFO] Motion finished.")
                break
            local_int = int(local_frame) % num_frames if loop_motion else int(local_frame)
            idx = start_frame + local_int

            # 1) 取状态 — LCM 数据按 mujoco 顺序读
            quat, omega = state.snapshot_imu()
            qpos_xml, qvel_xml = state.snapshot_joints_by_names(JOINT_NAMES_XML)
            # 转成 policy 顺序喂策略
            qpos_seq = qpos_xml[idx_xml2seq]
            qvel_seq = qvel_xml[idx_xml2seq]

            # IMU 装在 pelvis，但训练 obs 用的是 torso_link 朝向
            # 与 controller 一致：torso_q = transformPelvisToTorso(imu_q, q_[12], 0, 0)
            # torso_joint 在 JOINT_NAMES_XML 里的 index = 12
            torso_q = transform_pelvis_to_torso(quat, qpos_xml[12])

            # 2) yaw 对齐（用 torso_q，与 controller 一致）
            ref_quat_raw = ref_bquat[idx, motion_body_idx]
            if yaw_offset_quat is None:
                yaw_ref   = extract_yaw_quat(ref_quat_raw)
                yaw_robot = extract_yaw_quat(torso_q)
                yaw_offset_quat = quat_mul(yaw_robot, quat_conjugate(yaw_ref))
                yaw_offset_quat /= max(np.linalg.norm(yaw_offset_quat), 1e-9)
                print(f"[YAW] ref={ref_quat_raw}  pelvis={quat}  torso={torso_q}  offset={yaw_offset_quat}")
            corrected_ref_quat = quat_mul(yaw_offset_quat, ref_quat_raw)
            corrected_ref_quat /= max(np.linalg.norm(corrected_ref_quat), 1e-9)

            # 3) 构造 observation（全部 policy 顺序）
            #    robot_quat 用 torso_q（与训练侧一致），omega 用 pelvis IMU（与 controller 一致）
            obs = build_observation(
                motion_pos=ref_pos[idx],
                motion_vel=ref_vel[idx],
                motion_quat=corrected_ref_quat,
                robot_quat=torso_q,
                robot_omega=omega,
                qpos_seq=qpos_seq,
                qvel_seq=qvel_seq,
                default_pos_seq=default_pos_seq,
                action_buffer=action_buffer,
            )

            # 4) 推理（计时）
            t_infer_start = time.perf_counter()
            action = run_onnx_policy(
                policy,
                obs.reshape(1, -1),
                np.array([[float(idx)]], dtype=np.float32),
            )[0].reshape(-1)
            t_infer_ms = (time.perf_counter() - t_infer_start) * 1000.0
            infer_times.append(t_infer_ms)
            action_buffer = action.copy()

            # 5) target：policy 顺序 → xml 顺序
            target_seq = default_pos_seq + action * scale_seq
            target_xml = target_seq[idx_seq2xml]

            # 6) 下发（mujoco/xml 顺序）
            if not dry_run:
                send_joint_targets(lc, JOINT_NAMES_XML, target_xml, kp_xml, kd_xml)

            # 7) 打印（每 50 个控制步 = 每秒一次）
            if step > 0 and step % 50 == 0:
                avg_infer = sum(infer_times) / len(infer_times)
                max_infer = max(infer_times)
                elapsed   = time.perf_counter() - t_start
                actual_hz = step / max(elapsed, 1e-6)
                # 外层循环次数 / 控制步数 = 每个控制步内消费的 LCM iter 数
                iters_per_step = loop_iters / step
                print(f"[STEP] step={step:4d} frame={idx}/{end_frame}  "
                      f"hz={actual_hz:.1f}  "
                      f"infer_avg={avg_infer:.2f}ms  infer_max={max_infer:.2f}ms  "
                      f"lcm_iters/step={iters_per_step:.1f}  "
                      f"target_xml[:3]={target_xml[:3]}")
                infer_times.clear()

            step += 1

            # 8) 推进下一次推理时刻（绝对节拍，避免漂移）
            next_infer_time += dt
            # 如果已经超时（推理太慢 / 卡顿），重置避免连发追帧
            if time.perf_counter() > next_infer_time + dt:
                overrun_ms = (time.perf_counter() - next_infer_time) * 1000.0
                print(f"[WARN] control loop overrun by {overrun_ms:.1f} ms, resetting tick")
                next_infer_time = time.perf_counter()

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
    finally:
        print("[INFO] Exit. (注意：本脚本不会自动退出 SDK 模式 / 切 RESET，"
              "请用 sdk_debug.py 切回安全状态)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--motion_file", required=True, help="参考动作 npz")
    parser.add_argument("--policy_path", required=True, help="ONNX 策略文件")
    parser.add_argument("--loop", action="store_true", help="循环回放动作（在 [start,end) 区间内循环）")
    parser.add_argument("--dry_run", action="store_true",
                        help="只推理不下发，便于检查数据流")
    parser.add_argument("--start_frame", type=int, default=0,
                        help="参考动作开始帧 (默认 0)")
    parser.add_argument("--end_frame", type=int, default=None,
                        help="参考动作结束帧（不含），默认为动作末尾")
    parser.add_argument("--motion_fps", type=float, default=50.0,
                        help="参考动作 npz 的采样率 (Hz)。控制频率固定 50Hz，"
                             "若 motion 是 200Hz 采样则填 200，会按 4 帧/步推进。")
    args = parser.parse_args()
    run(args.motion_file, args.policy_path, args.loop, args.dry_run,
        args.start_frame, args.end_frame, args.motion_fps)


if __name__ == "__main__":
    main()
