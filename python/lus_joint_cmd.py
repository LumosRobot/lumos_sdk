#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LUS2 27 自由度 SDK 关节指令下发工具。

这个脚本通过 LCM 发布 ``sdk_lcmt_joint_cmds``，用于 LUS2/LUS 机器人
27 个关节的 Python SDK 控制：

    LEG_L 6 + LEG_R 6 + WAIST 1 + ARM_L 7 + ARM_R 7 = 27

安全边界：
    - 本脚本只发布关节指令，不负责进入或退出 SDK 模式。
    - 发布前先让机器人进入 RESET/STAND，并进入 SDK 模式，例如：
      ``python3 lumos_sdk/python/sdk_debug.py mode 1``。
    - 第一次操作必须先加 ``--dry-run``，确认 component、joint_id、目标位置和增益。
    - 真机发布时从小幅度、短时间、单关节开始，不要直接回放长动作。

常用示例：
    # 打印 27 个全局索引和组件内 joint_id 映射。
    python3 lumos_sdk/python/lus_joint_cmd.py list

    # 只预览左腕 yaw 指令，不发布 LCM。
    python3 lumos_sdk/python/lus_joint_cmd.py single \\
      --component ARM_L --joint-id 4 --pos 0.2 --kp 80 --kd 2 --dry-run

    # 按全局索引发布，17 表示 ARM_L[4] / left_wrist_yaw_joint。
    python3 lumos_sdk/python/lus_joint_cmd.py global \\
      --index 17 --pos 0.2 --kp 80 --kd 2 --duration 1

    # 一个 LCM 包里同时下发多个关节目标。
    python3 lumos_sdk/python/lus_joint_cmd.py batch \\
      --target ARM_L:4:0.20:80:2 \\
      --target ARM_R:4:-0.20:80:2

    # 下发 LUS2 配置里的 27 关节站姿。
    python3 lumos_sdk/python/lus_joint_cmd.py stand --dry-run

    # 回放动作文件：默认从每行第 0 列开始读取 27 个关节位置。
    python3 lumos_sdk/python/lus_joint_cmd.py replay \\
      --model-dir path/to/lus_model_dir --motion-start-col 0 --dry-run

    # 正弦激励：先读当前关节位置，再围绕当前位置做单关节正弦。
    python3 lumos_sdk/python/lus_joint_cmd.py sine-sweep \\
      --component ARM_L --joint-id 4 --amplitude 0.1 --duration 5 --dry-run
"""

from __future__ import annotations

import argparse
import csv
import importlib
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=255"
LOCAL_LCM_URL = "udpm://239.255.76.67:7667?ttl=0"
DEFAULT_COMMAND_CHANNEL = "lcm_joint_cmd"

# 与 controller/include/biz/sdk_base_define.hpp 的 SdkComponentType 对齐。
COMPONENTS = {
    "ARM_L": 1,
    "ARM_R": 2,
    "WAIST": 7,
    "LEG_L": 8,
    "LEG_R": 9,
}
COMPONENT_NAMES = {value: key for key, value in COMPONENTS.items()}

# LUS2 27 自由度 SDK 下发/反馈顺序：
# LEG_L(6), LEG_R(6), WAIST(1), ARM_L(7), ARM_R(7)。
# 这个顺序与 lumos_controller/config/robots/lus2/robot_state_param_stand.yaml
# 和 robot_state_param_dance_bfm27.yaml 中的 robot_joint_names 保持一致。
GLOBAL_JOINTS: Tuple[Tuple[str, int, str], ...] = (
    ("LEG_L", 0, "left_hip_pitch_joint"),
    ("LEG_L", 1, "left_hip_roll_joint"),
    ("LEG_L", 2, "left_hip_yaw_joint"),
    ("LEG_L", 3, "left_knee_joint"),
    ("LEG_L", 4, "left_ankle_pitch_joint"),
    ("LEG_L", 5, "left_ankle_roll_joint"),
    ("LEG_R", 0, "right_hip_pitch_joint"),
    ("LEG_R", 1, "right_hip_roll_joint"),
    ("LEG_R", 2, "right_hip_yaw_joint"),
    ("LEG_R", 3, "right_knee_joint"),
    ("LEG_R", 4, "right_ankle_pitch_joint"),
    ("LEG_R", 5, "right_ankle_roll_joint"),
    ("WAIST", 0, "torso_joint"),
    ("ARM_L", 0, "left_shoulder_pitch_joint"),
    ("ARM_L", 1, "left_shoulder_roll_joint"),
    ("ARM_L", 2, "left_shoulder_yaw_joint"),
    ("ARM_L", 3, "left_elbow_joint"),
    ("ARM_L", 4, "left_wrist_yaw_joint"),
    ("ARM_L", 5, "left_wrist_pitch_joint"),
    ("ARM_L", 6, "left_wrist_roll_joint"),
    ("ARM_R", 0, "right_shoulder_pitch_joint"),
    ("ARM_R", 1, "right_shoulder_roll_joint"),
    ("ARM_R", 2, "right_shoulder_yaw_joint"),
    ("ARM_R", 3, "right_elbow_joint"),
    ("ARM_R", 4, "right_wrist_yaw_joint"),
    ("ARM_R", 5, "right_wrist_pitch_joint"),
    ("ARM_R", 6, "right_wrist_roll_joint"),
)

JOINT_NAME_TO_GLOBAL = {name: idx for idx, (_, _, name) in enumerate(GLOBAL_JOINTS)}
REPLAY_FIELD_COUNT = len(GLOBAL_JOINTS)

# 来自 lumos_controller/config/robots/lus2/robot_state_param_stand.yaml。
LUS2_STAND_POS = (
    -0.37, 0.0, 0.0, 0.74, -0.37, 0.0,
    -0.37, 0.0, 0.0, 0.74, -0.37, 0.0,
    0.0,
    0.0, 0.25, 0.0, 1.2, 0.0, 0.0, 0.0,
    0.0, -0.25, 0.0, 1.2, 0.0, 0.0, 0.0,
)
LUS2_STAND_KP = (
    600.0, 600.0, 600.0, 600.0, 400.0, 400.0,
    600.0, 600.0, 600.0, 600.0, 400.0, 400.0,
    600.0,
    400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0,
    400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0,
)
LUS2_STAND_KD = (
    6.0, 2.0, 6.0, 6.0, 6.0, 6.0,
    6.0, 2.0, 6.0, 6.0, 6.0, 6.0,
    6.0,
    4.0, 4.0, 4.0, 4.0, 0.67, 0.67, 0.67,
    4.0, 4.0, 4.0, 4.0, 0.67, 0.67, 0.67,
)

@dataclass(frozen=True)
class JointTarget:
    component_type: int
    joint_id: int
    pos: float
    kp: float = 60.0
    kd: float = 2.0
    vel: float = 0.0
    tor: float = 0.0
    cur: float = 0.0
    ctrl_word: int = 3

    @property
    def component_name(self) -> str:
        return COMPONENT_NAMES.get(self.component_type, f"COMP_{self.component_type}")

    @property
    def joint_name(self) -> str:
        for component, joint_id, name in GLOBAL_JOINTS:
            if COMPONENTS[component] == self.component_type and joint_id == self.joint_id:
                return name
        return f"{self.component_name}[{self.joint_id}]"

    def summary(self) -> str:
        return (
            f"{self.component_name}[{self.joint_id}] {self.joint_name} "
            f"pos={self.pos:.4f} vel={self.vel:.4f} tor={self.tor:.4f} "
            f"kp={self.kp:.2f} kd={self.kd:.2f} ctrl={self.ctrl_word}"
        )


def _sdk_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _typedef_dir() -> Path:
    return _sdk_root() / "lcm_typedef" / "python"


def _load_lcm_class(modname: str):
    typedef_path = str(_typedef_dir())
    if typedef_path not in sys.path:
        sys.path.insert(0, typedef_path)
    module = importlib.import_module(modname)
    cls = getattr(module, modname, module)
    sys.modules[modname] = cls
    return cls


def load_lcm_runtime():
    try:
        lcm_mod = importlib.import_module("lcm")
    except ImportError as exc:
        raise RuntimeError(
            "Python LCM binding 不可用。请安装/编译 lcm Python 包，"
            "或先使用 --dry-run 只检查指令内容。"
        ) from exc
    joint_cmd = _load_lcm_class("sdk_lcmt_joint_cmd")
    joint_cmds = _load_lcm_class("sdk_lcmt_joint_cmds")
    return lcm_mod, joint_cmd, joint_cmds


def parse_component(value: str) -> int:
    raw = str(value).strip()
    key = raw.upper()
    if key in COMPONENTS:
        return COMPONENTS[key]
    try:
        component_type = int(raw)
    except ValueError as exc:
        valid = ", ".join(COMPONENTS)
        raise argparse.ArgumentTypeError(f"未知组件 {value!r}；应为 {valid} 或数字编号") from exc
    if component_type not in COMPONENT_NAMES:
        raise argparse.ArgumentTypeError(f"未知数字组件编号：{component_type}")
    return component_type


def target_from_global_index(
    index: int,
    pos: float,
    kp: float,
    kd: float,
    vel: float,
    tor: float,
    ctrl_word: int,
) -> JointTarget:
    if index < 0 or index >= len(GLOBAL_JOINTS):
        raise ValueError(f"全局关节索引必须在 [0, {len(GLOBAL_JOINTS) - 1}] 内，当前为 {index}")
    component, joint_id, _name = GLOBAL_JOINTS[index]
    return JointTarget(COMPONENTS[component], joint_id, pos, kp=kp, kd=kd, vel=vel, tor=tor, ctrl_word=ctrl_word)


def parse_target_spec(spec: str) -> JointTarget:
    parts = [part.strip() for part in str(spec).split(":")]
    if len(parts) < 3:
        raise argparse.ArgumentTypeError(
            "target 格式必须是 COMPONENT:JOINT_ID:POS[:KP[:KD[:VEL[:TOR[:CTRL_WORD]]]]]"
        )
    try:
        component_type = parse_component(parts[0])
        joint_id = int(parts[1])
        pos = float(parts[2])
        kp = float(parts[3]) if len(parts) > 3 and parts[3] else 60.0
        kd = float(parts[4]) if len(parts) > 4 and parts[4] else 2.0
        vel = float(parts[5]) if len(parts) > 5 and parts[5] else 0.0
        tor = float(parts[6]) if len(parts) > 6 and parts[6] else 0.0
        ctrl_word = int(parts[7]) if len(parts) > 7 and parts[7] else 3
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"无效 target {spec!r}: {exc}") from exc
    return JointTarget(component_type, joint_id, pos, kp=kp, kd=kd, vel=vel, tor=tor, ctrl_word=ctrl_word)


def validate_target(target: JointTarget) -> None:
    """检查目标是否是 LUS2 已知关节，不检查位置限位。"""

    known_pair = any(
        COMPONENTS[component] == target.component_type and joint_id == target.joint_id
        for component, joint_id, _name in GLOBAL_JOINTS
    )
    if not known_pair:
        raise ValueError(f"无效 LUS2 关节：{target.component_name}[{target.joint_id}]")


def load_targets_csv(path: str) -> List[JointTarget]:
    targets: List[JointTarget] = []
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required = {"component", "joint_id", "pos"}
        missing = sorted(required - set(reader.fieldnames or []))
        if missing:
            raise ValueError(f"target CSV 缺少必填列：{', '.join(missing)}")
        for line_no, row in enumerate(reader, start=2):
            if not any((value or "").strip() for value in row.values()):
                continue
            try:
                targets.append(
                    JointTarget(
                        component_type=parse_component(row["component"]),
                        joint_id=int(row["joint_id"]),
                        pos=float(row["pos"]),
                        kp=float(row.get("kp") or 60.0),
                        kd=float(row.get("kd") or 2.0),
                        vel=float(row.get("vel") or 0.0),
                        tor=float(row.get("tor") or 0.0),
                        cur=float(row.get("cur") or 0.0),
                        ctrl_word=int(row.get("ctrl_word") or 3),
                    )
                )
            except (ValueError, argparse.ArgumentTypeError) as exc:
                raise ValueError(f"target CSV 第 {line_no} 行无效：{exc}") from exc
    if not targets:
        raise ValueError(f"target CSV 没有可用数据行：{path}")
    return targets


def load_kp_kd_yaml(path: str) -> Tuple[List[float], List[float]]:
    """从模型目录的 ``kp_kd.yaml`` 读取 27 个 ``kps`` 和 27 个 ``kds``。

    与 ``nix_joint_cmd.py`` 保持一致，这里不用 PyYAML，只解析当前模型目录
    常见的简单列表格式，避免给 SDK 调试脚本增加额外依赖。
    """

    values = {"kps": [], "kds": []}
    section: Optional[str] = None
    with open(path, encoding="utf-8") as handle:
        for line_no, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if line.endswith(":"):
                key = line[:-1]
                section = key if key in values else None
                continue
            if section and line.startswith("-"):
                try:
                    values[section].append(float(line[1:].strip()))
                except ValueError as exc:
                    raise ValueError(f"{path}:{line_no} 中 {section} 数值无效：{line!r}") from exc

    kps = values["kps"]
    kds = values["kds"]
    if len(kps) != len(GLOBAL_JOINTS) or len(kds) != len(GLOBAL_JOINTS):
        raise ValueError(
            f"{path} 中 kp/kd 数量无效：kps={len(kps)} kds={len(kds)} "
            f"期望各 {len(GLOBAL_JOINTS)} 个"
        )
    return kps, kds


def load_ref_motion(path: str, min_fields: int, max_frames: int = 0) -> List[List[float]]:
    """读取参考动作帧，每行至少需要 ``min_fields`` 个浮点列。"""

    frames: List[List[float]] = []
    with open(path, encoding="utf-8") as handle:
        for line_no, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line:
                continue
            try:
                row = [float(value) for value in line.split()]
            except ValueError as exc:
                raise ValueError(f"{path}:{line_no} 中存在无效浮点数") from exc
            if len(row) < min_fields:
                raise ValueError(
                    f"动作文件第 {line_no} 行只有 {len(row)} 列，至少需要 {min_fields} 列"
                )
            frames.append(row)
            if max_frames > 0 and len(frames) >= max_frames:
                break
    if not frames:
        raise ValueError(f"动作文件没有可用帧：{path}")
    return frames


def replay_targets_from_frame(
    frame: Sequence[float],
    kps: Sequence[float],
    kds: Sequence[float],
    motion_start_col: int = 0,
) -> List[JointTarget]:
    """把一行参考动作转换成 27 个 SDK 关节目标。

    LUS2 没有像 NIX C++ 示例里那种固定 45 列参考动作映射；这里默认假设
    动作文件从 ``motion_start_col`` 开始，按 ``GLOBAL_JOINTS`` 顺序连续存放
    27 个关节位置。
    """

    targets: List[JointTarget] = []
    for idx, (component, joint_id, _name) in enumerate(GLOBAL_JOINTS):
        motion_col = motion_start_col + idx
        targets.append(
            JointTarget(
                component_type=COMPONENTS[component],
                joint_id=joint_id,
                pos=float(frame[motion_col]),
                kp=float(kps[idx]),
                kd=float(kds[idx]),
            )
        )
    return targets


def build_lcm_message(targets: Sequence[JointTarget], joint_cmd_cls, joint_cmds_cls):
    msg = joint_cmds_cls()
    msg.cmds_num = len(targets)
    msg.cmds = []
    for target in targets:
        cmd = joint_cmd_cls()
        cmd.component_type = int(target.component_type)
        cmd.joint_id = int(target.joint_id)
        cmd.ctrlWord = int(target.ctrl_word)
        cmd.tarPos = float(target.pos)
        cmd.tarVel = float(target.vel)
        cmd.tarCur = float(target.cur)
        cmd.tarTor = float(target.tor)
        cmd.res1 = float(target.kp)
        cmd.res2 = float(target.kd)
        cmd.res3 = 0.0
        cmd.res4 = 0.0
        msg.cmds.append(cmd)
    return msg


def resolve_publish_count(count: Optional[int], duration: float, rate_hz: float) -> int:
    if count is not None:
        if count <= 0:
            raise ValueError("--count 必须为正数")
        return int(count)
    if duration > 0:
        if rate_hz <= 0:
            raise ValueError("设置 --duration 时 --rate-hz 必须为正数")
        return max(1, int(math.ceil(duration * rate_hz)))
    return 1


def publish_targets(args: argparse.Namespace, targets: Sequence[JointTarget]) -> int:
    if not targets:
        raise ValueError("没有可发布的关节目标")
    for target in targets:
        validate_target(target)
        print(f"目标：{target.summary()}")

    if args.dry_run:
        print("dry-run：只打印目标，不发布 LCM 包")
        return 0

    lcm_mod, joint_cmd_cls, joint_cmds_cls = load_lcm_runtime()
    lcm_url = LOCAL_LCM_URL if args.local and args.url is None else (args.url or DEFAULT_LCM_URL)
    lc = lcm_mod.LCM(lcm_url)
    msg = build_lcm_message(targets, joint_cmd_cls, joint_cmds_cls)

    count = resolve_publish_count(args.count, args.duration, args.rate_hz)
    delay = 0.0 if args.rate_hz <= 0 else 1.0 / args.rate_hz
    for idx in range(count):
        lc.publish(args.channel, msg.encode())
        if args.verbose:
            print(f"已发布 {idx + 1}/{count} channel={args.channel} targets={len(targets)}")
        if delay > 0 and idx + 1 < count:
            time.sleep(delay)

    if not args.verbose:
        print(f"已发布 {count} 个包 channel={args.channel} targets={len(targets)}")
    return 0


def publish_replay(args: argparse.Namespace) -> int:
    """把模型参考动作回放为 27 关节 SDK 指令包。"""

    model_dir = Path(args.model_dir)
    motion_path = Path(args.motion) if args.motion else model_dir / "store_ref_motion.txt"
    kpkd_path = Path(args.kp_kd) if args.kp_kd else model_dir / "kp_kd.yaml"
    motion_start_col = int(args.motion_start_col)
    if motion_start_col < 0:
        raise ValueError("--motion-start-col 必须 >= 0")

    kps, kds = load_kp_kd_yaml(str(kpkd_path))
    frames = load_ref_motion(
        str(motion_path),
        min_fields=motion_start_col + REPLAY_FIELD_COUNT,
        max_frames=args.max_frames,
    )
    loops = int(args.loops)

    first_targets = replay_targets_from_frame(frames[0], kps, kds, motion_start_col=motion_start_col)
    last_targets = replay_targets_from_frame(frames[-1], kps, kds, motion_start_col=motion_start_col)
    total_packets = None if loops <= 0 else len(frames) * loops
    print(f"模型目录：{model_dir}")
    print(f"动作文件：{motion_path}")
    print(f"增益文件：{kpkd_path}")
    print(f"动作列：从第 {motion_start_col} 列开始读取 {REPLAY_FIELD_COUNT} 个关节位置")
    loop_text = "无限" if loops <= 0 else str(loops)
    packet_text = "无限" if total_packets is None else str(total_packets)
    print(f"帧数：{len(frames)} loops={loop_text} rate={args.rate_hz:g}Hz packets={packet_text}")
    print("第一帧：")
    print_replay_preview(first_targets, limit=args.preview_joints)
    print("最后一帧：")
    print_replay_preview(last_targets, limit=args.preview_joints)

    if args.dry_run:
        print("dry-run：只打印回放预览，不发布 LCM 包")
        return 0
    if args.rate_hz <= 0:
        raise ValueError("replay 模式下 --rate-hz 必须为正数")

    lcm_mod, joint_cmd_cls, joint_cmds_cls = load_lcm_runtime()
    lcm_url = LOCAL_LCM_URL if args.local and args.url is None else (args.url or DEFAULT_LCM_URL)
    lc = lcm_mod.LCM(lcm_url)
    delay = 1.0 / args.rate_hz
    sent = 0
    completed_loops = 0
    next_tick = time.monotonic()
    try:
        while loops <= 0 or completed_loops < loops:
            for frame in frames:
                if loops > 0 and completed_loops >= loops:
                    break
                targets = replay_targets_from_frame(frame, kps, kds, motion_start_col=motion_start_col)
                msg = build_lcm_message(targets, joint_cmd_cls, joint_cmds_cls)
                lc.publish(args.channel, msg.encode())
                sent += 1
                if args.verbose and (sent == 1 or sent % args.print_every == 0):
                    if total_packets is None:
                        print(f"已发布 {sent} 个包，loop={completed_loops + 1}")
                    else:
                        print(f"已发布 {sent}/{total_packets} loop={completed_loops + 1}/{loops}")
                next_tick += delay
                sleep_for = next_tick - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_tick = time.monotonic()
            completed_loops += 1
    except KeyboardInterrupt:
        print("\n已中断：停止回放发布")
    if not args.verbose:
        print(f"已发布 {sent} 个回放包 channel={args.channel}")
    return 0


def print_replay_preview(targets: Sequence[JointTarget], limit: int) -> None:
    """打印有限数量的回放目标，避免 dry-run 输出过长。"""

    shown = targets[:max(0, limit)]
    for target in shown:
        print(f"  {target.summary()}")
    remaining = len(targets) - len(shown)
    if remaining > 0:
        print(f"  ... 还有 {remaining} 个关节")


def build_stand_targets() -> List[JointTarget]:
    targets: List[JointTarget] = []
    for idx, (pos, kp, kd) in enumerate(zip(LUS2_STAND_POS, LUS2_STAND_KP, LUS2_STAND_KD)):
        component, joint_id, _name = GLOBAL_JOINTS[idx]
        targets.append(JointTarget(COMPONENTS[component], joint_id, pos, kp=kp, kd=kd))
    return targets


def localize_argparse(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    parser._positionals.title = "位置参数"  # pylint: disable=protected-access
    parser._optionals.title = "选项"  # pylint: disable=protected-access
    for action in parser._actions:  # pylint: disable=protected-access
        if "-h" in action.option_strings and "--help" in action.option_strings:
            action.help = "显示帮助信息并退出"
    return parser


def add_common_publish_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--channel", default=DEFAULT_COMMAND_CHANNEL, help="LCM 关节指令频道")
    parser.add_argument("--url", default=None, help="显式指定 LCM URL；默认使用 SDK 组播地址")
    parser.add_argument("--local", action="store_true", help="使用 ttl=0，适合同机 LCM mock 测试")
    parser.add_argument("--rate-hz", type=float, default=100.0, help="重复发布时的频率")
    parser.add_argument("--duration", type=float, default=0.0, help="按指定秒数重复发布")
    parser.add_argument("--count", type=int, default=None, help="精确发布指定数量的包")
    parser.add_argument("--dry-run", action="store_true", help="只打印目标，不发布 LCM 包")
    parser.add_argument("--verbose", action="store_true", help="打印每次发布进度")


def add_target_shape_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--pos", type=float, required=True, help="目标位置，单位 rad")
    parser.add_argument("--kp", type=float, default=60.0, help="PD 刚度，写入 res1")
    parser.add_argument("--kd", type=float, default=2.0, help="PD 阻尼，写入 res2")
    parser.add_argument("--vel", type=float, default=0.0, help="目标速度")
    parser.add_argument("--tor", type=float, default=0.0, help="前馈力矩")
    parser.add_argument("--ctrl-word", type=int, default=3, help="SDK 控制字；PD 控制通常使用 3")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="通过 Python 下发 LUS2 27 自由度 SDK 关节指令。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "全局索引顺序：0-5 LEG_L，6-11 LEG_R，12 WAIST，"
            "13-19 ARM_L，20-26 ARM_R。\n"
            "安全建议：真机发布前先使用 --dry-run；发布前确认机器人已进入 SDK 模式。"
        ),
    )
    localize_argparse(parser)
    sub = parser.add_subparsers(dest="command", required=True)

    p_list = sub.add_parser("list", help="打印组件编号和全局关节索引映射")
    localize_argparse(p_list)
    p_list.set_defaults(func=cmd_list)

    p_single = sub.add_parser("single", help="按组件内 joint_id 下发单个关节目标")
    localize_argparse(p_single)
    p_single.add_argument("--component", type=parse_component, required=True, help="组件名或数字组件编号")
    p_single.add_argument("--joint-id", type=int, required=True, help="组件内部关节编号")
    add_target_shape_args(p_single)
    add_common_publish_args(p_single)
    p_single.set_defaults(func=cmd_single)

    p_global = sub.add_parser("global", help="按 LUS2 全局 SDK 关节索引下发单个目标")
    localize_argparse(p_global)
    p_global.add_argument("--index", type=int, required=True, help="全局 SDK 关节索引，范围 0-26")
    add_target_shape_args(p_global)
    add_common_publish_args(p_global)
    p_global.set_defaults(func=cmd_global)

    p_batch = sub.add_parser("batch", help="在一个 LCM 包里下发多个命令行目标")
    localize_argparse(p_batch)
    p_batch.add_argument(
        "--target",
        action="append",
        type=parse_target_spec,
        required=True,
        help="目标格式：COMPONENT:JOINT_ID:POS[:KP[:KD[:VEL[:TOR[:CTRL_WORD]]]]]",
    )
    add_common_publish_args(p_batch)
    p_batch.set_defaults(func=cmd_batch)

    p_file = sub.add_parser("file", help="从 CSV 文件读取并下发关节目标")
    localize_argparse(p_file)
    p_file.add_argument("--path", required=True, help="CSV 路径，至少包含 component,joint_id,pos 列")
    add_common_publish_args(p_file)
    p_file.set_defaults(func=cmd_file)

    p_replay = sub.add_parser("replay", help="回放模型目录中的 store_ref_motion.txt + kp_kd.yaml")
    localize_argparse(p_replay)
    p_replay.add_argument(
        "--model-dir",
        required=True,
        help="模型目录，内部应包含 store_ref_motion.txt 和 kp_kd.yaml",
    )
    p_replay.add_argument("--motion", default="", help="覆盖参考动作文件路径")
    p_replay.add_argument("--kp-kd", default="", help="覆盖 kp_kd.yaml 路径")
    p_replay.add_argument("--motion-start-col", type=int, default=0,
                          help="每行动作中 27 个 LUS 关节位置的起始列，默认 0")
    p_replay.add_argument("--loops", type=int, default=1, help="回放循环次数；<=0 表示无限循环直到 Ctrl-C")
    p_replay.add_argument("--max-frames", type=int, default=0, help="只加载前 N 帧，用于检查或小范围测试")
    p_replay.add_argument("--preview-joints", type=int, default=6, help="dry-run 预览时打印的关节数量")
    p_replay.add_argument("--print-every", type=int, default=100, help="verbose 模式下每隔多少包打印一次进度")
    add_common_publish_args(p_replay)
    p_replay.set_defaults(func=cmd_replay)

    p_stand = sub.add_parser("stand", help="下发 LUS2 27 关节站姿目标")
    localize_argparse(p_stand)
    add_common_publish_args(p_stand)
    p_stand.set_defaults(func=cmd_stand)

    p_sweep = sub.add_parser("sine-sweep", help="正弦激励：自动读取当前角度 → 激励 + 采集反馈")
    localize_argparse(p_sweep)
    p_sweep.add_argument("--component", type=parse_component, required=True,
                         help="组件名或编号 (LEG_L=8, LEG_R=9, WAIST=7, ARM_L=1, ARM_R=2)")
    p_sweep.add_argument("--joint-id", type=int, required=True,
                         help="组件内关节编号。LEG: 0-5, WAIST: 0, ARM: 0-6")
    p_sweep.add_argument("--amplitude", type=float, required=True, help="正弦幅值 (rad)")
    p_sweep.add_argument("--freq-hz", type=float, default=0.5, help="正弦频率 (Hz)，默认 0.5")
    p_sweep.add_argument("--duration", type=float, default=10.0, help="持续时间 (s)，默认 10")
    p_sweep.add_argument("--rate-hz", type=float, default=100.0, help="LCM 发布率 (Hz)，默认 100")
    p_sweep.add_argument("--kp", type=float, default=160.0, help="PD 刚度，默认 160")
    p_sweep.add_argument("--kd", type=float, default=6.0, help="PD 阻尼，默认 6")
    p_sweep.add_argument("--csv", default=None, help="反馈 CSV 输出路径，默认 build/phase1/<关节名>_sine_sweep.csv")
    p_sweep.add_argument("--channel", default=DEFAULT_COMMAND_CHANNEL, help="LCM 指令频道")
    p_sweep.add_argument("--url", default=None, help="LCM URL")
    p_sweep.add_argument("--local", action="store_true", help="使用 ttl=0")
    p_sweep.add_argument("--dry-run", action="store_true", help="只读取角度 + 预览轨迹，不发布")
    p_sweep.set_defaults(func=cmd_sine_sweep)

    return parser


def cmd_list(_args: argparse.Namespace) -> int:
    print("组件编号：")
    for name, value in COMPONENTS.items():
        print(f"  {name:<7} {value}")
    print("\n全局 SDK 关节索引：")
    for idx, (component, joint_id, name) in enumerate(GLOBAL_JOINTS):
        print(f"  {idx:2d}: {component}[{joint_id}] {name}")
    return 0


def cmd_single(args: argparse.Namespace) -> int:
    target = JointTarget(
        component_type=args.component,
        joint_id=args.joint_id,
        pos=args.pos,
        kp=args.kp,
        kd=args.kd,
        vel=args.vel,
        tor=args.tor,
        ctrl_word=args.ctrl_word,
    )
    return publish_targets(args, [target])


def cmd_global(args: argparse.Namespace) -> int:
    target = target_from_global_index(args.index, args.pos, args.kp, args.kd, args.vel, args.tor, args.ctrl_word)
    return publish_targets(args, [target])


def cmd_batch(args: argparse.Namespace) -> int:
    return publish_targets(args, args.target)


def cmd_file(args: argparse.Namespace) -> int:
    return publish_targets(args, load_targets_csv(args.path))


def cmd_replay(args: argparse.Namespace) -> int:
    """回放模型参考动作目标。"""

    return publish_replay(args)


def cmd_stand(args: argparse.Namespace) -> int:
    return publish_targets(args, build_stand_targets())


def cmd_sine_sweep(args: argparse.Namespace) -> int:
    """读取当前关节位置 → 正弦激励 + 自动采集反馈 CSV。"""

    from nix_lcm_sub import NixLcmSubscriber, LcmUnavailableError, write_feedback_rows  # noqa: E402

    component_type = parse_component(args.component)
    joint_id = int(args.joint_id)
    probe_target = JointTarget(component_type=component_type, joint_id=joint_id, pos=0.0)
    validate_target(probe_target)
    joint_name = probe_target.joint_name

    print(f"读取当前关节位置 (component={component_type} joint={joint_id}) ...")
    lcm_url = LOCAL_LCM_URL if args.local and args.url is None else (args.url or DEFAULT_LCM_URL)
    try:
        sub = NixLcmSubscriber(lcm_url=lcm_url)
    except LcmUnavailableError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2

    sub.listen(once=True, timeout_ms=500, duration_sec=10.0)
    if sub.message_count == 0:
        print("错误：10 秒内未收到 JointsData，检查机器人是否在线", file=sys.stderr)
        return 3

    current_pos = None
    for sample in sub.last_samples:
        if sample.component_type == component_type and sample.joint_id == joint_id:
            current_pos = sample.pos_high
            break

    if current_pos is None:
        available = set((sample.component_type, sample.joint_id) for sample in sub.last_samples)
        print(
            f"错误：当前 LCM 帧中未找到 component={component_type} joint={joint_id}。"
            f"可用关节: {sorted(available)}",
            file=sys.stderr,
        )
        return 4

    center = float(current_pos)
    amplitude = float(args.amplitude)
    duration = float(args.duration)
    rate_hz = float(args.rate_hz)
    freq_hz = float(args.freq_hz)
    if duration <= 0:
        raise ValueError("--duration 必须为正数")
    if rate_hz <= 0:
        raise ValueError("--rate-hz 必须为正数")

    print(f"  当前角度 = {center:.4f} rad")
    print(f"  不做软件限位检查，轨迹范围 [{center - amplitude:.4f}, {center + amplitude:.4f}]")

    if args.dry_run:
        n = max(1, int(duration * rate_hz))
        print(f"\n轨迹预览：center={center:.4f} ampl={amplitude:.3f} "
              f"freq={freq_hz:.1f}Hz duration={duration:.0f}s "
              f"rate={rate_hz:.0f}Hz packets={n}")
        print(f"{'时间(s)':>8s}  {'位置(rad)':>10s}")
        print(f"{'-' * 8}  {'-' * 10}")
        preview = min(8, n)
        for i in range(preview):
            t = i * duration / (preview - 1) if preview > 1 else 0.0
            pos = center + amplitude * math.sin(2.0 * math.pi * freq_hz * t)
            marker = " <-起始" if i == 0 else (" <-结束" if i == preview - 1 else "")
            print(f"{t:8.3f}  {pos:10.4f}{marker}")
        print("\n[dry-run] 只预览，未发布 LCM 包。")
        return 0

    csv_path = args.csv or f"build/phase1/{joint_name}_sine_sweep.csv"
    Path(csv_path).parent.mkdir(parents=True, exist_ok=True)

    feedback_done = threading.Event()
    feedback_samples: List[dict] = []
    feedback_lock = threading.Lock()

    def _feedback_collector() -> None:
        try:
            fsub = NixLcmSubscriber(lcm_url=lcm_url)
        except LcmUnavailableError:
            return
        deadline = time.monotonic() + duration + 2.0
        while not feedback_done.is_set() and time.monotonic() < deadline:
            try:
                fsub.handle_timeout(timeout_ms=100)
            except Exception:
                break
            if fsub.last_samples:
                with feedback_lock:
                    for sample in fsub.last_samples:
                        feedback_samples.append(sample.feedback_row())
        if feedback_samples:
            with feedback_lock:
                rows = list(feedback_samples)
            write_feedback_rows(csv_path, rows, append=False)
            print(f"\n反馈已保存：{csv_path} ({len(rows)} 行)")

    feedback_thread = threading.Thread(target=_feedback_collector, daemon=True)
    feedback_thread.start()
    time.sleep(0.3)

    n = max(1, int(duration * rate_hz))
    dt = 1.0 / rate_hz
    print(f"\n开始正弦激励：{joint_name} "
          f"center={center:.4f} ampl={amplitude:.3f} "
          f"freq={freq_hz:.1f}Hz duration={duration:.0f}s "
          f"packets={n}")
    print(f"反馈 CSV：{csv_path}")

    lcm_mod, joint_cmd_cls, joint_cmds_cls = load_lcm_runtime()
    lc = lcm_mod.LCM(lcm_url)
    sent = 0
    next_tick = time.monotonic()
    try:
        for i in range(n):
            t = i * dt
            pos = center + amplitude * math.sin(2.0 * math.pi * freq_hz * t)
            target = JointTarget(
                component_type=component_type,
                joint_id=joint_id,
                pos=pos,
                kp=float(args.kp),
                kd=float(args.kd),
                ctrl_word=3,
            )
            msg = build_lcm_message([target], joint_cmd_cls, joint_cmds_cls)
            lc.publish(args.channel, msg.encode())
            sent += 1
            if sent == 1 or sent % 100 == 0:
                pct = sent / n * 100
                print(f"  {pct:5.1f}% ({sent}/{n}) t={t:.2f}s pos={pos:.4f}")
            next_tick += dt
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        print(f"\n已中断 ({sent}/{n})")
    finally:
        feedback_done.set()

    feedback_thread.join(timeout=5.0)
    if feedback_samples:
        print(f"完成。{sent} 个包 -> {csv_path} ({len(feedback_samples)} 行反馈)")
    else:
        print(f"完成。{sent} 个包（未采集到反馈数据，检查 LCM 订阅）")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    try:
        return int(args.func(args))
    except (RuntimeError, ValueError, argparse.ArgumentTypeError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
