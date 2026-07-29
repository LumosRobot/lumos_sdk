#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""NIX 关节指令下发工具。

这个脚本通过 LCM 发布 ``joint_cmds_lcmt``，用于替代
``example/nix_joint_cmd.cpp`` 中依赖宏切换的常用关节测试流程。
所有控制目标都通过命令行、CSV 或模型参考动作文件在运行时指定，不需要重新编译。

安全边界：
    - 本脚本只发布关节指令，不负责进入或退出 DEBUG 状态。
    - 使用前先让机器人进入 RESET/STAND/DEBUG，例如：
      ``python3 lumos_sdk/python/nix_debug_state.py enter --timeout 15``。
    - 第一次操作必须先加 ``--dry-run``，确认 component、joint_id、目标位置和增益。
    - 真机发布时从小幅度、短时间、单关节开始，不要直接回放长动作。

常用示例：
    # 只预览腰关节指令，不发布 LCM。
    python3 lumos_sdk/python/nix_joint_cmd.py single \\
      --component WAIST --joint-id 0 --pos 0.30 --kp 160 --kd 6 --dry-run

    # 发布一次腰关节指令。
    python3 lumos_sdk/python/nix_joint_cmd.py single \\
      --component WAIST --joint-id 0 --pos 0.30 --kp 160 --kd 6

    # 按全局关节索引发布，12 表示 WAIST[0]。
    python3 lumos_sdk/python/nix_joint_cmd.py global \\
      --index 12 --pos 0.30 --kp 160 --kd 6 --duration 2

    # 一个 LCM 包里同时下发多个关节目标。
    python3 lumos_sdk/python/nix_joint_cmd.py batch \\
      --target LEG_L:3:0.20:160:6 \\
      --target WAIST:0:0.30:160:6

    # 从 CSV 读取目标，列名：
    # component,joint_id,pos,kp,kd,vel,tor,ctrl_word
    python3 lumos_sdk/python/nix_joint_cmd.py file --path targets.csv --duration 1

    # 回放模型目录中的 store_ref_motion.txt + kp_kd.yaml。
    python3 lumos_sdk/python/nix_joint_cmd.py replay \\
      --model-dir lumos_sdk/models/nix2_policy/sanlin_04101426 --loops 1
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
from typing import Iterable, List, Optional, Sequence, Tuple


DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=255"
LOCAL_LCM_URL = "udpm://239.255.76.67:7667?ttl=0"
DEFAULT_COMMAND_CHANNEL = "lcm_joint_cmd"

COMPONENTS = {
    "ARM_L": 1,
    "ARM_R": 2,
    "WAIST": 7,
    "LEG_L": 8,
    "LEG_R": 9,
}
COMPONENT_NAMES = {value: key for key, value in COMPONENTS.items()} 

# SDK 下发顺序与 nix_joint_cmd.cpp 保持一致：
# LEG_L(6), LEG_R(6), WAIST(1), ARM_L(4), ARM_R(4)。
GLOBAL_JOINTS: Tuple[Tuple[str, int], ...] = (
    ("LEG_L", 0), ("LEG_L", 1), ("LEG_L", 2), ("LEG_L", 3), ("LEG_L", 4), ("LEG_L", 5),
    ("LEG_R", 0), ("LEG_R", 1), ("LEG_R", 2), ("LEG_R", 3), ("LEG_R", 4), ("LEG_R", 5),
    ("WAIST", 0),
    ("ARM_L", 0), ("ARM_L", 1), ("ARM_L", 2), ("ARM_L", 3),
    ("ARM_R", 0), ("ARM_R", 1), ("ARM_R", 2), ("ARM_R", 3),
)

# 从 example/nix_joint_cmd.cpp CONTROL_REPLAY 移植的映射。
# 每项为 (组件名, 组件内 joint_id, store_ref_motion.txt 中的位置列)。
REPLAY_MAPPING: Tuple[Tuple[str, int, int], ...] = (
    ("LEG_L", 0, 12), ("LEG_L", 1, 16), ("LEG_L", 2, 14), ("LEG_L", 3, 18), ("LEG_L", 4, 20), ("LEG_L", 5, 22),
    ("LEG_R", 0, 13), ("LEG_R", 1, 17), ("LEG_R", 2, 15), ("LEG_R", 3, 19), ("LEG_R", 4, 21), ("LEG_R", 5, 23),
    ("WAIST", 0, 3),
    ("ARM_L", 0, 4), ("ARM_L", 1, 6), ("ARM_L", 2, 8), ("ARM_L", 3, 10),
    ("ARM_R", 0, 5), ("ARM_R", 1, 7), ("ARM_R", 2, 9), ("ARM_R", 3, 11),
)
REPLAY_FIELD_COUNT = 45

# ── NIX2 腿关节限位（来自 lumos_assets/nix2/urdf/nix2_v3.urdf）─────
NIX2_LEG_LIMITS: dict = {
    8: {  # LEG_L
        0: (-1.6755, 1.5708, "left_hip_pitch"),
        1: (-0.0873, 1.8326, "left_hip_roll"),
        2: (-1.2217, 1.5700, "left_hip_yaw"),
        3: (-0.0873, 2.0944, "left_knee"),
        4: (-0.4600, 0.3600, "left_ankle_pitch"),
        5: (-0.3400, 0.3400, "left_ankle_roll"),
    },
    9: {  # LEG_R
        0: (-1.6755, 1.5708, "right_hip_pitch"),
        1: (-1.8326, 0.0873, "right_hip_roll"),
        2: (-1.5708, 1.2217, "right_hip_yaw"),
        3: (-0.0873, 2.0944, "right_knee"),
        4: (-0.4600, 0.3600, "right_ankle_pitch"),
        5: (-0.3400, 0.3400, "right_ankle_roll"),
    },
}


@dataclass(frozen=True)
class JointTarget:
    """一个 SDK 关节指令目标。

    ``kp`` 和 ``kd`` 分别写入 ``joint_cmd_lcmt`` 的 ``res1`` 和 ``res2``。
    ``ctrl_word=3`` 是现有 SDK 示例常用的 MIT/PD 控制路径；``ctrl_word=200``
    是复位语义，不要在普通动作测试里随意使用。
    """

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
        """返回组件编号对应的可读名称。"""

        return COMPONENT_NAMES.get(self.component_type, f"COMP_{self.component_type}")

    def summary(self) -> str:
        """返回一行适合操作前检查的指令摘要。"""

        return (
            f"{self.component_name}[{self.joint_id}] "
            f"pos={self.pos:.4f} vel={self.vel:.4f} tor={self.tor:.4f} "
            f"kp={self.kp:.2f} kd={self.kd:.2f} ctrl={self.ctrl_word}"
        )


def _sdk_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _typedef_dir() -> Path:
    return _sdk_root() / "lcm_typedef" / "python"


def _load_lcm_class(modname: str):
    """加载生成的 LCM 类型，并兼容生成代码里的 module/class 导入问题。"""

    typedef_path = str(_typedef_dir())
    if typedef_path not in sys.path:
        sys.path.insert(0, typedef_path)
    module = importlib.import_module(modname)
    cls = getattr(module, modname, module)
    sys.modules[modname] = cls
    return cls


def load_lcm_runtime():
    """返回 ``(lcm_module, joint_cmd_lcmt, joint_cmds_lcmt)``。

    LCM 相关导入刻意延迟到真正发布时执行，这样没有安装 Python LCM 绑定的机器
    也可以运行 ``--help`` 和 ``--dry-run``。
    """

    try:
        lcm_mod = importlib.import_module("lcm")
    except ImportError as exc:
        raise RuntimeError(
            "Python LCM binding 不可用。请安装/编译 lcm Python 包，"
            "或先使用 --dry-run 只检查指令内容。"
        ) from exc
    joint_cmd = _load_lcm_class("joint_cmd_lcmt")
    joint_cmds = _load_lcm_class("joint_cmds_lcmt")
    return lcm_mod, joint_cmd, joint_cmds


def parse_component(value: str) -> int:
    """解析组件名或数字组件编号。"""

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


def target_from_global_index(index: int, pos: float, kp: float, kd: float, vel: float, tor: float, ctrl_word: int) -> JointTarget:
    """根据 NIX 全局 SDK 关节索引 0-20 构造目标。"""

    if index < 0 or index >= len(GLOBAL_JOINTS):
        raise ValueError(f"全局关节索引必须在 [0, {len(GLOBAL_JOINTS) - 1}] 内，当前为 {index}")
    component, joint_id = GLOBAL_JOINTS[index]
    return JointTarget(
        component_type=COMPONENTS[component],
        joint_id=joint_id,
        pos=pos,
        kp=kp,
        kd=kd,
        vel=vel,
        tor=tor,
        ctrl_word=ctrl_word,
    )


def parse_target_spec(spec: str) -> JointTarget:
    """解析 ``COMPONENT:JOINT_ID:POS[:KP[:KD[:VEL[:TOR[:CTRL_WORD]]]]]``。"""

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


def load_targets_csv(path: str) -> List[JointTarget]:
    """从 CSV 读取关节目标，支持 component,joint_id,pos,kp,kd,vel,tor,ctrl_word 列。"""

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
    """从模型目录的 ``kp_kd.yaml`` 读取 21 个 ``kps`` 和 21 个 ``kds``。

    这里不用 PyYAML，只解析仓库模型目录里当前使用的简单列表格式，避免给 SDK
    调试脚本增加额外依赖。
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
    if len(kps) != len(REPLAY_MAPPING) or len(kds) != len(REPLAY_MAPPING):
        raise ValueError(
            f"{path} 中 kp/kd 数量无效：kps={len(kps)} kds={len(kds)} "
            f"期望各 {len(REPLAY_MAPPING)} 个"
        )
    return kps, kds


def load_ref_motion(path: str, max_frames: int = 0) -> List[List[float]]:
    """读取 ``store_ref_motion.txt`` 参考动作帧。

    每个非空行至少要有 45 个浮点列，与 C++ 回放工具里的
    ``kReplayFieldNum`` 保持一致。
    """

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
            if len(row) < REPLAY_FIELD_COUNT:
                raise ValueError(
                    f"动作文件第 {line_no} 行只有 {len(row)} 列，至少需要 {REPLAY_FIELD_COUNT} 列"
                )
            frames.append(row)
            if max_frames > 0 and len(frames) >= max_frames:
                break
    if not frames:
        raise ValueError(f"动作文件没有可用帧：{path}")
    return frames


def replay_targets_from_frame(frame: Sequence[float], kps: Sequence[float], kds: Sequence[float]) -> List[JointTarget]:
    """把一行参考动作转换成 21 个 SDK 关节目标。"""

    targets: List[JointTarget] = []
    for idx, (component, joint_id, motion_col) in enumerate(REPLAY_MAPPING):
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
    """把 ``JointTarget`` 列表转换成一个 ``joint_cmds_lcmt`` 包。"""

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


def publish_targets(args: argparse.Namespace, targets: Sequence[JointTarget]) -> int:
    """按照通用 CLI 时序参数发布一组关节目标。"""

    if not targets:
        raise ValueError("没有可发布的关节目标")
    for target in targets:
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
    """把模型参考动作回放为 21 关节 SDK 指令包。"""

    model_dir = Path(args.model_dir)
    motion_path = Path(args.motion) if args.motion else model_dir / "store_ref_motion.txt"
    kpkd_path = Path(args.kp_kd) if args.kp_kd else model_dir / "kp_kd.yaml"
    kps, kds = load_kp_kd_yaml(str(kpkd_path))
    frames = load_ref_motion(str(motion_path), max_frames=args.max_frames)
    loops = int(args.loops)

    first_targets = replay_targets_from_frame(frames[0], kps, kds)
    last_targets = replay_targets_from_frame(frames[-1], kps, kds)
    total_packets = None if loops <= 0 else len(frames) * loops
    print(f"模型目录：{model_dir}")
    print(f"动作文件：{motion_path}")
    print(f"增益文件：{kpkd_path}")
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
                targets = replay_targets_from_frame(frame, kps, kds)
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


def resolve_publish_count(count: Optional[int], duration: float, rate_hz: float) -> int:
    """把 ``--count``/``--duration``/``--rate-hz`` 解析成有限发布次数。"""

    if count is not None:
        if count <= 0:
            raise ValueError("--count 必须为正数")
        return int(count)
    if duration > 0:
        if rate_hz <= 0:
            raise ValueError("设置 --duration 时 --rate-hz 必须为正数")
        return max(1, int(math.ceil(duration * rate_hz)))
    return 1


def localize_argparse(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """把 argparse 默认帮助标题改成中文，并返回同一个 parser。"""

    parser._positionals.title = "位置参数"  # pylint: disable=protected-access
    parser._optionals.title = "选项"  # pylint: disable=protected-access
    for action in parser._actions:  # pylint: disable=protected-access
        if "-h" in action.option_strings and "--help" in action.option_strings:
            action.help = "显示帮助信息并退出"
    return parser


def add_common_publish_args(parser: argparse.ArgumentParser) -> None:
    """给子命令添加通用 LCM 发布参数。"""

    parser.add_argument("--channel", default=DEFAULT_COMMAND_CHANNEL, help="LCM 关节指令频道")
    parser.add_argument("--url", default=None, help="显式指定 LCM URL；默认使用 SDK 组播地址")
    parser.add_argument("--local", action="store_true", help="使用 ttl=0，适合同机 LCM mock 测试")
    parser.add_argument("--rate-hz", type=float, default=100.0, help="重复发布时的频率")
    parser.add_argument("--duration", type=float, default=0.0, help="按指定秒数重复发布")
    parser.add_argument("--count", type=int, default=None, help="精确发布指定数量的包")
    parser.add_argument("--dry-run", action="store_true", help="只打印目标，不发布 LCM 包")
    parser.add_argument("--verbose", action="store_true", help="打印每次发布进度")


def add_target_shape_args(parser: argparse.ArgumentParser) -> None:
    """给 single/global 子命令添加共享目标字段。"""

    parser.add_argument("--pos", type=float, required=True, help="目标位置，单位 rad")
    parser.add_argument("--kp", type=float, default=60.0, help="PD 刚度，写入 res1")
    parser.add_argument("--kd", type=float, default=2.0, help="PD 阻尼，写入 res2")
    parser.add_argument("--vel", type=float, default=0.0, help="目标速度")
    parser.add_argument("--tor", type=float, default=0.0, help="前馈力矩")
    parser.add_argument("--ctrl-word", type=int, default=3, help="SDK 控制字；PD 控制通常使用 3")


def build_arg_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""

    parser = argparse.ArgumentParser(
        description="通过 Python 下发 NIX SDK 关节指令，不需要重新编译 C++ 示例。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "全局索引顺序：0-5 LEG_L，6-11 LEG_R，12 WAIST，"
            "13-16 ARM_L，17-20 ARM_R。\n"
            "安全建议：真机发布前先使用 --dry-run；发布前确认机器人已进入 DEBUG(10)。"
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

    p_global = sub.add_parser("global", help="按 NIX 全局 SDK 关节索引下发单个目标")
    localize_argparse(p_global)
    p_global.add_argument("--index", type=int, required=True, help="全局 SDK 关节索引，范围 0-20")
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
        default=str(_sdk_root() / "models" / "nix2_policy" / "sanlin_04101426"),
        help="模型目录，内部应包含 store_ref_motion.txt 和 kp_kd.yaml",
    )
    p_replay.add_argument("--motion", default="", help="覆盖参考动作文件路径")
    p_replay.add_argument("--kp-kd", default="", help="覆盖 kp_kd.yaml 路径")
    p_replay.add_argument("--loops", type=int, default=1, help="回放循环次数；<=0 表示无限循环直到 Ctrl-C")
    p_replay.add_argument("--max-frames", type=int, default=0, help="只加载前 N 帧，用于检查或小范围测试")
    p_replay.add_argument("--preview-joints", type=int, default=6, help="dry-run 预览时打印的关节数量")
    p_replay.add_argument("--print-every", type=int, default=100, help="verbose 模式下每隔多少包打印一次进度")
    add_common_publish_args(p_replay)
    p_replay.set_defaults(func=cmd_replay)

    p_sweep = sub.add_parser("sine-sweep", help="正弦激励：自动读取当前角度 → 验证限位 → 激励 + 采集反馈")
    localize_argparse(p_sweep)
    p_sweep.add_argument("--component", type=parse_component, required=True,
                         help="组件名或编号 (LEG_L=8, LEG_R=9, WAIST=7, ARM_L=1, ARM_R=2)")
    p_sweep.add_argument("--joint-id", type=int, required=True,
                         help="组件内关节编号。LEG: 0=hip_pitch,1=hip_roll,2=hip_yaw,3=knee,4=ankle_pitch,5=ankle_roll")
    p_sweep.add_argument("--amplitude", type=float, required=True,
                         help="正弦幅值 (rad)。hip_pitch 建议 0.20，knee 建议 0.10-0.15")
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
    """打印组件编号和全局关节索引，供操作前核对。"""

    print("组件编号：")
    for name, value in COMPONENTS.items():
        print(f"  {name:<7} {value}")
    print("\n全局 SDK 关节索引：")
    for idx, (component, joint_id) in enumerate(GLOBAL_JOINTS):
        print(f"  {idx:2d}: {component}[{joint_id}]")
    return 0


def cmd_single(args: argparse.Namespace) -> int:
    """构造并发布一个组件内关节目标。"""

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
    """按全局 SDK 关节索引构造并发布一个目标。"""

    target = target_from_global_index(args.index, args.pos, args.kp, args.kd, args.vel, args.tor, args.ctrl_word)
    return publish_targets(args, [target])


def cmd_batch(args: argparse.Namespace) -> int:
    """发布命令行内联的多个目标。"""

    return publish_targets(args, args.target)


def cmd_file(args: argparse.Namespace) -> int:
    """读取 CSV 行并发布其中的关节目标。"""

    return publish_targets(args, load_targets_csv(args.path))


def cmd_replay(args: argparse.Namespace) -> int:
    """回放模型参考动作目标。"""

    return publish_replay(args)


def cmd_sine_sweep(args: argparse.Namespace) -> int:
    """读取当前关节位置 → 验证限位 → 正弦激励 + 自动采集反馈 CSV。"""

    from nix_lcm_sub import NixLcmSubscriber, LcmUnavailableError, write_feedback_rows  # noqa: E402

    component_type = parse_component(args.component)
    joint_id = int(args.joint_id)
    comp_limits = NIX2_LEG_LIMITS.get(component_type, {})
    limit_info = comp_limits.get(joint_id)

    # ── 1. 读取当前关节位置 ──
    print(f"读取当前关节位置 (component={component_type} joint={joint_id}) ...")
    try:
        sub = NixLcmSubscriber()
    except LcmUnavailableError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2

    sub.listen(once=True, timeout_ms=500, duration_sec=10.0)
    if sub.message_count == 0:
        print("错误：10 秒内未收到 lcm_joint_data，检查机器人是否在线", file=sys.stderr)
        return 3

    # 从收到的第一帧中找到指定关节
    current_pos = None
    for s in sub.last_samples:
        if s.component_type == component_type and s.joint_id == joint_id:
            current_pos = s.pos_high
            break

    if current_pos is None:
        available = set((s.component_type, s.joint_id) for s in sub.last_samples)
        print(
            f"错误：当前 LCM 帧中未找到 component={component_type} joint={joint_id}。"
            f"可用关节: {sorted(available)}",
            file=sys.stderr,
        )
        return 4

    print(f"  当前角度 = {current_pos:.4f} rad")
    center = current_pos

    # ── 2. 限位检查 ──
    amplitude = float(args.amplitude)
    p_min = center - amplitude
    p_max = center + amplitude

    joint_name = limit_info[2] if limit_info else f"component={component_type}_joint={joint_id}"

    if limit_info:
        lower, upper, _ = limit_info
        range_rad = upper - lower
        margin = 0.01 * range_rad
        if p_min < lower + margin:
            print(
                f"❌ 关节 {joint_name} 轨迹超出限位 [{lower:.4f}, {upper:.4f}]：\n"
                f"   最小位置 {p_min:.4f} < 下限 {lower:.4f} (超出 {lower - p_min:.4f} rad)\n"
                f"   建议：减小 --amplitude（当前 {amplitude:.3f}）",
                file=sys.stderr,
            )
            return 5
        if p_max > upper - margin:
            print(
                f"❌ 关节 {joint_name} 轨迹超出限位 [{lower:.4f}, {upper:.4f}]：\n"
                f"   最大位置 {p_max:.4f} > 上限 {upper:.4f} (超出 {p_max - upper:.4f} rad)\n"
                f"   建议：减小 --amplitude（当前 {amplitude:.3f}）",
                file=sys.stderr,
            )
            return 5
        print(f"  限位 [{lower:.4f}, {upper:.4f}]，轨迹范围 [{p_min:.4f}, {p_max:.4f}] ✅")
    else:
        print(f"  无限位数据，轨迹范围 [{p_min:.4f}, {p_max:.4f}] ⚠️")

    # ── 3. dry-run 模式 ──
    if args.dry_run:
        duration = float(args.duration)
        rate_hz = float(args.rate_hz)
        freq_hz = float(args.freq_hz)
        n = max(1, int(duration * rate_hz))
        print(f"\n轨迹预览：center={center:.4f} ampl={amplitude:.3f} "
              f"freq={freq_hz:.1f}Hz duration={duration:.0f}s "
              f"rate={rate_hz:.0f}Hz packets={n}")
        print(f"{'时间(s)':>8s}  {'位置(rad)':>10s}")
        print(f"{'─' * 8}  {'─' * 10}")
        preview = min(8, n)
        for i in range(preview):
            t = i * duration / (preview - 1) if preview > 1 else 0.0
            pos = center + amplitude * math.sin(2.0 * math.pi * freq_hz * t)
            marker = " ←起始" if i == 0 else (" ←结束" if i == preview - 1 else "")
            print(f"{t:8.3f}  {pos:10.4f}{marker}")
        print("\n[dry-run] 只预览，未发布 LCM 包。")
        return 0

    # ── 4. 启动后台订阅线程（采集反馈 CSV）──
    csv_path = args.csv or f"build/phase1/{joint_name}_sine_sweep.csv"
    Path(csv_path).parent.mkdir(parents=True, exist_ok=True)

    feedback_done = threading.Event()
    feedback_samples: List[dict] = []
    feedback_lock = threading.Lock()

    def _feedback_collector() -> None:
        try:
            fsub = NixLcmSubscriber()
        except LcmUnavailableError:
            return
        deadline = time.monotonic() + float(args.duration) + 2.0
        while not feedback_done.is_set() and time.monotonic() < deadline:
            try:
                fsub.handle_timeout(timeout_ms=100)
            except Exception:
                break
            if fsub.last_samples:
                with feedback_lock:
                    for s in fsub.last_samples:
                        feedback_samples.append(s.feedback_row())
        if feedback_samples:
            with feedback_lock:
                rows = list(feedback_samples)
            write_feedback_rows(csv_path, rows, append=False)
            print(f"\n反馈已保存：{csv_path} ({len(rows)} 行)")

    feedback_thread = threading.Thread(target=_feedback_collector, daemon=True)
    feedback_thread.start()
    time.sleep(0.3)  # 等订阅就绪

    # ── 5. 发布正弦激励 ──
    duration = float(args.duration)
    rate_hz = float(args.rate_hz)
    freq_hz = float(args.freq_hz)
    n = max(1, int(duration * rate_hz))
    dt = 1.0 / rate_hz

    print(f"\n开始正弦激励：{joint_name} "
          f"center={center:.4f} ampl={amplitude:.3f} "
          f"freq={freq_hz:.1f}Hz duration={duration:.0f}s "
          f"packets={n}")
    print(f"反馈 CSV：{csv_path}")

    lcm_mod, joint_cmd_cls, joint_cmds_cls = load_lcm_runtime()
    lcm_url = (
        LOCAL_LCM_URL if args.local and args.url is None
        else (args.url or DEFAULT_LCM_URL)
    )
    lc = lcm_mod.LCM(lcm_url)

    base_target = JointTarget(
        component_type=component_type,
        joint_id=joint_id,
        pos=0.0,
        kp=float(args.kp),
        kd=float(args.kd),
        vel=0.0, tor=0.0, cur=0.0, ctrl_word=3,
    )

    sent = 0
    next_tick = time.monotonic()
    try:
        for i in range(n):
            t = i * dt
            pos = center + amplitude * math.sin(2.0 * math.pi * freq_hz * t)
            target = JointTarget(
                component_type=base_target.component_type,
                joint_id=base_target.joint_id,
                pos=pos,
                kp=base_target.kp, kd=base_target.kd,
                vel=base_target.vel, tor=base_target.tor,
                cur=base_target.cur, ctrl_word=base_target.ctrl_word,
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

    # 等后台采集线程写完 CSV
    feedback_thread.join(timeout=5.0)
    if feedback_samples:
        print(f"完成。{sent} 个包 → {csv_path} ({len(feedback_samples)} 行反馈)")
    else:
        print(f"完成。{sent} 个包（⚠️ 未采集到反馈数据，检查 LCM 订阅）")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口，返回进程退出码。"""

    args = build_arg_parser().parse_args(argv)
    try:
        return int(args.func(args))
    except (RuntimeError, ValueError, argparse.ArgumentTypeError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
