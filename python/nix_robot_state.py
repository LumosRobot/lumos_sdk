#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""NIX 机器人状态切换与等待确认工具。

这个脚本负责发布机器人高层状态命令，并可订阅 ``lcm_robot_status`` 等待
真实状态确认。它和 ``nix_joint_cmd.py`` 的边界不同：

    - ``nix_robot_state.py`` 管 RESET/STAND/DEBUG/RL 等高层状态。
    - ``nix_joint_cmd.py`` 管 controller DEBUG 状态下的关节级 ``joint_cmds_lcmt``。

为什么需要它：
    只发布一次状态命令不能证明机器人已经进入目标状态。本脚本会在发送命令后
    持续处理 LCM status，直到看到目标
    状态或超时失败，适合写进 Phase 1 采集 runbook。

常用示例：
    # 只打印将要执行的 RESET -> STAND 流程，不发布 LCM。
    python3 lumos_sdk/python/nix_robot_state.py stand --dry-run

    # RESET 后等待 RESET(1)，再 STAND 并等待 STAND(2)，最后进入 DEBUG(10)。
    python3 lumos_sdk/python/nix_robot_state.py stand --timeout 15

    # 只发送并等待某个状态，例如 STAND。
    python3 lumos_sdk/python/nix_robot_state.py state STAND --wait --timeout 15

    # 监听当前机器人状态。
    python3 lumos_sdk/python/nix_robot_state.py listen --duration 10

安全边界：
    - 默认 ``stand`` 会先确认 RESET/STAND，再切入 DEBUG(10) 关节调试状态。
    - ``state STATE --wait`` 依赖 controller 发布 ``lcm_robot_status``。如果机器人
      已经处在同一个 STATE，controller 可能不会重复发布状态确认；真机 smoke
      test 优先使用 ``stand`` 或 ``nix_debug_state.py enter/leave``。
    - 本脚本不会发布任何关节 PD 目标。
    - 真机使用前先确认机器人周围安全、急停可用、controller 正常运行。
    - 如果等待超时，不要继续采集或下发关节命令；先检查 status 频道和机器人状态。
"""

from __future__ import annotations

import argparse
import importlib
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence


DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=255"
LOCAL_LCM_URL = "udpm://239.255.76.67:7667?ttl=0"

CH_ROBOT_CMD = "lcm_robot_cmd"
CH_STATUS = "lcm_robot_status"

STATE_TYPES = {
    "NOT_A_STATE": 0,
    "RESET": 1,
    "STAND": 2,
    "RL_WALK": 3,
    "RL_LIEDOWN": 5,
    "RL_MIMIC": 6,
    "DEBUG": 10,
    "RL_NAV": 11,
    "RL_WALK_AMP": 12,
    "BY_MIMIC": 20,
    "BFM_MIMIC": 21,
}
STATE_NAMES = {value: key for key, value in STATE_TYPES.items()}


@dataclass
class StatusSnapshot:
    """一次 ``lcm_robot_status`` 反馈快照。"""

    state: int
    controller_type: int
    received_at: float

    @property
    def state_name(self) -> str:
        """返回状态码对应的可读名称。"""

        return STATE_NAMES.get(self.state, f"UNKNOWN_{self.state}")

    def summary(self) -> str:
        """返回适合终端显示的一行状态摘要。"""

        return f"state={self.state_name}({self.state}) type={self.controller_type}"


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
    """返回 ``(lcm_module, robot_cmd_lcmt, robot_status_lcmt)``。

    LCM 导入延迟到真正发布/订阅时执行，保证 ``--help`` 和 ``--dry-run`` 不依赖
    机器人现场环境。
    """

    try:
        lcm_mod = importlib.import_module("lcm")
    except ImportError as exc:
        raise RuntimeError(
            "Python LCM binding 不可用。请安装/编译 lcm Python 包，"
            "或使用 --dry-run 只检查将要发送的状态命令。"
        ) from exc
    robot_cmd_cls = _load_lcm_class("robot_cmd_lcmt")
    status_cls = _load_lcm_class("robot_status_lcmt")
    return lcm_mod, robot_cmd_cls, status_cls


def parse_state(value: str) -> int:
    """解析状态名或数字状态码。"""

    raw = str(value).strip()
    key = raw.upper()
    if key in STATE_TYPES:
        return STATE_TYPES[key]
    try:
        state = int(raw)
    except ValueError as exc:
        valid = ", ".join(STATE_TYPES)
        raise argparse.ArgumentTypeError(f"未知状态 {value!r}；应为 {valid} 或数字状态码") from exc
    if state not in STATE_NAMES:
        raise argparse.ArgumentTypeError(f"未知状态码：{state}")
    return state


class RobotStateClient:
    """NIX 高层状态 LCM 客户端。

    这个类封装三件事：
        1. 发布 RESET/STAND/DEBUG/RL 等机器人状态命令。
        2. 订阅 ``lcm_robot_status`` 并等待目标状态。
    """

    def __init__(self, lcm_url: str = DEFAULT_LCM_URL) -> None:
        lcm_mod, robot_cmd_cls, status_cls = load_lcm_runtime()
        self._robot_cmd_cls = robot_cmd_cls
        self._status_cls = status_cls
        self._lc = lcm_mod.LCM(lcm_url)
        self.last_status: Optional[StatusSnapshot] = None
        self.status_count = 0
        self._lc.subscribe(CH_STATUS, self._on_status)

    def _on_status(self, _channel: str, data: bytes) -> None:
        msg = self._status_cls.decode(data)
        self.status_count += 1
        self.last_status = StatusSnapshot(
            state=int(msg.state),
            controller_type=int(msg.type),
            received_at=time.time(),
        )

    def publish_state(self, state: int, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> None:
        """发布机器人高层状态命令。"""

        msg = self._robot_cmd_cls()
        msg.state = int(state)
        msg.x = float(vx)
        msg.y = float(vy)
        msg.yaw = float(vyaw)
        msg.policy_type = 0
        self._lc.publish(CH_ROBOT_CMD, msg.encode())

    def wait_for_state(self, target_state: int, timeout: float, print_updates: bool = True) -> StatusSnapshot:
        """等待 ``lcm_robot_status.state`` 到达目标状态。

        如果在 ``timeout`` 秒内没有收到目标状态，会抛出 ``TimeoutError``。调用者
        应停止后续采集/控制动作，先检查 controller、多播路由或机器人实际状态。
        """

        deadline = time.monotonic() + timeout
        last_printed: Optional[int] = None
        while time.monotonic() < deadline:
            self._lc.handle_timeout(100)
            status = self.last_status
            if status is None:
                continue
            if print_updates and status.state != last_printed:
                print(f"[Status] {status.summary()}")
                last_printed = status.state
            if status.state == target_state:
                return status
        current = "未收到 status" if self.last_status is None else self.last_status.summary()
        target = STATE_NAMES.get(target_state, str(target_state))
        raise TimeoutError(f"等待 {target}({target_state}) 超时，当前：{current}")

    def listen(self, duration: float = 0.0, print_every: int = 1) -> None:
        """持续监听并打印机器人状态。"""

        deadline = None if duration <= 0 else time.monotonic() + duration
        printed = 0
        while deadline is None or time.monotonic() < deadline:
            self._lc.handle_timeout(200)
            if self.last_status is None:
                continue
            if self.status_count > printed and self.status_count % max(print_every, 1) == 0:
                print(f"[Status] messages={self.status_count} {self.last_status.summary()}")
                printed = self.status_count


def lcm_url_from_args(args: argparse.Namespace) -> str:
    """按 ``--local``/``--url`` 解析最终 LCM URL。"""

    if args.local and args.url is None:
        return LOCAL_LCM_URL
    return args.url or DEFAULT_LCM_URL


def state_name(state: int) -> str:
    """返回状态码名称。"""

    return STATE_NAMES.get(state, f"UNKNOWN_{state}")


def cmd_list(_args: argparse.Namespace) -> int:
    """打印可用状态码。"""

    print("Robot states:")
    for name, value in STATE_TYPES.items():
        print(f"  {name:<12} {value}")
    return 0


def cmd_state(args: argparse.Namespace) -> int:
    """发布一个状态命令，并可选等待状态确认。"""

    print(f"发送 state={state_name(args.state)}({args.state}) vx={args.vx:g} vy={args.vy:g} vyaw={args.vyaw:g}")
    if args.dry_run:
        print("dry-run：不发布 LCM")
        return 0
    client = RobotStateClient(lcm_url_from_args(args))
    client.publish_state(args.state, vx=args.vx, vy=args.vy, vyaw=args.vyaw)
    if args.wait:
        status = client.wait_for_state(args.state, timeout=args.timeout)
        print(f"已确认：{status.summary()}")
    return 0


def cmd_stand(args: argparse.Namespace) -> int:
    """执行安全站立准备流程：RESET、STAND，然后可选进入 DEBUG。"""

    steps = []
    if not args.skip_reset:
        steps.append(f"state RESET({STATE_TYPES['RESET']}) 并等待确认")
    steps.append(f"state STAND({STATE_TYPES['STAND']}) 并等待确认")
    if args.enter_debug:
        steps.append(f"state DEBUG({STATE_TYPES['DEBUG']}) 并等待确认")
    print("计划执行：")
    for step in steps:
        print(f"  - {step}")
    if args.dry_run:
        print("dry-run：不发布 LCM")
        return 0

    client = RobotStateClient(lcm_url_from_args(args))
    if not args.skip_reset:
        client.publish_state(STATE_TYPES["RESET"])
        print("已发送：state RESET(1)")
        reset_status = client.wait_for_state(STATE_TYPES["RESET"], timeout=args.timeout)
        print(f"已确认：{reset_status.summary()}")
        time.sleep(max(args.reset_settle, 0.0))

    client.publish_state(STATE_TYPES["STAND"])
    print("已发送：state STAND(2)")
    stand_status = client.wait_for_state(STATE_TYPES["STAND"], timeout=args.timeout)
    print(f"已确认：{stand_status.summary()}")
    if args.stand_settle > 0:
        print(f"等待站稳：{args.stand_settle:g}s")
        time.sleep(args.stand_settle)
    if args.enter_debug:
        client.publish_state(STATE_TYPES["DEBUG"])
        print("已发送：state DEBUG(10)")
        debug_status = client.wait_for_state(STATE_TYPES["DEBUG"], timeout=args.timeout)
        print(f"已确认：{debug_status.summary()}")
    return 0


def cmd_listen(args: argparse.Namespace) -> int:
    """监听并打印 ``lcm_robot_status``。"""

    client = RobotStateClient(lcm_url_from_args(args))
    print(f"监听 {CH_STATUS} url={lcm_url_from_args(args)}")
    try:
        client.listen(duration=args.duration, print_every=args.print_every)
    except KeyboardInterrupt:
        print("\n已停止监听")
    return 0


def localize_argparse(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """把 argparse 默认帮助标题改成中文，并返回同一个 parser。"""

    parser._positionals.title = "位置参数"  # pylint: disable=protected-access
    parser._optionals.title = "选项"  # pylint: disable=protected-access
    for action in parser._actions:  # pylint: disable=protected-access
        if "-h" in action.option_strings and "--help" in action.option_strings:
            action.help = "显示帮助信息并退出"
    return parser


def add_lcm_args(parser: argparse.ArgumentParser) -> None:
    """添加通用 LCM 参数。"""

    parser.add_argument("--url", default=None, help="显式指定 LCM URL；默认使用 SDK 组播地址")
    parser.add_argument("--local", action="store_true", help="使用 ttl=0，适合同机 LCM mock 测试")
    parser.add_argument("--dry-run", action="store_true", help="只打印计划，不发布 LCM")


def build_arg_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""

    parser = argparse.ArgumentParser(
        description="发布 NIX 高层状态命令，并等待 lcm_robot_status 确认。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "推荐采集前流程：\n"
            "  python3 lumos_sdk/python/nix_robot_state.py stand --timeout 15 --stand-settle 10\n"
            "成功标志：输出“已确认：state=STAND(2)”和“已确认：state=DEBUG(10)”。\n"
            "注意：state STATE --wait 在目标状态未变化时可能等不到新的 status；"
            "真机 smoke test 优先使用 stand。"
        ),
    )
    localize_argparse(parser)
    sub = parser.add_subparsers(dest="command", required=True)

    p_list = sub.add_parser("list", help="打印可用状态码")
    localize_argparse(p_list)
    p_list.set_defaults(func=cmd_list)

    p_state = sub.add_parser("state", help="发送一个机器人状态命令")
    localize_argparse(p_state)
    p_state.add_argument("state", type=parse_state, metavar="STATE", help="状态名或状态码，例如 RESET/STAND/DEBUG/1/2/10")
    p_state.add_argument("--wait", action="store_true", help="等待 lcm_robot_status 到达目标状态")
    p_state.add_argument("--timeout", type=float, default=15.0, help="等待目标状态的超时时间，单位秒")
    p_state.add_argument("--vx", type=float, default=0.0, help="x 方向速度命令，用于 RL_WALK/RL_WALK_AMP")
    p_state.add_argument("--vy", type=float, default=0.0, help="y 方向速度命令，用于 RL_WALK/RL_WALK_AMP")
    p_state.add_argument("--vyaw", type=float, default=0.0, help="yaw 速度命令，用于 RL_WALK/RL_WALK_AMP")
    add_lcm_args(p_state)
    p_state.set_defaults(func=cmd_state)

    p_stand = sub.add_parser("stand", help="执行 RESET -> STAND，确认后进入 DEBUG")
    localize_argparse(p_stand)
    p_stand.add_argument("--no-enter-debug", "--no-enter-sdk", dest="enter_debug", action="store_false", help="确认 STAND 后不自动进入 DEBUG(10)")
    p_stand.add_argument("--skip-reset", action="store_true", help="跳过 RESET，直接发送 STAND 并等待确认")
    p_stand.add_argument("--timeout", type=float, default=15.0, help="每个状态等待确认的超时时间，单位秒")
    p_stand.add_argument("--reset-settle", type=float, default=3.0, help="确认 RESET 后等待时间，单位秒")
    p_stand.add_argument("--stand-settle", type=float, default=0.0, help="确认 STAND 后继续等待站稳时间，单位秒")
    p_stand.set_defaults(enter_debug=True)
    add_lcm_args(p_stand)
    p_stand.set_defaults(func=cmd_stand)

    p_listen = sub.add_parser("listen", help="监听并打印 lcm_robot_status")
    localize_argparse(p_listen)
    p_listen.add_argument("--duration", type=float, default=0.0, help="监听时长，0 表示直到 Ctrl-C")
    p_listen.add_argument("--print-every", type=int, default=1, help="每 N 条 status 打印一次")
    add_lcm_args(p_listen)
    p_listen.set_defaults(func=cmd_listen)

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口，返回进程退出码。"""

    args = build_arg_parser().parse_args(argv)
    try:
        return int(args.func(args))
    except (RuntimeError, TimeoutError, argparse.ArgumentTypeError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
