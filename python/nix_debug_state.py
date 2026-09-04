#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""NIX DEBUG 状态进入/退出工具。

这个脚本是关节级调试流程的独立入口：

    # 默认执行 RESET -> STAND -> DEBUG，并等待 status 确认。
    python3 lumos_sdk/python/nix_debug_state.py enter --timeout 15

    # 退出 DEBUG，回到 STAND。
    python3 lumos_sdk/python/nix_debug_state.py leave --timeout 15

它只发布 ``lcm_robot_cmd`` 的机器人状态命令，不发布关节目标。关节目标仍由
``nix_joint_cmd.py`` / ``lus_joint_cmd.py`` 发布到 ``lcm_joint_cmd``。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional, Sequence


THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from nix_robot_state import (  # noqa: E402
    STATE_TYPES,
    RobotStateClient,
    add_lcm_args,
    lcm_url_from_args,
    localize_argparse,
)


def cmd_enter(args: argparse.Namespace) -> int:
    """执行进入 DEBUG 的状态序列。"""

    steps = []
    if not args.skip_reset:
        steps.append("state RESET(1) 并等待确认")
    steps.append("state STAND(2) 并等待确认")
    steps.append("state DEBUG(10) 并等待确认")
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
        if args.reset_settle > 0:
            print(f"等待 RESET 稳定：{args.reset_settle:g}s")
            time.sleep(args.reset_settle)

    client.publish_state(STATE_TYPES["STAND"])
    print("已发送：state STAND(2)")
    stand_status = client.wait_for_state(STATE_TYPES["STAND"], timeout=args.timeout)
    print(f"已确认：{stand_status.summary()}")
    if args.stand_settle > 0:
        print(f"等待站稳：{args.stand_settle:g}s")
        time.sleep(args.stand_settle)

    client.publish_state(STATE_TYPES["DEBUG"])
    print("已发送：state DEBUG(10)")
    debug_status = client.wait_for_state(STATE_TYPES["DEBUG"], timeout=args.timeout)
    print(f"已确认：{debug_status.summary()}")
    return 0


def cmd_leave(args: argparse.Namespace) -> int:
    """退出 DEBUG，切回 STAND。"""

    print("计划执行：")
    print("  - state STAND(2) 并等待确认")
    if args.dry_run:
        print("dry-run：不发布 LCM")
        return 0

    client = RobotStateClient(lcm_url_from_args(args))
    client.publish_state(STATE_TYPES["STAND"])
    print("已发送：state STAND(2)")
    if args.wait:
        stand_status = client.wait_for_state(STATE_TYPES["STAND"], timeout=args.timeout)
        print(f"已确认：{stand_status.summary()}")
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""

    parser = argparse.ArgumentParser(
        description="独立进入/退出 NIX DEBUG(10) 状态。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "常用流程：\n"
            "  python3 lumos_sdk/python/nix_debug_state.py enter --timeout 15\n"
            "  python3 lumos_sdk/python/nix_joint_cmd.py single --component WAIST --joint-id 0 --pos 0.0 --dry-run\n"
            "  python3 lumos_sdk/python/nix_debug_state.py leave --timeout 15"
        ),
    )
    localize_argparse(parser)
    sub = parser.add_subparsers(dest="command")

    p_enter = sub.add_parser("enter", help="执行 RESET -> STAND -> DEBUG")
    localize_argparse(p_enter)
    p_enter.add_argument("--skip-reset", action="store_true", help="跳过 RESET，直接 STAND -> DEBUG")
    p_enter.add_argument("--timeout", type=float, default=15.0, help="每个状态等待确认的超时时间，单位秒")
    p_enter.add_argument("--reset-settle", type=float, default=3.0, help="确认 RESET 后等待时间，单位秒")
    p_enter.add_argument("--stand-settle", type=float, default=10.0, help="确认 STAND 后继续等待站稳时间，单位秒")
    add_lcm_args(p_enter)
    p_enter.set_defaults(func=cmd_enter)

    p_leave = sub.add_parser("leave", help="退出 DEBUG，切回 STAND")
    localize_argparse(p_leave)
    p_leave.add_argument("--wait", action="store_true", default=True, help="等待 lcm_robot_status 确认 STAND")
    p_leave.add_argument("--no-wait", dest="wait", action="store_false", help="只发送 STAND，不等待确认")
    p_leave.add_argument("--timeout", type=float, default=15.0, help="等待 STAND 确认的超时时间，单位秒")
    add_lcm_args(p_leave)
    p_leave.set_defaults(func=cmd_leave)

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口，默认执行 enter。"""

    if argv is None:
        argv = list(sys.argv[1:])
    else:
        argv = list(argv)
    if not argv:
        argv = ["enter"]
    elif argv[0] not in ("-h", "--help") and argv[0].startswith("-"):
        argv = ["enter", *argv]
    args = build_arg_parser().parse_args(argv)
    try:
        return int(args.func(args))
    except (RuntimeError, TimeoutError, argparse.ArgumentTypeError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
