#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""订阅并打印 SDK 发布的 ``lcm_joint_cmd`` 关节命令。"""

from __future__ import annotations

import argparse
import importlib
import os
import sys
import time
from typing import Optional, Sequence

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(THIS_DIR)
TYPEDEF_DIR = os.path.join(ROOT_DIR, "lcm_typedef", "python")
sys.path.insert(0, TYPEDEF_DIR)


def _load_lcm_class(modname: str):
    """加载生成的 LCM 类型，并兼容生成代码里的 module/class 导入问题。"""

    module = importlib.import_module(modname)
    cls = getattr(module, modname, module)
    sys.modules[modname] = cls
    return cls


_load_lcm_class("joint_cmd_lcmt")
joint_cmds_lcmt = _load_lcm_class("joint_cmds_lcmt")


DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=255"
LOCAL_LCM_URL = "udpm://239.255.76.67:7667?ttl=0"
DEFAULT_CHANNEL = "lcm_joint_cmd"


def lcm_url_from_args(args: argparse.Namespace) -> str:
    """按 ``--local``/``--url`` 解析最终 LCM URL。"""

    if args.local and args.url is None:
        return LOCAL_LCM_URL
    return args.url or DEFAULT_LCM_URL


class JointCmdSubscriber:
    """``lcm_joint_cmd`` 订阅器。"""

    def __init__(self, url: str, channel: str, print_limit: int = 0) -> None:
        try:
            import lcm
        except ImportError as exc:
            raise RuntimeError("Python LCM binding 不可用。请先安装/编译 lcm Python 包。") from exc
        self.lc = lcm.LCM(url)
        self.channel = channel
        self.print_limit = print_limit
        self.message_count = 0
        self.decode_errors = 0
        self.lc.subscribe(channel, self.handler)

    def handler(self, channel: str, data: bytes) -> None:
        """LCM 消息回调。"""

        self.message_count += 1
        try:
            msg = joint_cmds_lcmt.decode(data)
        except Exception as exc:  # noqa: BLE001
            self.decode_errors += 1
            print(f"[nix_joint_cmd_sub] decode error #{self.decode_errors}: {exc}", file=sys.stderr)
            return

        print("=" * 50)
        print(f"收到 LCM 话题: {channel}")
        print(f"关节命令数量: {msg.cmds_num}")
        limit = msg.cmds_num if self.print_limit <= 0 else min(msg.cmds_num, self.print_limit)
        for cmd in msg.cmds[:limit]:
            print(
                f"component={cmd.component_type} joint={cmd.joint_id} "
                f"ctrl={cmd.ctrlWord} pos={cmd.tarPos:.4f} vel={cmd.tarVel:.4f} "
                f"tor={cmd.tarTor:.4f} kp={cmd.res1:.2f} kd={cmd.res2:.2f}"
            )
        if limit < msg.cmds_num:
            print(f"... 省略 {msg.cmds_num - limit} 条")

    def listen(self, duration: float, once: bool, timeout_ms: int) -> None:
        """监听关节命令。"""

        deadline = time.monotonic() + duration if duration > 0 else None
        while deadline is None or time.monotonic() < deadline:
            self.lc.handle_timeout(timeout_ms)
            if once and self.message_count > 0:
                break


def build_arg_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""

    parser = argparse.ArgumentParser(description="Subscribe to SDK lcm_joint_cmd messages.")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL, help="订阅频道，默认 lcm_joint_cmd")
    parser.add_argument("--url", default=None, help="显式指定 LCM URL；默认使用 SDK 组播地址")
    parser.add_argument("--local", action="store_true", help="使用 ttl=0，适合同机 LCM mock 测试")
    parser.add_argument("--duration", type=float, default=0.0, help="监听时长，0 表示直到 Ctrl-C")
    parser.add_argument("--once", action="store_true", help="收到第一条可解码消息后退出")
    parser.add_argument("--timeout-ms", type=int, default=200, help="LCM handle timeout，单位 ms")
    parser.add_argument("--print-limit", type=int, default=0, help="每包最多打印多少条关节命令，0 表示全部")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口。"""

    args = build_arg_parser().parse_args(argv)
    url = lcm_url_from_args(args)
    try:
        sub = JointCmdSubscriber(url=url, channel=args.channel, print_limit=args.print_limit)
    except RuntimeError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1
    print(f"[nix_joint_cmd_sub] listening channel={args.channel} url={url}")
    try:
        sub.listen(duration=args.duration, once=args.once, timeout_ms=args.timeout_ms)
    except KeyboardInterrupt:
        print("\n退出订阅")
    print(
        f"[nix_joint_cmd_sub] done messages={sub.message_count} "
        f"decode_errors={sub.decode_errors}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
