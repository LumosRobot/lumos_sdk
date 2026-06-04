#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mock NIX JointsData publisher for local subscriber testing.

这个脚本用于不上真机时验证 ``nix_lcm_sub.py``：

    终端 A:
        python3 lumos_sdk/python/nix_lcm_sub.py --local --once

    终端 B:
        python3 lumos_sdk/python/nix_lcm_pub_mock.py --local --count 5

如果订阅器打印 ``messages=1 samples=12``，说明本机 LCM、channel 名、
Python typedef 解码链路是通的。
"""

from __future__ import annotations

import argparse
import importlib
import sys
import time
from pathlib import Path
from typing import Optional, Sequence


DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=255"
LOCAL_LCM_URL = "udpm://239.255.76.67:7667?ttl=0" # 本机 pub/sub 用这个 URL，避免干扰同网段其它机器的订阅器
DEFAULT_JOINT_CHANNEL = "JointsData"  # LCM channel 
NIX_JOINT_KEYS = ( # 顺序与 SDK 下发顺序一致，方便对照订阅器输出。LEG_L 6 + LEG_R 6 + WAIST 1 + ARM_L 4 + ARM_R 4 = 21 joints
    (8, 0), (8, 1), (8, 2), (8, 3), (8, 4), (8, 5),
    (9, 0), (9, 1), (9, 2), (9, 3), (9, 4), (9, 5),
    (7, 0),
    (1, 0), (1, 1), (1, 2), (1, 3),
    (2, 0), (2, 1), (2, 2), (2, 3),
)*


class MockPublisherError(RuntimeError):
    """Raised when LCM or generated typedefs cannot be loaded."""


def _sdk_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _typedef_dir() -> Path:
    return _sdk_root() / "lcm_typedef" / "python"


def _load_lcm_class(modname: str):
    """Load generated LCM class using the same workaround as SDK debug scripts."""

    typedef_path = str(_typedef_dir())
    if typedef_path not in sys.path:
        sys.path.insert(0, typedef_path)
    module = importlib.import_module(modname)
    cls = getattr(module, modname, module)
    sys.modules[modname] = cls
    return cls


def load_lcm_module():
    try:
        return importlib.import_module("lcm")
    except ImportError as exc:
        raise MockPublisherError(
            "Python LCM binding is not available. Install/build the lcm Python package."
        ) from exc


def load_joint_types():
    """Return ``(sdk_lcmt_joint_datasets, sdk_lcmt_joint_data)`` classes."""

    try:
        joint_data = _load_lcm_class("sdk_lcmt_joint_data")
        datasets = _load_lcm_class("sdk_lcmt_joint_datasets")
        return datasets, joint_data
    except Exception as exc:  # pragma: no cover - exact import errors vary.
        raise MockPublisherError(
            f"Failed to load generated LCM typedefs from {_typedef_dir()}."
        ) from exc


def build_mock_message(frame_index: int = 0, joints: int = 12):
    """Build one fake ``sdk_lcmt_joint_datasets`` message.

    By default this mirrors the SDK 下发顺序 (NIX2, 21 joints):
    LEG_L 6 + LEG_R 6 + WAIST 1 + ARM_L 4 + ARM_R 4. Values change with
    ``frame_index`` so repeated publishes are easy to distinguish in subscriber
    output.
    """

    dataset_type, joint_type = load_joint_types()
    msg = dataset_type()
    msg.datasets_num = int(joints)
    msg.datasets = []

    keys = list(NIX_JOINT_KEYS)
    if joints != len(keys):
        keys = keys[:joints]

    for i, (component_type, joint_id) in enumerate(keys):
        d = joint_type()
        d.component_type = component_type
        d.joint_id = joint_id
        d.stat = 0
        d.pos_high = 0.1 * i + 0.01 * frame_index
        d.pos_low = 0.0
        d.vel = 0.2 * i + 0.01 * frame_index
        d.tor = 0.3 * i + 0.01 * frame_index
        d.cur = 0.4 * i + 0.01 * frame_index
        msg.datasets.append(d)

    return msg


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publish mock NIX JointsData messages for local testing.",
    )
    parser.add_argument("--channel", default=DEFAULT_JOINT_CHANNEL)
    parser.add_argument("--url", default=None, help="Explicit LCM URL. Defaults to the SDK URL with ttl=255.")
    parser.add_argument("--local", action="store_true", help="Use ttl=0 for local mock pub/sub on this machine.")
    parser.add_argument("--count", type=int, default=5, help="Number of messages to publish.")
    parser.add_argument("--rate-hz", type=float, default=10.0, help="Publish rate.")
    parser.add_argument("--joints", type=int, default=12, help="Number of fake joints per message.")
    parser.add_argument("--quiet", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    lcm_url = LOCAL_LCM_URL if args.local and args.url is None else (args.url or DEFAULT_LCM_URL)

    try:
        lcm_mod = load_lcm_module()
        lc = lcm_mod.LCM(lcm_url)
    except Exception as exc:
        print(f"[nix_lcm_pub_mock] Failed to create LCM publisher for {lcm_url}: {exc}", file=sys.stderr)
        return 2

    delay = 0.0 if args.rate_hz <= 0 else 1.0 / args.rate_hz
    for frame in range(max(args.count, 0)):
        try:
            msg = build_mock_message(frame_index=frame, joints=max(args.joints, 0))
            lc.publish(args.channel, msg.encode())
        except Exception as exc:
            print(f"[nix_lcm_pub_mock] publish failed: {exc}", file=sys.stderr)
            return 1

        if not args.quiet:
            print(
                f"[nix_lcm_pub_mock] published frame={frame + 1}/{args.count} "
                f"channel={args.channel} joints={args.joints}"
            )
        if delay > 0 and frame + 1 < args.count:
            time.sleep(delay)

    if not args.quiet:
        print("[nix_lcm_pub_mock] done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
