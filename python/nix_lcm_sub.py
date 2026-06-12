#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""NIX joint-data LCM subscriber.

这个脚本只做一件事：监听 NIX 控制器发布的 ``JointsData`` LCM 频道，
把 ``sdk_lcmt_joint_datasets`` 解码成 Python 里的关节样本，并可选写成
feedback CSV。

常用命令：

    # 连续监听 10 秒，收到消息就打印前 12 个关节
    python3 lumos_sdk/python/nix_lcm_sub.py --duration 10

    # 等到收到第一帧 JointsData 后退出；如果机器人没发数据，会一直等
    python3 lumos_sdk/python/nix_lcm_sub.py --once

    # 采 10 秒并写成当前 pipeline 能读取的 feedback CSV 列名
    python3 lumos_sdk/python/nix_lcm_sub.py --duration 10 --csv build/nix_feedback.csv

    # 真机验证：打印完整一帧 21 个关节，确认不是只收到部分肢体
    python3 lumos_sdk/python/nix_lcm_sub.py --duration 2 --print-limit 21

    # 真机采集：降低打印频率，同时保存 CSV
    python3 lumos_sdk/python/nix_lcm_sub.py \
      --duration 10 \
      --print-every 500 \
      --print-limit 21 \
      --csv build/nix_feedback_hw.csv

    # 采集后检查 CSV 行数和首尾内容
    wc -l build/nix_feedback_hw.csv
    head build/nix_feedback_hw.csv
    tail build/nix_feedback_hw.csv

    # 生成的 CSV 第一行会写入 schema 版本；优先读取 lumos_pipeline 的
    # 当前 schema JSON，读不到时退回特殊默认值 v0.0：
    #   # feedback_schema=v1.0
    # 可用 lumos_pipeline 校验：
    PYTHONPATH=lumos_pipeline/src:lumos_diagnostics/src \
      python -m lumos_pipeline.cli verify-schema \
      build/nix_feedback_hw.csv --robot-model nix --require-header

    # 本机 mock/pub-sub 环回测试时用 --local，避免依赖外部网络路由
    python3 lumos_sdk/python/nix_lcm_sub.py --local --duration 10

真机输出判断：
    - ``messages > 0`` 且 ``decode_errors=0``：LCM 订阅和 typedef 解码正常。
    - ``samples=21``：每帧收到 NIX 当前 21 个关节。
    - 21 个关节通常为 ARM_L 4 + ARM_R 4 + LEG_L 6 + LEG_R 6 + WAIST 1。
    - ``vel`` 大多为 0 通常表示机器人当前基本静止。
    - ``stat=0`` 通常表示正常状态/无故障。
    - ``cur=0`` 表示当前底层反馈未填电流或当前模式不提供电流；先记录，
      不阻塞 LCM 桥接验证。
    - CSV 里的 ``JointID`` 使用 ``<ComponentType>:<joint_id>`` 组合键，例如
      ``8:0`` / ``9:0``。不要只写原始 joint_id，否则 ARM/LEG/WAIST 的
      ``joint_id=0`` 会在 pipeline pivot 时被错误合并。

注意：
    - ``--once`` 不是“尝试一次”。它是“收到第一条消息后退出”。
    - 如果控制器没有进入会发布 ``JointsData`` 的状态，脚本不会有样本输出。
    - 如果 Ctrl+C 时 LCM 抛 ``lcm_handle_timeout() returned -1``，这里会把它
      当作干净退出处理，不再打印 Python traceback。
"""

from __future__ import annotations

import argparse
import csv
import importlib
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, List, Optional, Sequence


DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=255"
LOCAL_LCM_URL = "udpm://239.255.76.67:7667?ttl=0"
DEFAULT_JOINT_CHANNEL = "JointsData"
DEFAULT_FEEDBACK_SCHEMA_VERSION = "v0.0"


class LcmUnavailableError(RuntimeError):
    """Raised when Python LCM bindings or generated typedefs cannot be loaded."""


@dataclass(frozen=True)
class NixJointSample:
    """One decoded joint feedback sample from ``sdk_lcmt_joint_data``."""

    timestamp: float
    component_type: int
    joint_id: int
    stat: int
    pos_high: float
    vel: float          # 速度
    tor: float          # 力矩
    cur: float          # 电流
    pos_low: float
    temp: int           # 温度
    raw_component: str = DEFAULT_JOINT_CHANNEL

    def feedback_row(self) -> dict:
        """Return a row compatible with the current feedback CSV column shape."""

        return {
            "timestamp": self.timestamp,
            "ComponentType": self.component_type,
            "JointID": f"{self.component_type}:{self.joint_id}",
            "Position": self.pos_high,
            "ActualVel": self.vel,
            "Torque": self.tor,
            "MotorCurrent": self.cur, # 似乎电机向上发布
            "Temperature": self.temp,
            "ImuGyroX": "",
            "ImuGyroY": "",
            "ImuGyroZ": "",
            "ImuAccelX": "",
            "ImuAccelY": "",
            "ImuAccelZ": "",
            "ImuRoll": "",
            "ImuPitch": "",
            "ImuYaw": "",
            "CommStatus": "",
            "CommErrorCode": "",
            "CommErrorMessage": "",
            "PacketLoss": "",
            "LatencyMs": "",
            "RawComponent": self.raw_component,
            "RawJointType": self.stat,
            "SourceLine": "",
        }


FEEDBACK_CSV_COLUMNS = [
    "timestamp",
    "ComponentType",
    "JointID",
    "Position",
    "ActualVel",
    "Torque",
    "MotorCurrent",
    "Temperature",
    "ImuGyroX",
    "ImuGyroY",
    "ImuGyroZ",
    "ImuAccelX",
    "ImuAccelY",
    "ImuAccelZ",
    "ImuRoll",
    "ImuPitch",
    "ImuYaw",
    "CommStatus",
    "CommErrorCode",
    "CommErrorMessage",
    "PacketLoss",
    "LatencyMs",
    "RawComponent",
    "RawJointType",
    "SourceLine",
]

JointCallback = Callable[[str, object, List[NixJointSample]], None]


def _sdk_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _typedef_dir() -> Path:
    return _sdk_root() / "lcm_typedef" / "python"


def _repo_root() -> Path:
    return _sdk_root().parent


def feedback_schema_json_path() -> Path:
    return _repo_root() / "lumos_pipeline" / "src" / "lumos_pipeline" / "schemas" / "feedback_v1.json"


def resolve_feedback_schema_version() -> str:
    """Return the active feedback schema version, or ``v0.0`` when unavailable."""

    try:
        with feedback_schema_json_path().open("r", encoding="utf-8") as handle:
            version = json.load(handle).get("version", "")
    except (OSError, json.JSONDecodeError):
        return DEFAULT_FEEDBACK_SCHEMA_VERSION
    return str(version).strip() or DEFAULT_FEEDBACK_SCHEMA_VERSION


def _load_lcm_class(modname: str):
    """Load generated LCM Python classes despite module-vs-class imports.

    The generated files import nested types as modules, then call class methods
    on those names.  Existing SDK scripts work around this by replacing
    ``sys.modules[modname]`` with the class object; keep the same pattern here.
    """

    typedef_path = str(_typedef_dir())
    if typedef_path not in sys.path:
        sys.path.insert(0, typedef_path)
    module = importlib.import_module(modname)
    cls = getattr(module, modname, module)
    sys.modules[modname] = cls
    return cls


def load_joint_dataset_type():
    """Return the generated ``sdk_lcmt_joint_datasets`` class."""

    try:
        _load_lcm_class("sdk_lcmt_joint_data")
        return _load_lcm_class("sdk_lcmt_joint_datasets")
    except Exception as exc:  # pragma: no cover - exact import errors vary.
        raise LcmUnavailableError(
            "Failed to load generated LCM typedefs from "
            f"{_typedef_dir()}. Regenerate SDK Python typedefs if they are stale."
        ) from exc


def load_lcm_module():
    """Import Python LCM bindings with a clear fallback hint."""

    try:
        return importlib.import_module("lcm")
    except ImportError as exc:
        raise LcmUnavailableError(
            "Python LCM binding is not available. Install/build the lcm Python "
            "package, or use the shell-out fallback: lumos_sdk/build/"
            "nix_recv_all_data."
        ) from exc


class NixLcmSubscriber:
    """Thin subscriber for NIX ``JointsData`` feedback.

    这个类是给代码复用/测试用的；命令行入口在文件底部的 ``main()``。

    数据流：
        LCM bytes -> sdk_lcmt_joint_datasets.decode()
                  -> NixJointSample list
                  -> callback / CSV writer / stdout printer
    """

    def __init__(
        self,
        channel: str = DEFAULT_JOINT_CHANNEL,
        lcm_url: Optional[str] = None,
    ) -> None:
        self.channel = channel
        self.lcm_url = lcm_url or DEFAULT_LCM_URL

        # 这里才加载 lcm 和 typedef，避免 ``--help`` 或静态导入时强依赖 LCM 环境。
        self._lcm_mod = load_lcm_module()
        self._dataset_type = load_joint_dataset_type()
        try:
            self._lc = self._lcm_mod.LCM(self.lcm_url)
        except Exception as exc:
            raise LcmUnavailableError(
                "Failed to create LCM subscriber for "
                f"{self.lcm_url}. Check multicast routing/network setup, try "
                "--local for same-machine loopback tests, or use "
                "lumos_sdk/build/nix_recv_all_data."
            ) from exc
        self._subscription = None
        self._callback: Optional[JointCallback] = None
        self.message_count = 0
        self.sample_count = 0
        self.decode_error_count = 0
        self.last_received_at: Optional[float] = None
        self.last_samples: List[NixJointSample] = []

    def subscribe(self, callback: Optional[JointCallback] = None):
        """Subscribe to the configured joint channel.

        The callback receives ``(channel, decoded_message, samples)``.
        ``samples`` 是已经提取好的关节列表，通常业务代码只需要看它。
        """

        self._callback = callback
        self._subscription = self._lc.subscribe(self.channel, self._handle_message)
        return self._subscription

    def handle_timeout(self, timeout_ms: int = 200) -> int:
        """Handle one LCM event with timeout and return LCM's status code.

        LCM 的 ``handle_timeout`` 会阻塞最多 ``timeout_ms`` 毫秒：
        - 收到消息：调用 ``_handle_message``
        - 没收到消息：返回，由外层循环继续等
        """

        if self._subscription is None:
            self.subscribe()
        return int(self._lc.handle_timeout(timeout_ms))

    def listen(
        self,
        duration_sec: Optional[float] = None,
        once: bool = False,
        timeout_ms: int = 200,
    ) -> int:
        """Run the receive loop.

        Returns the number of decoded messages seen during this call.

        ``once=True`` 的语义是“收到一条消息后退出”，不是“只等一个 timeout”。
        所以如果没有发布者，建议同时传 ``duration_sec`` 做最大等待时间。
        """

        if self._subscription is None:
            self.subscribe()

        start_count = self.message_count
        deadline = None if duration_sec is None else time.monotonic() + duration_sec
        while True:
            try:
                self.handle_timeout(timeout_ms)
            except OSError as exc:
                # python-lcm 在 Ctrl+C 或底层 fd 被打断时可能抛这个错误，
                # 而不是标准 KeyboardInterrupt。向上交给 main() 做干净退出。
                raise LcmUnavailableError(f"LCM receive loop stopped: {exc}") from exc
            if once and self.message_count > start_count:
                break
            if deadline is not None and time.monotonic() >= deadline:
                break
        return self.message_count - start_count

    def _handle_message(self, channel: str, data: bytes) -> None:
        """Decode one raw LCM packet.

        ``sdk_lcmt_joint_datasets`` 是一个数组消息：
            msg.datasets_num 表示数组里有多少个关节
            msg.datasets[i] 是单个 sdk_lcmt_joint_data

        真正的字段名是 pos_high / vel / tor，不是 position / velocity / torque。
        """

        received_at = time.time()
        try:
            msg = self._dataset_type.decode(data)
            samples = self.samples_from_message(msg, received_at, raw_component=channel)
        except Exception:
            self.decode_error_count += 1
            raise

        self.message_count += 1
        self.sample_count += len(samples)
        self.last_received_at = received_at
        self.last_samples = samples
        if self._callback is not None:
            self._callback(channel, msg, samples)

    @staticmethod
    def samples_from_message(
        msg: object,
        timestamp: Optional[float] = None,
        raw_component: str = DEFAULT_JOINT_CHANNEL,
    ) -> List[NixJointSample]:
        """Decode a generated LCM message into typed joint samples."""

        sample_time = time.time() if timestamp is None else float(timestamp)
        datasets_num = int(getattr(msg, "datasets_num"))
        datasets = getattr(msg, "datasets")
        samples: List[NixJointSample] = []
        for joint_data in datasets[:datasets_num]:
            samples.append(
                NixJointSample(
                    timestamp=sample_time,
                    component_type=int(joint_data.component_type),
                    joint_id=int(joint_data.joint_id),
                    stat=int(joint_data.stat),
                    pos_high=float(joint_data.pos_high),
                    vel=float(joint_data.vel),
                    tor=float(joint_data.tor),
                    cur=float(joint_data.cur),
                    pos_low=float(joint_data.pos_low),
                    temp=int(joint_data.res4),
                    raw_component=raw_component,
                )
            )
        return samples


def write_feedback_rows(csv_path: str, rows: Iterable[dict], append: bool = True) -> None:
    """Write decoded samples using the current feedback CSV column names.

    新文件第一行写入 ``# feedback_schema=<version>``。版本优先来自
    ``lumos_pipeline`` 的 schema JSON；读不到时退回特殊默认值 ``v0.0``。
    pipeline 读取时使用 ``comment="#"`` 会跳过该元数据行；``lpx
    verify-schema --require-header`` 会用它确认 schema 版本。
    """

    path = Path(csv_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    file_exists = path.exists() and path.stat().st_size > 0
    mode = "a" if append else "w"
    write_header = not file_exists or not append
    with path.open(mode, newline="", encoding="utf-8") as handle:
        if write_header:
            handle.write(f"# feedback_schema={resolve_feedback_schema_version()}\n")
        writer = csv.DictWriter(handle, fieldnames=FEEDBACK_CSV_COLUMNS)
        if write_header:
            writer.writeheader()
        writer.writerows(rows)


def _print_samples(samples: Sequence[NixJointSample], limit: int) -> None:
    for sample in samples[:limit]:
        print(
            "  "
            f"component={sample.component_type} joint={sample.joint_id} "
            f"pos_high={sample.pos_high:+.6f} vel={sample.vel:+.6f} "
            f"tor={sample.tor:+.6f} cur={sample.cur:+.6f} "
            f"temp={sample.temp} stat={sample.stat}"
        )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Subscribe to NIX JointsData LCM feedback.",
    )
    parser.add_argument("--channel", default=DEFAULT_JOINT_CHANNEL)
    parser.add_argument("--url", default=None, help="Explicit LCM URL. Defaults to the SDK URL with ttl=255.")
    parser.add_argument("--local", action="store_true", help="Use ttl=0 for local mock pub/sub on this machine.")
    parser.add_argument("--duration", type=float, default=None, help="Max listen duration in seconds.")
    parser.add_argument("--once", action="store_true", help="Exit after the first decoded message.")
    parser.add_argument("--timeout-ms", type=int, default=200)
    parser.add_argument("--csv", default=None, help="Optional feedback CSV output path.")
    parser.add_argument("--print-every", type=int, default=1, help="Print every N decoded messages.")
    parser.add_argument("--print-limit", type=int, default=12, help="Max samples printed per message.")
    parser.add_argument("--quiet", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)

    try:
        url = LOCAL_LCM_URL if args.local and args.url is None else args.url
        subscriber = NixLcmSubscriber(channel=args.channel, lcm_url=url)
    except LcmUnavailableError as exc:
        print(f"[nix_lcm_sub] {exc}", file=sys.stderr)
        return 2

    def on_message(channel: str, _msg: object, samples: List[NixJointSample]) -> None:
        if args.csv:
            write_feedback_rows(args.csv, (sample.feedback_row() for sample in samples))
        if not args.quiet and subscriber.message_count % max(args.print_every, 1) == 0:
            print(
                f"[{channel}] messages={subscriber.message_count} "
                f"samples={len(samples)} total_samples={subscriber.sample_count}"
            )
            _print_samples(samples, limit=max(args.print_limit, 0))

    subscriber.subscribe(on_message)

    if not args.quiet:
        print(f"[nix_lcm_sub] listening channel={subscriber.channel} url={subscriber.lcm_url}")
    try:
        subscriber.listen(duration_sec=args.duration, once=args.once, timeout_ms=args.timeout_ms)
    except KeyboardInterrupt:
        if not args.quiet:
            print("\n[nix_lcm_sub] interrupted")
    except LcmUnavailableError as exc:
        if not args.quiet:
            print(f"\n[nix_lcm_sub] {exc}", file=sys.stderr)

    if not args.quiet:
        print(
            "[nix_lcm_sub] done "
            f"messages={subscriber.message_count} "
            f"samples={subscriber.sample_count} "
            f"decode_errors={subscriber.decode_error_count}"
        )
        if subscriber.message_count == 0 and "ttl=0" in subscriber.lcm_url:
            print(
                "[nix_lcm_sub] hint: ttl=0 is for local mock loopback only. "
                "Use the default ttl=255, or pass --url explicitly, when listening to a robot over Ethernet."
            )
    return 0 if subscriber.decode_error_count == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
