#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NIX2 SDK Python 调试脚本（极简版）

功能：
    1. 订阅机器人数据：关节状态 / IMU / 机器人状态
    2. 下发单个关节的目标位置

用法（在 lumos_sdk 根目录运行）：
    # 只监听数据
    python3 python/sdk_debug.py listen

    # 进入 SDK 模式
    python3 python/sdk_debug.py mode 1     # 1=SDK控制模式, 0=RL模式

    # 切状态
    python3 python/sdk_debug.py state 1    # 1=RESET, 2=STAND
    python3 python/sdk_debug.py state 2

    # 下发单关节目标位置（component_type, joint_id, tarPos, kp, kd）
    # 例：腰部(WAIST=7) 第 0 个关节，目标 0.3 rad，kp=160，kd=6
    python3 python/sdk_debug.py joint 7 0 0.3 160 6

前置：先 source config_network_lcm.sh <你的网卡> 配置多播路由。
"""

import os
import sys
import time
import threading

# 让脚本能直接 import lcm_typedef/python 里的消息类型
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(THIS_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, "lcm_typedef", "python"))

import lcm

# 生成的 LCM Python 代码里嵌套类型用 `import xxx` 但又直接当类调用 xxx._get_hash_recursive，
# 这里把每个模块替换成它里面的同名类，绕过该 bug。
def _load_lcm_class(modname: str):
    import importlib
    m = importlib.import_module(modname)
    cls = getattr(m, modname)
    sys.modules[modname] = cls  # 让其它生成文件 `import modname` 时拿到的是类
    return cls

sdk_lcmt_joint_cmd      = _load_lcm_class("sdk_lcmt_joint_cmd")
sdk_lcmt_joint_data     = _load_lcm_class("sdk_lcmt_joint_data")
sdk_lcmt_joint_cmds     = _load_lcm_class("sdk_lcmt_joint_cmds")
sdk_lcmt_joint_datasets = _load_lcm_class("sdk_lcmt_joint_datasets")
sdk_lcmt_type           = _load_lcm_class("sdk_lcmt_type")
robot_cmd_lcmt          = _load_lcm_class("robot_cmd_lcmt")
robot_status_lcmt       = _load_lcm_class("robot_status_lcmt")
microstrain_lcmt        = _load_lcm_class("microstrain_lcmt")


# ── 与 C++ SDK 保持一致 ─────────────────────────────────────────────
LCM_URL = "udpm://239.255.76.67:7667?ttl=255"

# 发布通道
CH_JOINT_CMDS = "lcm_joint_cmd"
CH_MODE_CMD   = "lcm_control_type"
CH_ROBOT_CMD  = "lcm_robot_cmd"
# 订阅通道
CH_JOINT_DATA = "lcm_joint_data"
CH_IMU        = "lcm_imu_data"
CH_STATUS     = "lcm_robot_status"

# 组件类型（对应 SdkComponentType）
COMPONENT_NAME = {1: "ARM_L", 2: "ARM_R", 7: "WAIST", 8: "LEG_L", 9: "LEG_R"}
STATE_NAME = {0: "NOT_A_STATE", 1: "RESET", 2: "STAND", 3: "RL_WALK",
              5: "RL_LIEDOWN", 6: "RL_MIMIC", 11: "RL_NAV", 12: "RL_WALK_AMP",
              20: "BY_MIMIC", 21: "BFM_MIMIC"}


# ═══════════════════════════════════════════════════════════════════
# 订阅回调
# ═══════════════════════════════════════════════════════════════════
class Listener:
    def __init__(self, lc):
        self.lc = lc
        self.joint_cnt = 0
        self.imu_cnt = 0
        self.status_cnt = 0
        lc.subscribe(CH_JOINT_DATA, self._on_joint)
        lc.subscribe(CH_IMU, self._on_imu)
        lc.subscribe(CH_STATUS, self._on_status)

    def _on_joint(self, channel, data):
        msg = sdk_lcmt_joint_datasets.decode(data)
        self.joint_cnt += 1
        # 每 100 帧打印一次完整的关节数据（约 1s @100Hz）
        if self.joint_cnt % 100 == 1:
            print(f"[JointsData] num={msg.datasets_num}")
            for d in msg.datasets[:msg.datasets_num]:
                cname = COMPONENT_NAME.get(d.component_type, "?")
                print(f"  {cname}[{d.joint_id}] pos={d.pos_high:+.3f} "
                      f"vel={d.vel:+.3f} tor={d.tor:+.3f} stat={d.stat}")

    def _on_imu(self, channel, data):
        msg = microstrain_lcmt.decode(data)
        self.imu_cnt += 1
        if self.imu_cnt % 200 == 1:
            print(f"[IMU] rpy={msg.navRPY[0]:+.3f},{msg.navRPY[1]:+.3f},{msg.navRPY[2]:+.3f} "
                  f"acc={msg.acc[0]:+.2f},{msg.acc[1]:+.2f},{msg.acc[2]:+.2f}")

    def _on_status(self, channel, data):
        msg = robot_status_lcmt.decode(data)
        self.status_cnt += 1
        print(f"[Status] state={STATE_NAME.get(msg.state, '?')}({msg.state}) "
              f"type={msg.type}")


def start_listen_thread(lc):
    """开一个后台线程持续 handle LCM 消息"""
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            lc.handle_timeout(200)  # ms

    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return stop, t


# ═══════════════════════════════════════════════════════════════════
# 发布
# ═══════════════════════════════════════════════════════════════════
def send_mode(lc, mode: int):
    msg = sdk_lcmt_type()
    msg.controller_type = mode
    lc.publish(CH_MODE_CMD, msg.encode())
    print(f"[Send] mode={mode}  (0=RL, 1=SDK)")


def send_state(lc, state: int, vx=0.0, vy=0.0, vyaw=0.0):
    msg = robot_cmd_lcmt()
    msg.state = state
    msg.x = vx
    msg.y = vy
    msg.yaw = vyaw
    msg.policy_type = 0
    lc.publish(CH_ROBOT_CMD, msg.encode())
    print(f"[Send] state={STATE_NAME.get(state, '?')}({state})")


def send_single_joint(lc, component_type: int, joint_id: int,
                      tar_pos: float, kp: float = 60.0, kd: float = 2.0,
                      ctrl_word: int = 3):
    """下发单个关节目标位置（MIT/PD 控制：ctrlWord=3）"""
    cmds = sdk_lcmt_joint_cmds()
    cmd = sdk_lcmt_joint_cmd()
    cmd.component_type = component_type
    cmd.joint_id = joint_id
    cmd.ctrlWord = ctrl_word
    cmd.tarPos = tar_pos
    cmd.tarVel = 0.0
    cmd.tarCur = 0.0
    cmd.tarTor = 0.0
    cmd.res1 = kp   # kp
    cmd.res2 = kd*1.2   # kd
    cmd.res3 = 0.0
    cmd.res4 = 0.0

    cmds.cmds_num = 1
    cmds.cmds = [cmd]
    lc.publish(CH_JOINT_CMDS, cmds.encode())

    cname = COMPONENT_NAME.get(component_type, "?")
    print(f"[Send] joint {cname}[{joint_id}] -> pos={tar_pos:.3f} kp={kp} kd={kd}")


# ═══════════════════════════════════════════════════════════════════
# 主入口
# ═══════════════════════════════════════════════════════════════════
def usage():
    print(__doc__)
    sys.exit(1)


def main():
    if len(sys.argv) < 2:
        usage()

    lc = lcm.LCM(LCM_URL)
    cmd = sys.argv[1]

    if cmd == "listen":
        listener = Listener(lc)
        print(f"订阅中：{CH_JOINT_DATA}, {CH_IMU}, {CH_STATUS}  (Ctrl-C 退出)")
        try:
            while True:
                lc.handle_timeout(500)
        except KeyboardInterrupt:
            print(f"\n收到 joint={listener.joint_cnt} imu={listener.imu_cnt} "
                  f"status={listener.status_cnt}")

    elif cmd == "mode":
        send_mode(lc, int(sys.argv[2]))

    elif cmd == "state":
        send_state(lc, int(sys.argv[2]))

    elif cmd == "joint":
        # joint <component_type> <joint_id> <tarPos> [kp] [kd]
        if len(sys.argv) < 5:
            usage()
        ctype = int(sys.argv[2])
        jid   = int(sys.argv[3])
        pos   = float(sys.argv[4])
        kp    = float(sys.argv[5]) if len(sys.argv) > 5 else 60.0
        kd    = float(sys.argv[6]) if len(sys.argv) > 6 else 2.0

        # 边发边监听，便于看到关节反馈
        listener = Listener(lc)
        stop, _ = start_listen_thread(lc)
        try:
            print("持续以 100Hz 下发目标位置，Ctrl-C 退出")
            while True:
                send_single_joint(lc, ctype, jid, pos, kp, kd)
                time.sleep(0.01)
        except KeyboardInterrupt:
            stop.set()
            print("\n停止下发")

    else:
        usage()


if __name__ == "__main__":
    main()
