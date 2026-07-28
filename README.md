# Lumos SDK

`lumos_sdk` 当前已对齐 `lumos_controller` 的状态机、LCM topic 和消息类型。SDK 不再维护旧的 `sdk_lcmt_*`、`control_type_lcmt` 或 `sdk_debug.py` 语义。

完整说明见 [docs/user_guide.md](docs/user_guide.md)。

## 当前控制链路

| 功能 | Topic | Type |
| --- | --- | --- |
| 高层状态/速度命令 | `lcm_robot_cmd` | `robot_cmd_lcmt` |
| 关节级命令 | `lcm_joint_cmd` | `joint_cmds_lcmt` |
| 关节反馈 | `lcm_joint_data` | `joint_datasets_lcmt` |
| 机器人状态 | `lcm_robot_status` | `robot_status_lcmt` |
| IMU 数据 | `lcm_imu_data` | `imu_data_lcmt` |
| 手臂命令 | `lcm_arm_cmd` | `arm_cmd_lcmt` |

关节级控制只在 controller 的 `DEBUG(10)` 状态下生效。标准顺序是：

```text
RESET(1) -> STAND(2) -> DEBUG(10) -> publish lcm_joint_cmd/joint_cmds_lcmt
```

进入 DEBUG，也就是现场常说的 SDK 模式，目前有两种方式：

1. 手柄按键：`Back + Home`。
2. 通过 `lumos_sdk` 发布状态命令切换到 `DEBUG(10)`，推荐使用 `python/nix_debug_state.py enter`。

从 `lumos_controller` 当前 NIX2 配置确认，进入 DEBUG 有状态机前提：

- 只允许从 `RESET` 或 `STAND` 转到 `DEBUG`。
- 不能从 `NOT_A_STATE`、`RL_WALK`、`RL_WALK_AMP`、`MIMIC`、`RL_LIEDOWN` 等状态直接进入 DEBUG；需要先按允许路径回到 `RESET` 或 `STAND`。
- 通过 `lcm_robot_cmd` 切状态时，`x/y/yaw` 必须为 0；带速度的消息只作为速度命令处理，不触发状态切换。
- `DebugState` 进入后默认保持当前关节位置，不会自动插值到站姿；所以建议先完成 `RESET -> STAND` 并确认机器人稳定，再进入 DEBUG。

进入 DEBUG 后，SDK 关节控制可以走两种网络链路。两种链路使用同一套 LCM topic/type 和同一套脚本，不需要维护两套控制代码：

1. 网线链路：连接机器人网口，配置本机有线网卡静态 IPv4 和 LCM 组播路由，然后运行 `lumos_sdk` 下的脚本控制机器人。
2. Wi-Fi 链路：连接机器人自身热点，例如 SN 尾号 `005` 的机器人热点通常是 `nix_NIX005`，密码是 `nix_NIX005_pd`；确认 LCM 组播路由指向 Wi-Fi 网卡后，运行同一套 `lumos_sdk` 脚本控制机器人。

Wi-Fi 只是网络承载方式，不是新的 DEBUG 进入方式，也不是新的消息协议。连接机器人 Wi-Fi 时，本机可能临时失去外网，这是正常现象。

## 环境配置

### 网线链路

机器人网口常用 IP 是 `192.168.54.110`。SDK 运行设备需要先把连接机器人的有线网卡配置到同一网段，再把 LCM 组播路由放到这块网卡上。

先用 `ifconfig` 找到连接机器人网线的本机网卡名：

```bash
ifconfig
```

通常应选择有线网卡，而不是 Wi-Fi、`lo`、`docker0` 等虚拟网卡。判断依据：

- 插着机器人网线后接口处于 `UP` / `RUNNING`。
- 接口上配置了和机器人同网段的地址，例如 `inet 192.168.xx.xx`。
- 网卡名可能是 `enx...`、`eth...`、`enp...`，不要照抄文档里的示例名。

假设找到的网卡名是 `IFACE`，先给这块有线网卡配置静态 IPv4。当前真机测试使用：

- Address: `192.168.54.111`
- Netmask: `255.255.0.0`
- Robot IP: `192.168.54.110`

可以在系统网络设置里配置，也可以临时用命令配置：

```bash
sudo ifconfig IFACE 192.168.54.111 netmask 255.255.0.0 up
```

确认能 ping 通机器人后，再配置 LCM 组播：

```bash
cd lumos_sdk
bash config_network_lcm.sh IFACE
```

例如实际网卡叫 `enp3s0`，就运行：

```bash
bash config_network_lcm.sh enp3s0
```

### Wi-Fi 链路

机器人身上可能提供 Wi-Fi 热点。热点名和密码通常由机器人型号和 SN 派生，例如 SN 尾号为 `005` 时：

- SSID: `nix_NIX005`
- Password: `nix_NIX005_pd`

连接热点后，把 LCM 组播路由指到 Wi-Fi 网卡。假设 Wi-Fi 网卡名是 `wlp3s0`：

```bash
nmcli dev wifi connect nix_NIX005 password nix_NIX005_pd ifname wlp3s0
cd lumos_sdk
bash config_network_lcm.sh wlp3s0
```

Wi-Fi 链路可用于进入 DEBUG，也可用于进入 DEBUG 后运行同一套 SDK 关节控制脚本。它和网线链路的差异只在网络配置，不在 SDK topic/type 或脚本接口。

真机测试前确认机器人端 `lumos_controller` 正在运行、急停可用、机器人支撑和周围空间安全。手柄可以用于进入 DEBUG；进入 DEBUG 后通过 SDK 做关节控制时，不要让手柄或其它控制源持续发送冲突命令。

## 编译

```bash
cd lumos_sdk
cmake -S . -B build
cmake --build build
```

构建后会生成：

- `build/nix_lcm_sub`
- `build/nix_robot_state`
- `build/nix_debug_state`
- `build/nix_joint_cmd`
- `build/nix_data_collection`
- `build/lud_send_robot_cmd`
- `build/lud_send_joint_cmds`

所有 C++ 示例都支持 `--help`，`--help` 不会初始化 LCM，也不会发命令。

## 真机基础测试顺序

先做被动监听，不发控制命令：

```bash
cd lumos_sdk
./build/nix_lcm_sub
```

或 Python 监听：

```bash
python3 python/nix_lcm_sub.py --once --print-limit 21
```

进入 DEBUG 前，先 dry-run：

```bash
python3 python/nix_debug_state.py enter --dry-run
python3 python/nix_joint_cmd.py single --component WAIST --joint-id 0 --pos 0.0 --dry-run
```

进入 DEBUG。也可以用手柄 `Back + Home` 进入；如果当前使用机器人 Wi-Fi 链路，仍然运行同一个 SDK 命令。根据 controller 状态机，当前必须先处于 `RESET` 或 `STAND`，推荐用 SDK 命令执行完整 `RESET -> STAND -> DEBUG`：

```bash
python3 python/nix_debug_state.py enter --timeout 25 --stand-settle 2
```

确认 DEBUG 下能收到 21 个关节反馈：

```bash
python3 python/nix_lcm_sub.py --once --print-limit 21
```

最小关节命令建议使用当前关节位置做保持命令。例如真机反馈里 `WAIST[0] pos_high=0.0004`：

```bash
python3 python/nix_joint_cmd.py single \
  --component WAIST \
  --joint-id 0 \
  --pos 0.0004 \
  --kp 60 \
  --kd 2 \
  --duration 0.2 \
  --rate-hz 20
```

测试结束回到 STAND：

```bash
python3 python/nix_debug_state.py leave --timeout 25
```

## Python 工具

- `python/nix_debug_state.py`: NIX DEBUG 专用入口，负责进入/退出 `DEBUG(10)`。
- `python/nix_robot_state.py`: 通用状态切换/监听工具。
- `python/nix_lcm_sub.py`: 订阅 `lcm_joint_data`，可写 feedback CSV。
- `python/nix_lcm_pub_mock.py`: 本机 mock 发布 `lcm_joint_data`。
- `python/nix_joint_cmd.py`: NIX 21 关节命令工具，发布 `joint_cmds_lcmt`。
- `python/nix_joint_cmd_sub.py`: 订阅 `lcm_joint_cmd`，用于本机或调试监听。
- `python/lus_joint_cmd.py`: LUS2 27 关节命令工具。
- `python/sim2real_lumos.py`: NIX2 ONNX 策略回放入口，需要 `onnx` 和 `onnxruntime`。
- `python/sim2real_mimic.py`: Mimic ONNX 策略回放入口，需要 `onnxruntime` 和模型目录。

本机 LCM 回环测试：

```bash
# terminal A
python3 python/nix_lcm_sub.py --local --once --print-limit 2

# terminal B
python3 python/nix_lcm_pub_mock.py --local --count 1 --joints 21
```

## C++ 示例边界

- `nix_lcm_sub`: 被动监听，安全，不发状态或关节命令。
- `nix_robot_state`: 默认只做 `RESET -> STAND` smoke test；`--walk-test` 才发送低速行走命令。
- `nix_debug_state`: 发送 `RESET -> STAND -> DEBUG -> RESET -> STAND`，不发关节目标。
- `nix_joint_cmd`: 会进入 DEBUG 并发布关节命令；真机运行前先用 Python 工具做小目标验证。
- `nix_data_collection`: 主动采集场景，会发送 RESET/STAND/DEBUG/RL/MIMIC 等状态，不是普通 smoke test。
- `lud_send_robot_cmd`、`lud_send_joint_cmds`: LUD 机器人示例，不用于 NIX 真机。

## 已验证状态

当前真机测试结果：

- C++ 构建通过，所有 C++ 示例 `--help` 通过。
- Python 脚本语法编译通过，所有 Python 脚本 `--help` 通过。
- 真机通过 `nix_lcm_sub`、`nix_robot_state`、`nix_debug_state`。
- 真机通过 `nix_debug_state.py`、`nix_robot_state.py stand`、`nix_lcm_sub.py`。
- 真机通过 `nix_joint_cmd.py` 最小 WAIST 保持命令。
- `sim2real_lumos.py` / `sim2real_mimic.py` 当前只验证入口；实际策略回放需要安装 ONNX 依赖并提供模型/动作文件。

## C++ API 摘要

```cpp
SdkRobotManager manager;
manager.Init();

manager.SendRobotCmd(SdkStateType::RESET);
manager.SendRobotCmd(SdkStateType::STAND);
manager.SendRobotCmd(SdkStateType::DEBUG);
manager.SendJointCmds(cmds);
```

常用回调：

```cpp
bool SetJointDataCb(JointDateCb cb);  // const joint_datasets_lcmt*
bool SetImuDataCb(ImuDateCb cb);      // const imu_data_lcmt*
bool SetRobotStatusCb(RobotStatusCb cb); // const robot_status_lcmt*
```
