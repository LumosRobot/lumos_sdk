# Lumos SDK User Guide

本文档是 `lumos_sdk` 当前唯一完整使用说明。SDK 已按 `lumos_controller` 的状态机、LCM topic 和消息类型对齐，不再保留旧的 `sdk_lcmt_*`、`control_type_lcmt`、`sdk_debug.py` 运行路径。

## 1. 控制链路

`lumos_controller` 使用状态机控制。SDK 当前只使用 controller 运行路径里的 topic 和 type：

| 功能 | Topic | Type | 说明 |
| --- | --- | --- | --- |
| 高层状态/速度命令 | `lcm_robot_cmd` | `robot_cmd_lcmt` | `LumosRobot::handleLCMControl` 订阅 |
| 手臂命令 | `lcm_arm_cmd` | `arm_cmd_lcmt` | controller 手臂命令入口 |
| 关节级命令 | `lcm_joint_cmd` | `joint_cmds_lcmt` | 只在 `DEBUG(10)` 状态下由 `DebugState` 应用 |
| 关节反馈 | `lcm_joint_data` | `joint_datasets_lcmt` | `DebugState` 聚合发布 |
| 机器人状态 | `lcm_robot_status` | `robot_status_lcmt` | SDK 用它确认状态切换 |
| IMU 数据 | `lcm_imu_data` | `imu_data_lcmt` | SDK 直接订阅 |

关节级控制必须先进入 `DEBUG(10)`：

```text
RESET(1) -> STAND(2) -> DEBUG(10) -> publish lcm_joint_cmd/joint_cmds_lcmt
```

如果未进入 `DEBUG(10)`，controller 可以收到 `lcm_joint_cmd`，但不会在普通状态下执行关节命令。

进入 `DEBUG(10)`，也就是现场常说的 SDK 模式，目前有两种方式：

| 方式 | 当前状态 | 说明 |
| --- | --- | --- |
| 手柄 `Back + Home` | 可用 | 由手柄直接触发进入 DEBUG |
| `lumos_sdk` 状态命令 | 可用 | 推荐使用 `python/nix_debug_state.py enter`，会执行 `RESET -> STAND -> DEBUG` 并等待 status 确认 |

退出 `DEBUG(10)` 也有两种方式：

| 方式 | 当前状态 | 说明 |
| --- | --- | --- |
| 手柄 `Start` | 可用 | 请求从 DEBUG 切到 `STAND` |
| 手柄双击 `Back` | 可用 | 请求从 DEBUG 切到 `RESET`；单按 Back 只是第一次 RESET 按键记录，`Home + Back` 是进入 DEBUG |
| `lumos_sdk` 状态命令 | 可用 | 推荐使用 `python/nix_debug_state.py leave` 切回 `STAND`；也可以用 `python/nix_robot_state.py state RESET --wait` 请求回 `RESET` |

从 `lumos_controller` 当前 NIX2 配置确认，进入 DEBUG 的状态机前提是：

| 当前状态 | 是否可直接进入 DEBUG | 说明 |
| --- | --- | --- |
| `RESET` | 可以 | `nix2.yaml` 中 `RESET` 的可达状态包含 `DEBUG` |
| `STAND` | 可以 | `nix2.yaml` 中 `STAND` 的可达状态包含 `DEBUG` |
| `NOT_A_STATE` | 不可以 | 只能先到 `RESET` |
| `RL_WALK` / `RL_WALK_AMP` / `RL_RUN_AMP` | 不可以 | 需要先按允许路径回到 `RESET`，再进入 `STAND/DEBUG` |
| `ST_MIMIC` / `BY_MIMIC` / `BFM_MIMIC` | 不可以 | 需要先回到 `RESET` |
| `RL_LIEDOWN` | 不可以 | NIX2 配置中只能先回 `RESET` |

其它前提：

- 通过 `lcm_robot_cmd` 切换状态时，`x/y/yaw` 必须为 0。controller 中带速度的消息会被当作速度命令，不触发状态切换。
- 手柄 `Back + Home` 也会走相同的状态转移检查，不会绕过 `safe_to_go`。
- `DebugState::onEnter()` 会把当前关节位置作为保持目标，不会自动插值到站姿。因此真机建议先完成 `RESET -> STAND`，确认机器人稳定，再进入 DEBUG。
- 进入 DEBUG 后才会执行 `lcm_joint_cmd/joint_cmds_lcmt` 关节命令，并发布聚合 `lcm_joint_data/joint_datasets_lcmt`。
- NIX2 配置中 `DEBUG` 只允许转到 `RESET` 或 `STAND`。手柄 `Start`、双击 `Back` 和 SDK 状态命令都会走同一套 `safe_to_go` 检查。

## 2. 网络配置

进入 DEBUG 后，SDK 关节控制可以走两种网络链路。两种链路使用同一套 LCM topic/type 和同一套脚本，不需要维护两套控制代码：

| 链路 | 用途 | 说明 |
| --- | --- | --- |
| 网线 | 进入 DEBUG、订阅反馈、下发关节命令 | 更推荐，链路稳定，IP 和路由可控 |
| 机器人 Wi-Fi | 进入 DEBUG、订阅反馈、下发关节命令 | 可行性取决于热点网络是否允许 LCM 组播；连接后本机可能临时失去外网 |

Wi-Fi 只是网络承载方式，不是新的 DEBUG 进入方式，也不是新的消息协议。进入 DEBUG 仍然只有手柄和 `lumos_sdk` 状态命令两类方式。

### 2.1 网线链路

机器人网口常用 IP 是 `192.168.54.110`。SDK 运行设备需要先把连接机器人的有线网卡配置到同一网段，再把 LCM 组播路由放到这块网卡。

先用 `ifconfig` 找到连接机器人网线的本机网卡名：

```bash
ifconfig
```

选择网卡时不要照抄某台机器的示例名。通常应选择有线网卡，而不是 Wi-Fi、`lo`、`docker0` 等虚拟网卡。判断依据：

- 插着机器人网线后接口处于 `UP` / `RUNNING`。
- 接口上配置了和机器人同网段的地址，例如 `inet 192.168.xx.xx`。
- 网卡名可能是 `enx...`、`eth...`、`enp...`。

假设找到的网卡名是 `IFACE`，先给这块有线网卡配置静态 IPv4。当前真机测试使用：

- Address: `192.168.54.111`
- Netmask: `255.255.0.0`
- Robot IP: `192.168.54.110`

可以在系统网络设置里配置，也可以临时用命令配置：

```bash
sudo ifconfig IFACE 192.168.54.111 netmask 255.255.0.0 up
```

确认能 ping 通机器人：

```bash
ping 192.168.54.110
```

然后配置 LCM 组播：

```bash
cd lumos_sdk
bash config_network_lcm.sh IFACE
```

例如实际网卡叫 `enp3s0`，就运行：

```bash
bash config_network_lcm.sh enp3s0
```

等价核心操作如下，其中 `IFACE` 同样要替换成实际网卡名：

```bash
sudo ifconfig IFACE 192.168.54.111 netmask 255.255.0.0 up
sudo ifconfig IFACE multicast
sudo ip route del 224.0.0.0/4 dev lo 2>/dev/null || true
sudo ip route add 224.0.0.0/4 dev IFACE
```

确认网卡和路由：

```bash
ifconfig IFACE
ip route show
```

### 2.2 Wi-Fi 链路

机器人身上可能提供 Wi-Fi 热点。热点名和密码通常由机器人型号和 SN 派生，例如 SN 尾号为 `005` 时：

- SSID: `nix_NIX005`
- Password: `nix_NIX005_pd`

先用 `ifconfig` 或 `ip addr` 找到本机 Wi-Fi 网卡名。常见名称是 `wlp...` 或 `wlan...`。假设 Wi-Fi 网卡名是 `wlp3s0`：

```bash
nmcli dev wifi connect nix_NIX005 password nix_NIX005_pd ifname wlp3s0
cd lumos_sdk
bash config_network_lcm.sh wlp3s0
```

确认 Wi-Fi 和 LCM 组播路由：

```bash
ifconfig wlp3s0
ip route show
```

连接机器人 Wi-Fi 后，本机可能临时失去外网。如果需要恢复原 Wi-Fi，可以用 NetworkManager 切回原连接，例如：

```bash
nmcli connection up "原来的 Wi-Fi 连接名"
```

Wi-Fi 链路配置完成后，进入 DEBUG 和关节控制命令与网线链路完全相同：

```bash
python3 python/nix_debug_state.py enter --timeout 25 --stand-settle 2
python3 python/nix_lcm_sub.py --once --print-limit 21
python3 python/nix_joint_cmd.py single --component WAIST --joint-id 0 --pos 0.0 --dry-run
```

### 2.3 真机检查

真机测试前确认：

- 机器人端 `lumos_controller` 正在运行。
- 急停可用。
- 机器人支撑和周围空间安全。
- 手柄可以用于进入 DEBUG；进入 DEBUG 后通过 SDK 做关节控制时，不要让手柄或其它控制源持续发送冲突命令。

## 3. 编译和静态检查

```bash
cd lumos_sdk
cmake -S . -B build
cmake --build build
python3 -m py_compile python/*.py lcm_typedef/python/*.py
```

所有 C++ 示例都支持 `--help`，且 `--help` 不会初始化 LCM 或发命令：

```bash
./build/nix_robot_state --help
./build/nix_debug_state --help
./build/nix_joint_cmd --help
```

所有 Python 脚本也支持 `--help`：

```bash
python3 python/nix_debug_state.py --help
python3 python/nix_joint_cmd.py --help
```

## 4. 真机基础验证流程

### 4.1 被动监听

先确认真机 LCM 基础链路，不发送任何状态或关节命令：

```bash
cd lumos_sdk
./build/nix_lcm_sub
```

在非 DEBUG 状态下，通常能看到 `lcm_imu_data` 约 50Hz；`lcm_joint_data` 可能为 0，这是正常的，因为聚合关节反馈通常在 DEBUG 下发布。

Python 版本：

```bash
python3 python/nix_lcm_sub.py --duration 5
```

### 4.2 进入 DEBUG

进入 DEBUG 有两种方式：手柄 `Back + Home`，或者通过 SDK 状态命令。当前文档里的可复现命令使用 SDK 状态命令；网线和 Wi-Fi 链路都运行同一个命令。NIX2 当前只能从 `RESET` 或 `STAND` 直接进入 DEBUG，所以这里使用完整 `RESET -> STAND -> DEBUG` 流程。

先 dry-run：

```bash
python3 python/nix_debug_state.py enter --dry-run
```

真机进入 DEBUG：

```bash
python3 python/nix_debug_state.py enter --timeout 25 --stand-settle 2
```

成功标志：

```text
已确认：state=RESET(1)
已确认：state=STAND(2)
已确认：state=DEBUG(10)
```

### 4.3 确认 21 关节反馈

```bash
python3 python/nix_lcm_sub.py --once --print-limit 21
```

应满足：

- `messages=1`
- `samples=21`
- `decode_errors=0`
- 关节组件通常为 `ARM_L 4 + ARM_R 4 + LEG_L 6 + LEG_R 6 + WAIST 1`

### 4.4 最小关节命令测试

先读取当前关节位置：

```bash
python3 python/nix_lcm_sub.py --once --print-limit 21
```

选择当前 `pos_high` 做保持命令。例如 `WAIST[0] pos_high=0.0004`：

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

不要一开始就下发大幅目标或 replay。第一次必须使用 `--dry-run` 检查目标：

```bash
python3 python/nix_joint_cmd.py single \
  --component WAIST \
  --joint-id 0 \
  --pos 0.0004 \
  --kp 60 \
  --kd 2 \
  --dry-run
```

### 4.5 退出 DEBUG

```bash
python3 python/nix_debug_state.py leave --timeout 25
```

成功标志：

```text
已确认：state=STAND(2)
```

## 5. Python 工具

### `nix_debug_state.py`

NIX DEBUG 专用入口，只发布 `lcm_robot_cmd`，不发布关节命令。

```bash
python3 python/nix_debug_state.py enter --timeout 25 --stand-settle 2
python3 python/nix_debug_state.py leave --timeout 25
python3 python/nix_debug_state.py --dry-run
```

### `nix_robot_state.py`

通用状态切换和监听工具。

```bash
python3 python/nix_robot_state.py list
python3 python/nix_robot_state.py stand --timeout 25 --stand-settle 2
python3 python/nix_robot_state.py listen --duration 10
python3 python/nix_robot_state.py state STAND --wait --timeout 25
```

注意：`state STATE --wait` 依赖 controller 发布新的 `lcm_robot_status`。如果机器人已经处在同一个状态，controller 可能不会重复发布确认；真机 smoke test 优先使用 `stand` 或 `nix_debug_state.py enter/leave`。

### `nix_lcm_sub.py`

订阅 `lcm_joint_data`，解码 `joint_datasets_lcmt`，可选写 feedback CSV。

```bash
python3 python/nix_lcm_sub.py --once --print-limit 21
python3 python/nix_lcm_sub.py --duration 10 --print-every 100 --print-limit 5
python3 python/nix_lcm_sub.py --duration 10 --csv build/nix_feedback_hw.csv
```

CSV 第一行会写入 schema 版本：

```text
# feedback_schema=v1.0
```

CSV 中 `JointID` 使用 `<ComponentType>:<joint_id>` 组合键，例如 `8:0`、`9:5`、`7:0`，避免不同组件的局部 joint_id 冲突。

### `nix_lcm_pub_mock.py`

本机 mock 发布 `lcm_joint_data`，用于不连真机时验证 Python typedef 和订阅逻辑。

```bash
# terminal A
python3 python/nix_lcm_sub.py --local --once --print-limit 2

# terminal B
python3 python/nix_lcm_pub_mock.py --local --count 1 --joints 21
```

### `nix_joint_cmd.py`

NIX 21 关节命令工具，发布 `joint_cmds_lcmt` 到 `lcm_joint_cmd`。它不负责进入或退出 DEBUG。

查看映射：

```bash
python3 python/nix_joint_cmd.py list
```

单关节：

```bash
python3 python/nix_joint_cmd.py single \
  --component WAIST \
  --joint-id 0 \
  --pos 0.0004 \
  --kp 60 \
  --kd 2 \
  --dry-run
```

按 NIX 全局索引：

```bash
python3 python/nix_joint_cmd.py global \
  --index 12 \
  --pos 0.0004 \
  --duration 0.2 \
  --rate-hz 20 \
  --dry-run
```

批量目标：

```bash
python3 python/nix_joint_cmd.py batch \
  --target WAIST:0:0.0004:60:2 \
  --target LEG_L:3:0.16:60:2 \
  --dry-run
```

文件目标，CSV 至少包含 `component,joint_id,pos`，可选列为 `kp,kd,vel,tor,cur,ctrl_word`：

```csv
component,joint_id,pos,kp,kd
WAIST,0,0.0004,60,2
```

```bash
python3 python/nix_joint_cmd.py file --path build/nix_joint_targets.csv --dry-run
```

Replay 需要模型目录：

```bash
python3 python/nix_joint_cmd.py replay \
  --model-dir models/nix2_policy/sanlin_04101426 \
  --loops 1 \
  --dry-run
```

### `nix_joint_cmd_sub.py`

订阅 `lcm_joint_cmd`，用于确认 SDK 发布的关节命令内容。

```bash
python3 python/nix_joint_cmd_sub.py --local --once --print-limit 5
```

### `lus_joint_cmd.py`

LUS2 27 关节命令工具，发布同样的 `joint_cmds_lcmt`。不要用于 NIX 21 关节真机。

```bash
python3 python/lus_joint_cmd.py list
python3 python/lus_joint_cmd.py stand --dry-run
```

### `sim2real_lumos.py` / `sim2real_mimic.py`

策略回放入口。当前环境若缺少依赖，会给出明确错误，不再 traceback。

需要依赖：

- `sim2real_lumos.py`: `onnx`, `onnxruntime`
- `sim2real_mimic.py`: `onnxruntime`, `PyYAML`

需要模型/动作文件，不属于基础 SDK smoke test。真机运行前必须先完成 DEBUG、feedback、最小关节命令测试。

## 6. C++ 示例

| 可执行文件 | 行为 | 真机风险 |
| --- | --- | --- |
| `nix_lcm_sub` | 被动订阅 IMU/status/joint feedback，写 CSV | 低，不发命令 |
| `nix_robot_state` | 默认 `RESET -> STAND`，可选 `--walk-test` | 默认中等，`--walk-test` 高 |
| `nix_debug_state` | `RESET -> STAND -> DEBUG -> RESET -> STAND` | 中等，不发关节目标 |
| `nix_joint_cmd` | 进入 DEBUG 并发布关节命令 | 高，先用 Python 小目标验证 |
| `nix_data_collection` | 主动状态/RL/MIMIC 采集场景 | 高，不是 smoke test |
| `lud_send_robot_cmd` | LUD 状态/速度示例 | 仅 LUD |
| `lud_send_joint_cmds` | LUD 关节示例 | 仅 LUD |

构建后先看帮助：

```bash
./build/nix_lcm_sub --help
./build/nix_robot_state --help
./build/nix_debug_state --help
./build/nix_joint_cmd --help
./build/nix_data_collection --help
```

## 7. C++ API

### `SdkRobotManager`

```cpp
SdkRobotManager manager;
manager.Init();
```

### `SendRobotCmd`

```cpp
bool SendRobotCmd(
    SdkStateType state,
    float vx = 0,
    float vy = 0,
    float vyaw = 0,
    int8_t policy_type = 0);
```

常用状态：

| 状态 | 值 | 说明 |
| --- | ---: | --- |
| `RESET` | 1 | 复位 |
| `STAND` | 2 | 站立 |
| `RL_WALK` | 3 | RL 行走 |
| `DEBUG` | 10 | 关节调试状态 |
| `RL_NAV` | 11 | 导航策略 |
| `RL_WALK_AMP` | 12 | AMP 行走 |
| `BY_MIMIC` | 20 | mimic 策略 |
| `BFM_MIMIC` | 21 | mimic 策略 |

### `SendJointCmds`

```cpp
bool SendJointCmds(const std::vector<SdkJointCmd>& joint_cmds);
```

`SdkJointCmd` 字段：

| 字段 | 说明 |
| --- | --- |
| `component_type` | 组件类型，见 `SdkComponentType` |
| `joint_id` | 组件内局部关节编号 |
| `ctrlWord` | `3` 为常用 PD/MIT 控制；`200` 为复位语义 |
| `tarPos` | 目标位置，rad |
| `tarVel` | 目标速度 |
| `tarCur` | 预留/电流 |
| `tarTor` | 目标力矩 |
| `res1` | `kp` |
| `res2` | `kd` |
| `res3` | 预留 |
| `res4` | 预留 |

组件值：

| 组件 | 值 |
| --- | ---: |
| `ARM_L` | 1 |
| `ARM_R` | 2 |
| `WAIST` | 7 |
| `LEG_L` | 8 |
| `LEG_R` | 9 |

### 回调

```cpp
bool SetJointDataCb(JointDateCb cb);     // const joint_datasets_lcmt*
bool SetImuDataCb(ImuDateCb cb);         // const imu_data_lcmt*
bool SetRobotStatusCb(RobotStatusCb cb); // const robot_status_lcmt*
```

## 8. LCM 类型字段

### `robot_cmd_lcmt`

| 字段 | 说明 |
| --- | --- |
| `state` | 机器人状态 |
| `x` | x 方向速度 |
| `y` | y 方向速度 |
| `yaw` | yaw 速度 |
| `policy_type` | policy 选择 |

### `joint_cmd_lcmt`

| 字段 | 说明 |
| --- | --- |
| `component_type` | 组件类型 |
| `joint_id` | 组件内关节编号 |
| `ctrlWord` | 控制字 |
| `tarPos` | 目标位置 |
| `tarVel` | 目标速度 |
| `tarCur` | 目标电流/预留 |
| `tarTor` | 目标力矩 |
| `res1` | `kp` |
| `res2` | `kd` |
| `res3` | 预留 |
| `res4` | 预留 |

### `joint_cmds_lcmt`

| 字段 | 说明 |
| --- | --- |
| `cmds_num` | 命令数量 |
| `cmds` | `joint_cmd_lcmt[]` |

### `joint_data_lcmt`

| 字段 | 说明 |
| --- | --- |
| `component_type` | 组件类型 |
| `joint_id` | 组件内关节编号 |
| `stat` | 关节状态 |
| `pos_high` | 当前位置高位 |
| `num_cycles_high` | 高位圈数 |
| `pos_low` | 当前位置低位 |
| `num_cycles_low` | 低位圈数 |
| `vel` | 当前速度 |
| `cur` | 当前电流/预留 |
| `tor` | 当前力矩 |
| `res1..res4` | 预留 |

### `joint_datasets_lcmt`

| 字段 | 说明 |
| --- | --- |
| `datasets_num` | 反馈数量 |
| `datasets` | `joint_data_lcmt[]` |

### `imu_data_lcmt`

| 字段 | 说明 |
| --- | --- |
| `omega` | 陀螺仪角速度 `(rad/s)` |
| `acc` | 加速度 `(m/s^2)` |
| `temp` | 温度 |
| `good_packets` | 有效包计数 |
| `bad_packets` | 错误包计数 |
| `navQuat` | 导航四元数 `[w, x, y, z]` |
| `navOmega` | 导航角速度 |
| `navRPY` | 导航欧拉角 |

## 9. 已验证状态

最近真机测试结果：

- C++ 全部构建通过。
- C++ 所有示例 `--help` 通过。
- Python 全部脚本语法编译通过。
- Python 所有脚本 `--help` 通过。
- 真机通过 `nix_lcm_sub`，IMU 约 50Hz。
- 真机通过 `nix_robot_state` 默认 `RESET -> STAND`。
- 真机通过 `nix_debug_state`。
- 真机通过 `nix_debug_state.py enter/leave`。
- 真机通过 `nix_robot_state.py stand`。
- DEBUG 下 `nix_lcm_sub.py --once --print-limit 21` 收到 21 关节，`decode_errors=0`。
- `nix_joint_cmd.py` 通过最小 WAIST 保持命令。

未计入“实际运行通过”：

- `sim2real_lumos.py` / `sim2real_mimic.py` 策略动作回放。原因是需要 ONNX 运行依赖、模型文件和动作文件；当前只验证了入口和依赖提示。
