# Lumos SDK — High-Level 使用指南

本文档介绍通过 `SdkRobotManager` 高层接口控制机器人的方法：状态切换、整机速度控制、以及数据回调订阅。关节级控制请参考 `docs/user_guide.md`。

## 1. 依赖

- glog、gflags：日志输出
- lcm：SDK 与机器人通讯协议
- C++20

## 2. 网络配置

机器人网口 IP：`192.168.54.110`，确保 SDK 运行设备与机器人同网段、能 ping 通。

需要开启多播支持（将 `ethXXX` 替换为与机器人连接的网卡）：

```bash
sudo ip link set ethXXX multicast on
sudo ip route add 224.0.0.0/4 dev ethXXX
```

也可使用仓库根目录的脚本：`bash config_network_lcm.sh ethXXX`

## 3. 编译

```bash
cmake -S . -B build
cd build
make -j$(nproc)
```

编译产物在 `build/` 目录下，每个 `example/*.cpp` 对应一个可执行文件。

## 4. SdkRobotManager 高层 API

### 4.1 构造与初始化

```cpp
#include "sdk_robot_manager.hpp"

SdkRobotManager manager;
manager.Init();   // 阻塞直到 LCM 就绪，然后启动后台接收线程
```

### 4.2 SendRobotCmd — 状态切换与整机速度控制

```cpp
bool SendRobotCmd(SdkStateType state, float vx = 0, float vy = 0, float vyaw = 0, int8_t policy_type = 0);
```

切换到目标状态，并在行走/导航状态下控制整机移动速度。

**参数说明：**

| 参数 | 说明 |
|------|------|
| `state` | 目标状态，见 [SdkStateType 枚举](#51-sdkstatetype) |
| `vx` | 前进速度 (m/s)，范围 `[-0.5, 0.5]`。仅在 `RL_WALK` / `RL_WALK_AMP` / `RL_NAV` 下有效 |
| `vy` | 侧向速度 (m/s)，范围 `[-0.3, 0.3]`。仅在 `RL_WALK` / `RL_WALK_AMP` / `RL_NAV` 下有效 |
| `vyaw` | 偏航角速度 (rad/s)，范围 `[-0.5, 0.5]`。仅在 `RL_WALK` / `RL_WALK_AMP` / `RL_NAV` 下有效 |
| `policy_type` | 策略类型，用于 `BY_MIMIC` / `BFM_MIMIC` 选择舞蹈编号 |

速度有死区：过小的速度值机器人不会响应。

### 4.3 SendModeCmd — 控制模式切换

```cpp
bool SendModeCmd(int mode);
```

| mode | 含义 |
|------|------|
| `0` | RL 控制模式，关节运动由机器人自身运控程序控制 |
| `1` | SDK 控制模式，关节运动由 SDK 指令控制，同时开始发布关节数据和机器人状态 |

**注意**：使用 `SendRobotCmd` 切换状态前，通常需要先进入 SDK 模式（`SendModeCmd(1)`），确保机器人的状态机由 SDK 驱动。

### 4.4 数据回调注册

SdkRobotManager 在 `Init()` 后启动后台 LCM 接收线程，回调在后台线程中触发，用户无需手动调用 `handleTimeout`。

```cpp
// 机器人状态回调（状态切换确认、音频文件等）
bool SetRobotStatusCb(RobotStatusCb cb);
// 回调签名: void (*)(const robot_status_lcmt*)
// robot_status_lcmt 字段: state, type, audio_file

// 关节数据回调（21 个关节的位置、速度、力矩）
bool SetJointDataCb(JointDateCb cb);
// 回调签名: void (*)(const sdk_lcmt_joint_datasets*)

// IMU 数据回调（角速度、加速度、四元数、欧拉角）
bool SetImuDataCb(ImuDateCb cb);
// 回调签名: void (*)(const microstrain_lcmt*)

// 手柄命令回显（订阅 lcm_robot_cmd 的回显，用于诊断）
bool SetGameHandlerCmdCb(GameHandlerCmdCb cb);
// 回调签名: void (*)(const robot_cmd_lcmt*)
```

**注意**：所有回调在 LCM 后台线程中执行，多线程访问共享数据需加锁。

### 4.5 HandleLcmTimeout — 手动驱动 LCM（可选）

```cpp
int HandleLcmTimeout(int timeout_ms = 0);
```

如果你的程序需要自己控制事件循环（不需要后台线程），可以不调 `Init()`，在自己的循环中定期调用此方法手动分发 LCM 消息。

## 5. 枚举定义

### 5.1 SdkStateType

| 枚举值 | 数值 | 适用平台 | 说明 |
|--------|------|----------|------|
| `RESET` | 1 | LUD1 / NIX2 | 复位/趴下 |
| `STAND` | 2 | LUD1 / NIX2 | 站立 |
| `RL_WALK` | 3 | LUD1 / NIX2 | 强化学习行走 |
| `RL_LIEDOWN` | 5 | LUD1 / NIX2 | 趴下 |
| `RL_MIMIC` | 6 | NIX2 | 模仿学习动作 |
| `RL_NAV` | 11 | NIX2 | 导航模式 |
| `RL_WALK_AMP` | 12 | LUD1 / NIX2 | 周期性行走（AMP） |
| `BY_MIMIC` | 20 | NIX2 | 舞蹈模仿（通过 `policy_type` 选舞） |
| `BFM_MIMIC` | 21 | NIX2 | BFM 模仿 |

### 5.2 状态切换典型路径

```
NOT_A_STATE (0) ──→ RESET (1) ──→ STAND (2) ──→ RL_WALK (3) / RL_WALK_AMP (12) / RL_NAV (11)
                                                    │
                                                    ▼
                                              RESET (1) ──→ STAND (2) ──→ RL_MIMIC (6) / BY_MIMIC (20)
```

从行走状态退出时先回到 RESET，从 RESET 退出时回到 STAND。RL_LIEDOWN 可以在任意状态下触发（紧急趴下）。

## 6. 典型工作流程

```cpp
#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <glog/logging.h>

static volatile bool g_running = true;
static std::atomic<int> g_robot_state{0};

static void sigint_handler(int) { g_running = false; }

static void on_robot_status(const robot_status_lcmt* msg) {
    g_robot_state = msg->state;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, sigint_handler);
    google::InitGoogleLogging(argv[0]);

    // 1. 初始化
    SdkRobotManager manager;
    manager.Init();
    manager.SetRobotStatusCb(on_robot_status);  // 订阅状态变化

    // 2. 进入 SDK 控制模式
    manager.SendModeCmd(1);
    sleep(1);

    // 3. RESET → STAND
    manager.SendRobotCmd(SdkStateType::RESET);
    sleep(10);  // 等待复位完成
    manager.SendRobotCmd(SdkStateType::STAND);
    sleep(20);  // 等待站立完成

    // 4. 进入行走状态，控制移动
    manager.SendRobotCmd(SdkStateType::RL_WALK_AMP);
    sleep(5);

    // 前进 0.1 m/s
    manager.SendRobotCmd(SdkStateType::RL_WALK_AMP, 0.1f);
    sleep(3);

    // 原地转弯
    manager.SendRobotCmd(SdkStateType::RL_WALK_AMP, 0.0f, 0.0f, 0.2f);
    sleep(3);

    // 停止
    manager.SendRobotCmd(SdkStateType::RL_WALK_AMP, 0.0f, 0.0f, 0.0f);
    sleep(5);

    // 5. 退出：回 RESET，退出 SDK 模式
    manager.SendRobotCmd(SdkStateType::RL_LIEDOWN);

    google::ShutdownGoogleLogging();
    return 0;
}
```

## 7. LCM 通道参考

| 通道 | 方向 | 类型 | 用途 |
|------|------|------|------|
| `lcm_robot_cmd` | 发布 | `robot_cmd_lcmt` | `SendRobotCmd` 发送 |
| `lcm_robot_cmd_echo` | 订阅 | `robot_cmd_lcmt` | 命令回显（诊断用） |
| `lcm_robot_status` | 订阅 | `robot_status_lcmt` | 机器人状态反馈 |
| `sdk_lcm_set_type_cmd` | 发布 | `sdk_lcmt_type` | `SendModeCmd` 发送 |
| `JointsData` | 订阅 | `sdk_lcmt_joint_datasets` | 关节状态数据 |
| `myIMU` | 订阅 | `microstrain_lcmt` | IMU 传感器数据 |

LCM 多播地址：`udpm://239.255.76.67:7667?ttl=255`

## 8. LCM 数据类型参考

### robot_cmd_lcmt

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | `int8_t` | 目标状态 |
| `x` | `float` | vx (m/s) |
| `y` | `float` | vy (m/s) |
| `yaw` | `float` | vyaw (rad/s) |
| `policy_type` | `int8_t` | 策略类型 |

### robot_status_lcmt

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | `int8_t` | 当前机器人状态 |
| `type` | `int8_t` | 机器人类型 |
| `audio_file` | `string` | 音频文件名 |

### microstrain_lcmt (IMU)

| 字段 | 类型 | 说明 |
|------|------|------|
| `omega` | `float[3]` | 陀螺仪角速度 (rad/s) |
| `acc` | `float[3]` | 加速度 (m/s²) |
| `temp` | `float` | 温度 (°C) |
| `good_packets` | `int64_t` | 有效包计数 |
| `bad_packets` | `int64_t` | 错误包计数 |
| `navQuat` | `float[4]` | 导航四元数 [w, x, y, z] |
| `navOmega` | `float[3]` | 导航角速度 (rad/s) |
| `navRPY` | `float[3]` | 欧拉角 [roll, pitch, yaw] (rad) |

### sdk_lcmt_joint_datasets

| 字段 | 类型 | 说明 |
|------|------|------|
| `datasets_num` | `int16_t` | 关节数据条数 |
| `datasets` | `vector<sdk_lcmt_joint_data>` | 各关节数据 |

`sdk_lcmt_joint_data` 字段：`component_type`, `joint_id`, `stat`, `pos_high`, `pos_low`, `vel`, `cur`, `tor` 等。

## 9. 安全注意事项

- **运行前确保机器人处于安全环境**，周围无障碍物，避免摔倒和意外。
- **断开手柄连接**：手柄连接状态下，机器人会屏蔽 SDK 发送的指令。
- 在 SDK 控制模式下，如果程序崩溃或被 SIGINT 中断，务必发送 `RESET` 或 `RL_LIEDOWN` 使机器人回到安全状态，并调用 `SendModeCmd(0)` 退出 SDK 模式。
- 建议注册 SIGINT 处理函数做安全退出，参考示例 `nix_test_highlevel.cpp`。

## 10. 示例程序

| 示例 | 说明 |
|------|------|
| `nix_test_highlevel` | NIX2 高层控制测试：SDK模式 → RESET → STAND → RL_WALK（速度控制）→ RESET，含数据率统计 |
| `nix_test_sdk_mode` | NIX2 SDK 模式完整性测试：含所有回调订阅、状态确认超时、SIGINT 安全退出 |
| `lud_send_robot_cmd` | LUD1 高层控制：RESET → STAND → RL_WALK_AMP（前后移动、转弯）→ RL_LIEDOWN |
| `nix_recv_all_data` | 纯数据记录（不发控制指令），安全可与其他程序同时运行，输出 CSV |
| `recv_all_data` | 主动数据记录：进入 SDK 模式 → RESET → STAND，同时记录关节/IMU/状态数据到 CSV |
