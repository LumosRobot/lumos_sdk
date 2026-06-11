# SDK

## 1. 依赖库
- glog、gflags: 日志输出控制
- lcm: SDK与机器人通讯协议


## 2. 运行环境及配置
机器人的网口ip是 192.168.54.110，请确保SDK的运行设备与机器人处于同一个网段，确保可以ping通。

需要在SDK的运行设备上开启多播支持，以下一种配置方法供参考。
```
## 以下命令中的ethXXX，需要替换为SDK的运行设备上与机器人连接的网卡
# 打开多播
sudo ip link set ethXXX multicast on

# 添加多播地址转发
sudo ip route add 224.0.0.0/4 dev ethXXX
```

## 3. 示例编译
```bash
cmake -S . -B build
cd build
make
```

### 示例运行
执行上述编译后，会在build目录下生成对应的可执行文件。

每个示例源文件都对应一个可执行文件，在build目录下直接运行即可。

!!!注意!!!
涉及运动控制相关的示例程序，运行前请务必确保机器人处于安全的可用环境，避免摔倒和发生意外！
### 完整示例
- [Lud1示例](./example/lud1_send_joint_cmds.cpp)
- [接收手柄数据](./example/recv_game_handler_cmd.cpp)
- [接收imu数据](./example/recv_imu_data.cpp)
- [接收关节数据](./example/recv_joint_datasets.cpp)

## 4. sdk_robot_manager 类接口及使用说明
SdkRobotManager 是机器人 SDK 的通信管理类，负责通过 LCM与机器人控制系统进行通信。
``` 
# LCM通信地址：
udpm://239.255.76.67:7667?ttl=255
```
- 机器人运动控制 SendRobotCmd
``` C++
// state：机器人状态  
// vx: x方向速度     范围: [-0.5, 0.5]
// vy: y方向速度     范围: [-0.3, 0.3]
// vyaw: 偏航角速度  范围: [-0.5, 0.5]
bool SendRobotCmd(SdkStateType state, float vx, float vy, float vyaw);
``` 
- 关节控制 SendJointCmds
``` C++
// joint_cmds: 关节控制命令
bool SendJointCmds(const std::vector<SdkJointCmd>& joint_cmds);
```
SdkJointCmd结构字段说明参考第5节的SdkJointCmd参数说明

- 控制模式 SendModeCmd

设置关节运动的控制模式，可选RL控制和SDK控制。
如果需要通过SDK控制关节运动或接收关节数据，请将模式设置为SDK控制。
``` C++
// mode:
//   0: RL控制模式，关节运动由机器人自身的运控程序控制
//   1: SDK控制模式，关节运动由SDK发送指令控制
bool SendModeCmd(int mode);
```
---

## 5. 参数说明
### 5.1 sdk_base_define
#### SdkJointCmd
| Field | Description | Notes |
|------|-------------|------|
| component_type | 表示要控制的组件类型，例如左臂、右腿等 | 请使用 `SdkComponentType` 枚举类型赋值，组件列表见 **SdkComponentType 表** |
| joint_id | 表示 `component_type` 组件的关节编号 | 为组件的 **局部关节编号**，每个组件均从 `0` 开始编号，注意不要超过该组件的关节数量 |
| ctrlWord | 控制模式 | `3`：正常发送目标位置进行控制 <br>`200`：复位 |
| tarPos | 目标位置 | 单位：`rad` |
| tarVel | 目标速度 |  |
| tarCur | 预留字段 | 目前未使用 |
| tarTor | 目标力矩 | 单位通常为 `Nm` |
| res1 | 控制参数 `kp` | 位置控制比例系数 |
| res2 | 控制参数 `kd` | 速度控制微分系数 |
| res3 | 预留字段 | 目前未使用 |
| res4 | 温度信息 | 电机温度反馈 |
---
#### SdkComponentType

| Component | LUS2 (Humanoid) | Nix / Pix (Humanoid) | LUD1 (Quadruped) |
|-----------|-----------------|----------------------|------------------|
| HEAD      |  |  |  |
| ARM_L     | ✓ | ✓ | ✓ |
| ARM_R     | ✓ | ✓ | ✓ |
| WAIST     | ✓ | ✓ |  |
| LEG_L     | ✓ | ✓ | ✓ |
| LEG_R     | ✓ | ✓ | ✓ |
---
#### SdkStateType：要切换到的目的状态

| State        | LUS2 (Humanoid) | Nix / Pix (Humanoid) | LUD1 (Quadruped) |
|--------------|----|-----------|------|
| NOT_A_STATE  |  ✓ | ✓ | ✓ |
| RESET        |  ✓ | ✓ | ✓ |
| STAND        |  ✓ | ✓ | ✓ |
| RL_WALK      |  ✓ | ✓ | ✓ |
| RL_WALK_AMP  |  ✓ | ✓ | ✓ |
| RL_NAV       |  ✓ | ✓ | --|
| RL_SITUP     |  ✓ | ✓ | --|
| RL_LIEDOWN   |  ✓ | ✓ | ✓ |
| RL_MIMIC     |  ✓ | ✓ | --| 

---

### 5.2 LCM type

#### lumos_lcm_control

| 参数 | 说明  |
| ----- | ------------ |
| state | 控制状态         |
| x     | 目标 x 方向速度或位置 |
| y     | 目标 y 方向速度或位置 |
| yaw   | 目标偏航角        |

#### microstrain_lcmt 传感器数据

| 参数          | 说明                          |
| ------------ | ---------------------------------- |
| omega        | 陀螺仪角速度 `(rad/s)` `[x, y, z]`       |
| acc          | 加速度计数据 `(m/s²)` `[x, y, z]`        |
| temp         | 温度传感器读数 `(°C)`                     |
| good_packets | 有效数据包计数                            |
| bad_packets  | 错误数据包计数                            |
| navQuat      | 导航四元数 `[w, x, y, z]`               |
| navOmega     | 导航解算角速度 `(rad/s)`                  |
| navRPY       | 导航欧拉角 `[roll, pitch, yaw]` `(rad)` |

####  sdk_lcmt_joint_cmd 单个关节控制命令

| 参数          | 说明                          |
| -------------- | -------------------------------- |
| component_type | 控制的组件类型（见 `SdkComponentType` 枚举） |
| joint_id       | 组件内部关节编号                         |
| ctrlWord       | 控制模式                             |
| tarPos         | 目标位置                             |
| tarVel         | 目标速度                             |
| tarCur         | 预留                             |
| tarTor         | 目标力矩                             |
| res1           | `kp`                             |
| res2           | `kd`                             |
| res3           | 预留                               |
| res4           | 温度                               |

#### sdk_lcmt_joint_cmds 多关节控制命令集合
| 参数          | 说明                          |
| -------- | --------------------------------- |
| cmds_num | 控制命令数量                        |
| cmds     | `std::vector<sdk_lcmt_joint_cmd>` |


#### sdk_lcmt_joint_data 单个关节状态数据：
| 参数          | 说明                          |
| --------------- | ----------- |
| component_type  | 组件类型        |
| joint_id        | 组件内部关节编号    |
| stat            | 关节状态        |
| pos_high        | 当前位置（高位）    |
| num_cycles_high | 高位循环计数      |
| pos_low         | 当前位置（低位）    |
| num_cycles_low  | 低位循环计数      |
| vel             | 当前速度        |
| cur             | 预留        |
| tor             | 当前力矩        |
| res1            | 预留          |
| res2            | 预留          |
| res3            | 预留          |
| res4            | 预留          |

#### sdk_lcmt_joint_datasets 多关节状态数据集合
| 参数          | 说明                          |
| ------------ | ---------------------------------- |
| datasets_num | 数据数量                               |
| datasets     | `std::vector<sdk_lcmt_joint_data>` |

#### sdk_lcmt_type
| 参数          | 说明                          |
| --------------- | ----------- |
| controller_type | 当前控制类型     |


