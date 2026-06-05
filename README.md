# Lumos SDK

### 依赖库
- glog、gflags: 日志输出控制
- lcm: SDK与机器人通讯协议


### 运行环境及配置
机器人的网口ip是 192.168.54.110，请确保SDK的运行设备与机器人处于同一个网段，确保可以ping通。

需要在SDK的运行设备上开启多播支持，以下一种配置方法供参考。
```
## 以下命令中的ethXXX，需要替换为SDK的运行设备上与机器人连接的网卡
# 打开多播
sudo ip link set ethXXX multicast on

# 添加多播地址转发
sudo ip route add 224.0.0.0/4 dev ethXXX
```

!!!注意，需要断开机器人与手柄之间的连接!!!
在手柄连接的状态下，机器人会屏蔽通过此sdk发送的指令。


### SdkRobotManager类接口及使用说明
#### 构造函数
SdkRobotManager();

无参数，以LUMOS_LCM_URL_PORT 初始化lcm类实例。

#### Init
void Init();

阻塞直到lcm实例的good方法返回成功。

#### SendRobotCmd
bool SendRobotCmd(SdkStateType state, float vx = 0, float vy = 0, float vyaw = 0);

高层运动控制接口，用于切换机器人状态，以及控制机器人整机移动。

在行走状态下，支持前后移动、左右移动、左右旋转。

参数说明：
- SdkStateType
  -  要切换到的目的状态
  -  当前支持的状态：RESET、STAND、RL_WALK、RL_MIMIC、RL_NAV
- vx, vy, vyaw
  -  仅在RL_WALK 和RL_NAV 状态下设置才有效
  -  vx: 机器人前进方向移动速度，单位m/s，范围[-0.5, 0.5]，其中[-0.2, 0.2]是无效区间
  -  vy: 机器人侧向移动速度，单位m/s，范围[-0.3, 0.3]，其中[-0.2, 0.2]是无效区间
  -  vyaw: 机身角速度，单位rad/s，范围[-0.5, 0.5]，其中[-0.1, 0.1]是无效区间


#### SendModeCmd
bool SendModeCmd(int mode);

设置关节运动的控制模式，可选RL控制和SDK控制。
如果需要通过SDK控制关节运动或接收关节数据，请将模式设置为SDK控制。

参数说明：
- mode
  - 0: RL控制模式，关节运动由机器人自身的运控程序控制
  - 1: SDK控制模式，关节运动由SDK发送指令控制


#### SendJointCmds
bool SendJointCmds(const std::vector<SdkJointCmd>& joint_cmds);

底层运动控制接口，用于控制机器人的关节移动。

一个SdkJointCmd结构表示一次对某个关节的控制指令，支持以vector的形式一次控制多个关节。

SdkJointCmd结构字段说明：
- component_type
  - 表示要控制的组件类型，比如左手、右脚等
  - 请使用SdkComponentType枚举类型给其赋值
- joint_id
  - 表示要控制的组件的关节编号，这是对应组件的局部关节编号，每个组件的关节都从0开始编号
  - 注意不要超过对应组件的关节数量
- ctrlWord: 正常控制 3，复位 200
- tarPos: 目标位置，单位rad
- tarVel: 
- tarCur: 
- tarTor: 
- res1: 
- res2: 
- res3: 
- res4: 

注意，控制关节运动的一般的调用流程：
- 调用SendRobotCmd，设置机器人进入RESET状态，再进入STAND状态
- 调用SendModeCmd，设置关节控制模式为SDK控制模式
- 调用SendJointCmds，控制关节运动


#### SetJointDataCb
bool SetJointDataCb(JointDateCb cb);

设置关节数据的回调函数，当订阅的关节数据到达时会自动调用。

回调函数原型：
    using JointDateCb = void (*)(const sdk_lcmt_joint_datasets*);

sdk_lcmt_joint_datasets结构中包含了所有的关节数据，单个关节数据用sdk_lcmt_joint_data表示。

sdk_lcmt_joint_data结构字段说明：
- component_type: 同SdkJointCmd结构中的相同字段
- joint_id: 同SdkJointCmd结构中的相同字段
- stat: 
- pos_high: 
- num_cycles_high: 
- pos_low: 
- num_cycles_low: 
- vel: 
- cur: 
- tor: 
- res1: 
- res2: 
- res3: 
- res4: 

### NIX Python LCM 反馈采集

`python/nix_lcm_sub.py` 用于订阅 NIX 控制器发布的 `JointsData`，解码
`sdk_lcmt_joint_datasets`，并可选写成 `lumos_pipeline` 可读取的 feedback CSV。
原有 `python/lcm_sub.py` 保持不变。

本机 mock 环回测试：

```bash
# 终端 A
python3 lumos_sdk/python/nix_lcm_sub.py --local --once

# 终端 B
python3 lumos_sdk/python/nix_lcm_pub_mock.py --local --count 5
```

真机监听前，先配置连接机器人的网卡多播路由：

```bash
cd lumos_sdk
source config_network_lcm.sh ethXXX
cd ..
```

确认真机每帧 21 个关节：

```bash
python3 lumos_sdk/python/nix_lcm_sub.py --duration 2 --print-limit 21
```

采集 10 秒真机 feedback CSV：

```bash
python3 lumos_sdk/python/nix_lcm_sub.py \
  --duration 10 \
  --print-every 500 \
  --print-limit 21 \
  --csv build/nix_feedback_hw_v2.csv
```

采集后检查：

```bash
wc -l build/nix_feedback_hw_v2.csv
head build/nix_feedback_hw_v2.csv
tail build/nix_feedback_hw_v2.csv
```

生成的 CSV 第一行会写入 feedback schema 版本：

```text
# feedback_schema=v1.0
```

版本来源规则：

- 默认值是特殊值 `v0.0`
- 运行时优先读取 `../lumos_pipeline/src/lumos_pipeline/schemas/feedback_v1.json` 中的 `version`
- 如果读取失败、JSON 解析失败或没有 `version` 字段，则保留默认值 `v0.0`

验证当前能解析到的 schema 版本：

```bash
cd lumos_sdk
python3 - <<'PY'
import sys
sys.path.insert(0, "python")
import nix_lcm_sub as s

print("schema path:", s.feedback_schema_json_path())
print("resolved version:", s.resolve_feedback_schema_version())
PY
```

注意：最后一行 `PY` 必须顶格写，前面不能有空格；否则 shell 会进入
`heredoc>` 续行等待。

验证生成的 CSV 带版本头并通过 pipeline schema 校验：

```bash
PYTHONPATH=lumos_pipeline/src:lumos_diagnostics/src \
python -m lumos_pipeline.cli verify-schema \
  build/nix_feedback_hw_v2.csv --robot-model nix --require-header
```

正常结果应满足：

- `decode_errors=0`
- 每帧 `samples=21`
- CSV 中 `JointID` 为全局唯一键，例如 `1:0`、`8:0`、`9:5`、`7:0`
- `MotorCurrent` 当前可能全 0，此字段在 NIX 真机样本中暂未验证

### NIX Python 关节指令下发

`python/nix_joint_cmd.py` 用于替代 `example/nix_cmd_all_joints.cpp` 中依赖宏切换的常用关节指令测试。它通过运行时参数发布 `sdk_lcmt_joint_cmds`，不需要重新编译 C++ 示例。

边界：

- 这个脚本只发关节指令，不负责进入或退出 SDK 模式。
- 先用 `python/sdk_debug.py mode 1` 或其它已验证流程进入 SDK 模式。
- 第一次使用先加 `--dry-run`，确认 component、joint 和目标值后再发布。

查看组件编号和 NIX2 全局关节索引：

```bash
python3 lumos_sdk/python/nix_joint_cmd.py list
```

单关节 dry-run：

```bash
python3 lumos_sdk/python/nix_joint_cmd.py single \
  --component WAIST \
  --joint-id 0 \
  --pos 0.30 \
  --kp 160 \
  --kd 6 \
  --dry-run
```

发布一次腰关节目标：

```bash
python3 lumos_sdk/python/nix_joint_cmd.py single \
  --component WAIST \
  --joint-id 0 \
  --pos 0.30 \
  --kp 160 \
  --kd 6
```

按 NIX2 全局关节索引发布，索引顺序为 `LEG_L(0-5)`、`LEG_R(6-11)`、`WAIST(12)`、`ARM_L(13-16)`、`ARM_R(17-20)`：

```bash
python3 lumos_sdk/python/nix_joint_cmd.py global \
  --index 12 \
  --pos 0.30 \
  --kp 160 \
  --kd 6 \
  --duration 2 \
  --rate-hz 100
```

批量下发多个目标，同一个 `--target` 格式为 `COMPONENT:JOINT_ID:POS[:KP[:KD[:VEL[:TOR[:CTRL_WORD]]]]]`：

```bash
python3 lumos_sdk/python/nix_joint_cmd.py batch \
  --target LEG_L:3:0.20:160:6 \
  --target WAIST:0:0.30:160:6 \
  --duration 1
```

从 CSV 下发目标，CSV 至少包含 `component,joint_id,pos`，可选列为 `kp,kd,vel,tor,cur,ctrl_word`：

```csv
component,joint_id,pos,kp,kd
WAIST,0,0.30,160,6
LEG_L,3,0.20,160,6
```

```bash
python3 lumos_sdk/python/nix_joint_cmd.py file \
  --path build/nix_joint_targets.csv \
  --duration 1 \
  --rate-hz 100
```

回放模型目录中的舞蹈/参考动作。该命令读取 `store_ref_motion.txt` 和 `kp_kd.yaml`，按 `nix_cmd_all_joints.cpp` 的 replay 映射重排为 SDK 21 关节顺序：

```bash
python3 lumos_sdk/python/nix_joint_cmd.py replay \
  --model-dir lumos_sdk/models/nix2_policy/sanlin_04101426 \
  --loops 1 \
  --dry-run
```

确认首帧/末帧和增益后，去掉 `--dry-run` 发布：

```bash
python3 lumos_sdk/python/nix_joint_cmd.py replay \
  --model-dir lumos_sdk/models/nix2_policy/sanlin_04101426 \
  --loops 1 \
  --rate-hz 100
```

`--loops 0` 与旧 C++ 示例一致，表示持续循环直到 Ctrl-C：

```bash
python3 lumos_sdk/python/nix_joint_cmd.py replay \
  --model-dir lumos_sdk/models/nix2_policy/sanlin_04101426 \
  --loops 0 \
  --rate-hz 100
```

调试时可只加载前几帧：

```bash
python3 lumos_sdk/python/nix_joint_cmd.py replay \
  --model-dir lumos_sdk/models/nix2_policy/sanlin_04101426 \
  --max-frames 20 \
  --dry-run
```

### NIX Python 状态切换与等待确认

`python/nix_robot_state.py` 用于发送机器人高层状态命令，并等待 `lcm_robot_status` 确认。它只管 `RESET` / `STAND` / `RL_*` 这类状态，不发布关节级 PD 指令。

推荐在真实采集前使用它替代手动执行 `sdk_debug.py state 1`、`sdk_debug.py state 2` 和人工观察 `listen`。

只预览流程，不发布 LCM：

```bash
python3 lumos_sdk/python/nix_robot_state.py stand --dry-run
```

执行 `RESET -> STAND`，等待确认后进入 SDK 模式：

```bash
python3 lumos_sdk/python/nix_robot_state.py stand \
  --timeout 15 \
  --stand-settle 10
```

成功时应看到：

```text
已发送：state RESET(1)
已确认：state=RESET(1)
已发送：state STAND(2)
已确认：state=STAND(2)
已发送：mode SDK(1)
```

只发送并等待单个状态：

```bash
python3 lumos_sdk/python/nix_robot_state.py state STAND \
  --wait \
  --timeout 15
```

监听机器人状态：

```bash
python3 lumos_sdk/python/nix_robot_state.py listen --duration 10
```

采集结束后切回 RL controller type：

```bash
python3 lumos_sdk/python/nix_robot_state.py mode RL
```


#### SetImuDataCb
bool SetImuDataCb(ImuDateCb cb);

设置IMU数据的回调函数，当订阅的IMU数据到达时会自动调用。

回调函数原型：
    using ImuDateCb = void (*)(const microstrain_lcmt*);

microstrain_lcmt结构字段说明：
- omega: 陀螺仪角速度 (rad/s) - [x, y, z]
- acc: 加速度计数据 (m/s²) - [x, y, z]
- temp: 温度传感器读数 (°C)
- good_packets: 有效数据包计数
- bad_packets: 错误数据包计数
- navQuat: 导航四元数 [w, x, y, z]
- navOmega: 导航解算角速度 (rad/s)
- navRPY: 导航欧拉角 [roll, pitch, yaw] (rad)


### 完整示例参考
- ![高层运动控制，控制机器人整机移动](./example/send_robot_cmd.cpp)
- ![底层运动控制，控制机器人的关节移动 - 腿部运动](./example/send_joint_cmds.cpp)
- ![底层运动控制，控制机器人的关节移动 - 全面的关节运动测试](./example/send_joint_cmds2.cpp)
- ![接收并处理关节数据](./example/recv_joint_datasets.cpp)
- ![接收并处理IMU数据](./example/recv_imu_data.cpp)

#### 示例编译
```bash
cmake -S . -B build
cd build
make
```

#### 示例运行
执行上述编译后，会在build目录下生成对应的可执行文件。

每个示例源文件都对应一个可执行文件，在build目录下直接运行即可。

!!!注意!!!
涉及运动控制相关的示例程序，运行前请务必确保机器人处于安全的可用环境，避免摔倒和发生意外！
