# Lumos SDK

### 依赖库
- glog、gflags: 日志输出控制
- lcm: SDK与机器人通讯协议

### 编译
```bash
cmake -S . -B build
cd build
make
```

### 运行环境及配置
确保SDK的运行环境与机器人处于同一个网段（192.168.54.XXX）
lcm通过LUMOS_LCM_URL_PORT配置，位于sdk_base_define.hpp


### SdkRobotManager类接口及使用说明
#### 构造函数
SdkRobotManager();

无参数，以LUMOS_LCM_URL_PORT 初始化lcm类实例

#### Init
void Init();

阻塞直到lcm实例的good方法返回成功

#### SendRobotCmd
bool SendRobotCmd(SdkStateType state, float x = 0, float y = 0, float yaw = 0);

用于切换机器人状态，以及控制对应状态下的运行参数。
参数说明：
- SdkStateType
  - 当前支持切换的状态：RESET、STAND、RL_WALK、RL_MIMIC、RL_NAV
- x, y, yaw
  -  仅RL_WALK 和RL_NAV 状态下才有效
  -  x: 机器人前进方向移动速度
  -  y: 机器人侧向移动速度
  -  yaw: 机身角速度


#### SendJointCmds
bool SendJointCmds(const std::vector<SdkJointCmd>& joint_cmds);

一个SdkJointCmd结构表示一次关节控制指令，支持以vector的形式一次控制多个关节。
SdkJointCmd结构字段说明：
- component_type
  - 表示要控制的组件类型，比如左手、右脚等
  - 请使用SdkComponentType枚举类型给其赋值
- joint_id
  - 表示要控制的组件的关节编号，这是对应组件的局部关节编号，每个组件的关节都从0开始编号
  - 注意不要超过对应组件的关节数量
- 其他
  - 参考ecat通讯协议


#### 使用示例
```
    // 构造并初始化
    SdkRobotManager manager;
    manager.Init();

    ...

    // 发送机器人切换状态及控制命令
    manager.SendRobotCmd(...);

    ...

    // 发送关节控制命令
    manager.SendJointCmds(...);
```

### 完整例子参考
- ![example for robot state and movement control](./example_robot_cmd.cpp)
- ![example for low level joints control](./example_robot_cmd.cpp)