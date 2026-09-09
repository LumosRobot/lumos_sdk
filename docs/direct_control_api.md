# Direct Control HTTP API 参考

机器人直连控制与数据接口。第三方开发者可通过连接机器人热点，访问 `http://192.168.1.1:8080` 控制机器人、读取遥测、获取视频流、上传并播放 MP3 音频。

> 数据流向：HTTP 请求 → `reporter_agent.direct_control` 解析 → 发布 LCM 消息 → `lumos_controller`（运动/手臂）或 `lumos_voice`（语音/音频播放）消费。

---

## 基础信息

- **机器人热点**：Wi-Fi: `nix_NIX005`；pwd: `nix_NIX005_pd`
- **Base URL**: `http://192.168.1.1:8080`（连接机器人热点后）
- **协议**: HTTP/JSON（视频为 MJPEG 流；音频上传为 `multipart/form-data`）
- **鉴权**: 无（局域网直连，端口见 `config.yml` 的 `direct_control.port`）
- **默认端口 / 视频 / 音频目录**（`config.yml` → `direct_control`）：

```yaml
direct_control:
  enabled: true
  port: 8080
  camera_index: 0
  web_dir: "/devel/lumos_ws/lumos_app/web"
  audio_dir: "/devel/lumos_ws/lumos_audio"
```

---

## 一、运动 / 状态控制

### `POST /api/cmd` — 机器人运动 / 状态切换

发布 `lcm_robot_cmd`（`robot_cmd_lcmt`）。

| 字段 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `state` | int8 | 0 | 目标状态码，见 [状态码表](#状态码-state) |
| `x` | float | 0.0 | X 方向速度（m/s） |
| `y` | float | 0.0 | Y 方向速度（m/s） |
| `yaw` | float | 0.0 | 偏航角速度（rad/s） |
| `policy_type` | int8 | 0 | 策略选择，见 [策略表](#策略-policy_type) |

> **重要语义**：`x/y/yaw` 全为 0 时执行**状态切换**；任一非 0 时作为**速度指令**（不改变状态）。`policy_type` 会覆盖手柄的策略选择（如 MIMIC 状态下的舞蹈索引）。

```bash
# 切换为站立
curl -X POST http://192.168.1.1:8080/api/cmd \
  -H "Content-Type: application/json" -d '{"state": 2}'

# 前进速度 0.5 m/s（state 忽略，不切状态）
curl -X POST http://192.168.1.1:8080/api/cmd \
  -H "Content-Type: application/json" -d '{"state": 3, "x": 0.5}'
```

响应：`{"status": "ok"}`

### `POST /api/arm_cmd` — 机械臂动作

发布 `lcm_arm_cmd`（`arm_cmd_lcmt`）。

| 字段 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `armstate` | int8 | 0 | 手臂动作码，见 [手臂动作表](#手臂-armstate) |

```bash
curl -X POST http://192.168.1.1:8080/api/arm_cmd \
  -H "Content-Type: application/json" -d '{"armstate": 9}'
```

响应：`{"status": "ok"}`

### `POST /api/goal_location` — 目标点导航(暂不支持)

发布 `lcm_goal_location`（`goal_location_lcmt`）。

| 字段 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `x` | float | 0.0 | 目标 X |
| `y` | float | 0.0 | 目标 Y |
| `yaw` | float | 0.0 | 目标偏航 |
| `duration` | float | 0.0 | 运动时长（秒） |
| `execute` | int8 | 1 | 是否立即执行（1=执行，0=仅设置） |

```bash
curl -X POST http://192.168.1.1:8080/api/goal_location \
  -H "Content-Type: application/json" \
  -d '{"x": 1.0, "y": 0.0, "yaw": 0.0, "duration": 5.0, "execute": 1}'
```

### `POST /api/led` — LED 灯效控制

发布 `lcm_led_control`（`led_control_lcmt`），由 `lumos_controller` 的 `LedHandler` 消费（串口下发）。

| 字段 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `control_word` | int16 | 0 | 灯效控制字，见 [LED 控制字表](#led-控制字-control_word) |
| `classes` | string | "" | 类别标签（记录用，不影响灯效） |
| `description` | string | "" | 描述（记录用） |

```bash
# 蓝灯常亮
curl -X POST http://192.168.1.1:8080/api/led \
  -H "Content-Type: application/json" -d '{"control_word": 1}'

# 红灯闪烁
curl -X POST http://192.168.1.1:8080/api/led \
  -H "Content-Type: application/json" -d '{"control_word": 6}'
```

响应：`{"status": "ok"}`

> `classes` 与 `description` 仅作记录，`LedHandler` 只读取 `control_word`；相同 `control_word` 连续下发会被去重忽略（避免重复串口指令）。

### `POST /api/voice_cmd` — 语音 / 状态播报

发布 `lcm_robot_status`（`robot_status_lcmt`），由 `lumos_voice` 消费播放。

| 字段 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `state` | int8 | 70 | 语音/状态码，见 [语音状态表](#语音状态-voice_cmdstate) |
| `type` | int8 | 0 | 子类型（部分状态使用，如选歌/静音/表演模式） |
| `audio_file` | string | "" | 指定播放的音频文件（相对 `lumos_voice` 音频目录，或**绝对路径**） |

```bash
curl -X POST http://192.168.1.1:8080/api/voice_cmd \
  -H "Content-Type: application/json" \
  -d '{"state": 108, "type": 0, "audio_file": ""}'
```

---

## 二、MP3 音频上传播放

### `POST /api/audio` — 上传 MP3 并立即播放

上传一个 MP3 文件，保存到 `audio_dir`，并通过 `lcm_robot_status.audio_file`（绝对路径）触发 `lumos_voice` 播放。**无需修改 `lumos_voice`**。

- **请求**: `multipart/form-data`
  - `file`（必填）：MP3 文件（最大 20 MB）
- **响应**:

```json
{"status": "ok", "file": "greeting.mp3", "path": "/devel/lumos_ws/lumos_audio/greeting.mp3"}
```

```bash
curl -X POST http://192.168.1.1:8080/api/audio \
  -F "file=@/path/to/greeting.mp3"
```


> 文件名会经过 `os.path.basename` 清洗（防路径穿越）；非 `.mp3` 后缀会自动补 `.mp3`。若机器人处于全局静音（`voice_cmd state=111 type=1`），上传的音频不会被播放（遵循静音语义）。

---

## 三、数据获取

### `GET /api/health` — 健康检查

```json
{"status": "ok", "sn": "<机器人序列号>"}
```

### `GET /api/robot_info` — 机器人信息汇总

```json
{
  "robot_id": "<sn>",
  "sys_info": { /* RK3588 系统信息：CPU/内存/温度/磁盘等 */ },
  "battery": { "voltage": 28.5, "current": -1.2, "percentage": 86.0 },
  "status": { "state": 0, "type": 0, "audio_file": "" }
}
```

### `GET /api/telemetry` — 全量遥测缓存

```json
{
  "robot_status": { "state": 0, "type": 0, "audio_file": "" },
  "diagnosis": {
    "<component>": { "timestamp": 1234567890, "component": "xxx", "status": 0, "detail": "..." }
  },
  "motor_data": {
    "<limb>": [
      { "errcode": 0, "pos": 0.12, "vol": 0, "vel": 0.0, "cur": 0, "tor": 0.0, "temp": 35.2 }
    ]
  },
  "sys_info": {},
  "battery_info": { "voltage": 28.5, "current": -1.2, "percentage": 86.0 }
}
```

> `sys_info` 在此接口始终为 `{}`，系统信息请用 `/api/robot_info`。

---

## 四、视频流

### `GET /api/video` — 实时视频流（MJPEG）

返回 `multipart/x-mixed-replace` JPEG 帧流（`/dev/video0`，1280x720@20fps）。

```html
<img src="http://192.168.1.1:8080/api/video" />
```

> **流实现**：服务端为每个请求拉起一个 `ffmpeg`（`v4l2` → `mjpeg`，`/dev/video0`，1280x720@20fps），逐帧封装为 MJPEG multipart 输出。客户端断开后 ffmpeg 会被及时回收；若连续约 10 秒无帧（相机停流），流会自动结束。
>
> **排查**：若 `/api/video` 返回 HTTP 200 但为空流（无任何帧数据），通常是残留的孤儿 `ffmpeg` 进程占用 `/dev/video0` 所致（手动跑 ffmpeg 会报 `Device or resource busy`）。在机器人上执行 `ps aux | grep ffmpeg` 找到并 `kill` 该进程后重试即可。

### `POST /api/video/toggle` — 开关视频流

```bash
curl -X POST http://192.168.1.1:8080/api/video/toggle \
  -H "Content-Type: application/json" -d '{"enabled": false}'
```

响应：`{"status": "ok", "video_enabled": false}`

### `GET /api/video/status` — 视频流状态

```json
{"video_enabled": true}
```

---

## 五、枚举参考

> 权威来源：`lumos_controller/include/state/state_base.hpp` 与 `biz/lumos_robot.cpp`（`mapLCMStateToRobotState`、`static_cast<PolicyType>`、`static_cast<ArmState>`）。

### 状态码 `state`（`/api/cmd`）

| 值 | 含义 |
|----|------|
| 1 | RESET（复位） |
| 2 | STAND（站立） |
| 3 | RL_WALK（行走） |
| 6 | ST_MIMIC（表演） |
| 7 | STAND_WALK |
| 10 | DEBUG |
| 12 | RL_WALK_AMP |
| 13 | RL_RUN_AMP |
| 20 | BY_MIMIC |
| 21 | BFM_MIMIC |
| 22 | IMU 软标定（直接触发，不切状态） |
| 23 | IMU 硬标定（直接触发，不切状态） |

> 值 14/15/16 未在控制协议中定义（仅在语音侧映射到 MIMIC 提示音）。旧枚举 `FSM_StateEnum`（`third_party/biped_lcm_types/biped_enum.h`）已被上述 `LCM_STATE_*` 取代，不再使用。

### 策略 `policy_type`（`/api/cmd`）

| 值 | 含义 |
|----|------|
| 0 | DefaultPolicy |
| 1 | mimic1 |
| 2 | mimic2 |
| 3 | mimic3 |
| 4 | mimic4 |
| 5 | mimic5 |
| 6 | mimic6 |

### 手臂 `armstate`（`/api/arm_cmd`）

| 值 | 含义 |
|----|------|
| 2 | Raise_right_hand |
| 3 | Reset_right_hand |
| 4 | Wave_left_hand |
| 5 | Wave_right_hand |
| 6 | Wave_arm |
| 7 | Reset_arm |
| 9 | Bixin_action |
| 10 | cross_waist_action |
| 11 | fist_action |
| 12 | Superman_action |
| 13 | wave_action |
| 21 | None |
| 22+ | 动态 arm_motions（由 policy `arm_motions.yaml` 映射整数→轨迹文件） |

### LED 控制字 `control_word`（`/api/led`）

| 值 | 枚举 | 含义 |
|----|------|------|
| 1 | blueOn | 蓝灯常亮 |
| 2 | blueLink | 蓝灯闪烁 |
| 3 | greenOn | 绿灯常亮 |
| 4 | greenLink | 绿灯闪烁 |
| 5 | yellowLink | 黄灯闪烁 |
| 6 | redLink | 红灯闪烁 |
| 7 | yellow_blue_link | 黄蓝交替闪烁 |
| 8 | resetCmd | 复位/断电重启 |

### 语音状态 `state`（`/api/voice_cmd`）

由 `lumos_voice` 的 `LCMListener` 消费（`lumos_voice_sdk/iot/things/lcm_listener.py`）。

**告警（负数，打断当前播放并立即播报）**

| 值 | 含义 |
|----|------|
| -1 | Low_battery（电量不足） |
| -2 | Low_voltage（电压不足） |
| -3 | Motor_temperature_over（电机过热） |
| -4 | cut_off_soon（即将断电） |
| -5 | 电量低于 25% |
| -6 | motor_error（电机故障） |
| -8 | command_error（指令有误） |
| -10 | motor_over_current（过流） |
| -11 | motor_under_voltage（欠压） |
| -12 | motor_over_voltage（过压） |
| -13 | motor_uncalibrated（未标定） |
| -14 | udp_lost_package |
| -15 | udp_timeout |
| -16 | udp_disconnected |
| -17 | can_disconnected |

**状态音（0–49）**

| 值 | 含义 |
|----|------|
| 0 | 已上电 |
| 1 | 复位 |
| 2 | 站立 |
| 3 | wave_arm_walk |
| 4 | 起身 |
| 5 | 躺下 |
| 6 | 表演模式 |
| 10 | debug 模式 |
| 11 | 导航模式 |
| 12 | 行走 |
| 13 | walk_run |
| 14–16 | RL_MIMIC1 |
| 17–18 | 表演提示 |
| 30–34 | 固定短语（比个心/帅不帅/多多指教/超人起飞/你好） |

**特殊状态**

| 值 | `type` | 含义 |
|----|--------|------|
| 50 | 0–3 | 手动选歌（type→dance1~dance4.mp3；同条再发即停） |
| 70–85 | — | operation 表演提示音（lcm_70~lcm_85） |
| 98–101 | — | 固定音频序列 |
| 102 | — | 打断 state=108 的播放 |
| 103 | — | 触发“自动连接并进入 listening” |
| 108 | — | 顺序播报（lcm_108_1~8） |
| 109 | — | 固定音频序列（lcm_109_1~4） |
| 110 | 1/0 | 进入/退出 operation 表演模式 |
| 111 | 1/0 | 全局静音 开/关 |

> 设置 `audio_file`（相对 `lumos_voice` 音频目录或绝对路径）时，`LCMListener` 会优先播放该文件并忽略 `state` 的常规状态音逻辑。
