#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <map>
#include <vector>
#include <glog/logging.h>

/**
 * @brief   NIX2 SDK 关节级控制 — 支持单关节 / 单肢体 / 全身控制
 *
 *         控制范围（三选一，取消注释对应的宏）:
 *           CONTROL_ALL            — 全身 21 关节
 *           CONTROL_COMPONENT XXX  — 指定肢体 (ARM_L / ARM_R / WAIST / LEG_L / LEG_R)
 *           CONTROL_SINGLE  <idx>  — 指定单关节 (0-20, 全局索引见下方注释)
 *
 *         下发模式（二选一）:
 *           MODE_ONESHOT    — 测试: 进入 STAND → 下发一次目标位置 → 保持至 Ctrl-C
 *           MODE_CONTINUOUS — 正式: 进入 STAND → 循环持续下发 (200 Hz)
 *
 *         操作流程:
 *           1. 机器人运行 lumos_controller，SDK 程序负责切换 SDK 模式
 *           2. 本地运行: ./build/nix_cmd_all_joints
 *           3. Ctrl-C 安全退出 (自动 RESET)
 *
 *         全局关节索引 (NIX2, 21 关节):
 *           0-3:   ARM_L  [肩俯仰, 肩横滚, 肩偏航, 肘]
 *           4-7:   ARM_R  [肩俯仰, 肩横滚, 肩偏航, 肘]
 *           8:     WAIST  [腰]
 *           9-14:  LEG_L  [髋俯仰, 髋横滚, 髋偏航, 膝, 踝俯仰, 踝横滚]
 *           15-20: LEG_R  [髋俯仰, 髋横滚, 髋偏航, 膝, 踝俯仰, 踝横滚]
 *
 * @author  jiangbin
 * @date    2026-05-26
 */

// ═══════════════════════════════════════════════════════════════════
// 控制范围 — 三选一，只保留一个不注释的
// ═══════════════════════════════════════════════════════════════════
//#define CONTROL_ALL
#define CONTROL_COMPONENT static_cast<int>(SdkComponentType::LEG_R)  // ARM_L | ARM_R | WAIST | LEG_L | LEG_R
//#define CONTROL_SINGLE  8          // 全局索引: 8 = WAIST 腰

// ═══════════════════════════════════════════════════════════════════
// 下发模式 — 二选一
// ═══════════════════════════════════════════════════════════════════
#define MODE_ONESHOT
//#define MODE_CONTINUOUS

// ── 常量 ─────────────────────────────────────────────────────────
static constexpr int    kControlHz        = 200;
static constexpr int    kTotalJoints      = 21;
static constexpr int    kArmJointsPerSide = 4;
static constexpr int    kLegJointsPerSide = 6;
static constexpr int    kWaistJoints      = 1;
static constexpr float  kControlDt        = 1.0f / kControlHz;
static constexpr float  kRampDuration     = 2.0f;   // 插值时间 (秒)
static constexpr int    kStandSettleSec   = 11;     // controller StandState 插值约 10s，留 1s 余量

// ── 全局 ─────────────────────────────────────────────────────────
static volatile bool g_running = true;
static void sigint_handler(int) { g_running = false; }

// ── 实时关节位置缓存 (component_type,joint_id) → pos_high ──────
static std::mutex g_jpos_mutex;
static std::map<std::pair<int16_t, int16_t>, float> g_current_pos;

static void on_joint_data(const sdk_lcmt_joint_datasets* msg) {
    std::lock_guard<std::mutex> lock(g_jpos_mutex);
    for (int i = 0; i < msg->datasets_num; i++) {
        const auto& d = msg->datasets[i];
        g_current_pos[{d.component_type, d.joint_id}] = d.pos_high;
    }
}

// ═══════════════════════════════════════════════════════════════════
// 每个关节的预留接口 — 修改下面这些数组来控制目标值
// ═══════════════════════════════════════════════════════════════════

struct JointTarget {
    int   component_type;   // SdkComponentType 的 LCM 数值，必须与 controller 端保持一致
    int   joint_id;         // 组件内索引
    int   ctrlWord;         // 3=MIT/PD控制, 5=力矩控制；运动测试使用 3
    float tarPos;           // 目标位置 (rad)
    float tarVel;           // 目标速度 (rad/s)
    float tarTor;           // 前馈力矩 (Nm)
    float kp;               // 刚度 (Nm/rad)
    float kd;               // 阻尼 (Nm·s/rad)
};

// --- ARM_L (全局索引 0-3) ------------------------------------------
static JointTarget g_arm_l[4] = {
    // component_type, joint_id, ctrlWord, tarPos, tarVel, tarTor, kp,  kd
    // 左臂前伸验证姿态。幅度保守，先用于确认单肢体覆盖链路；如方向相反，
    // 优先只调整 shoulder_pitch 的符号，不要一次扩大多个关节幅度。
    { static_cast<int>(SdkComponentType::ARM_L), 0, 3, -0.35f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肩俯仰：前伸主关节
    { static_cast<int>(SdkComponentType::ARM_L), 1, 3,  0.20f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肩横滚：接近站立外展
    { static_cast<int>(SdkComponentType::ARM_L), 2, 3, -0.20f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肩偏航：轻微内收对齐前方
    { static_cast<int>(SdkComponentType::ARM_L), 3, 3, -0.45f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肘：轻微弯曲，避免完全伸直
};

// --- ARM_R (全局索引 4-7) ------------------------------------------
static JointTarget g_arm_r[4] = {
    // 右臂前伸验证姿态。肩俯仰与左臂同号；横滚/偏航按左右镜像取相反号。
    { static_cast<int>(SdkComponentType::ARM_R), 0, 3, -0.35f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肩俯仰：前伸主关节
    { static_cast<int>(SdkComponentType::ARM_R), 1, 3, -0.20f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肩横滚：接近站立外展
    { static_cast<int>(SdkComponentType::ARM_R), 2, 3,  0.20f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肩偏航：轻微内收对齐前方
    { static_cast<int>(SdkComponentType::ARM_R), 3, 3, -0.45f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肘：轻微弯曲，避免完全伸直
};

// --- WAIST (全局索引 8) ---------------------------------------------
static JointTarget g_waist[1] = {
    // 默认单关节测试给腰一个小角度，便于确认 SDK 指令确实覆盖了 stand 状态命令。
    { static_cast<int>(SdkComponentType::WAIST), 0, 3,  0.30f,  0.0f, 0.0f, 160.0f, 6.0f },
};

// --- LEG_L (全局索引 9-14) -----------------------------------------
static JointTarget g_leg_l[6] = {
    // 左腿前伸验证姿态。只改矢状面三个关节，横滚/偏航保持站立值，降低侧向失稳风险。
    { static_cast<int>(SdkComponentType::LEG_L), 0, 3, -0.35f,  0.0f, 0.0f, 160.0f, 6.0f },  // 髋俯仰：从站立 -0.10 小幅前伸
    { static_cast<int>(SdkComponentType::LEG_L), 1, 3,  0.0f,   0.0f, 0.0f, 160.0f, 6.0f },  // 髋横滚：保持站立
    { static_cast<int>(SdkComponentType::LEG_L), 2, 3,  0.0f,   0.0f, 0.0f, 160.0f, 6.0f },  // 髋偏航：保持站立
    { static_cast<int>(SdkComponentType::LEG_L), 3, 3,  0.35f,  0.0f, 0.0f, 160.0f, 6.0f },  // 膝：轻微屈膝，给前伸留余量
    { static_cast<int>(SdkComponentType::LEG_L), 4, 3, -0.22f,  0.0f, 0.0f,  60.0f, 0.8f },  // 踝俯仰：配合髋/膝，保持脚姿态温和变化
    { static_cast<int>(SdkComponentType::LEG_L), 5, 3,  0.0f,   0.0f, 0.0f,  60.0f, 0.8f },  // 踝横滚：保持站立
};

// --- LEG_R (全局索引 15-20) ----------------------------------------
static JointTarget g_leg_r[6] = {
    // 右腿前伸验证姿态。与左腿同号，因为左右髋/膝/踝俯仰轴方向一致。
    { static_cast<int>(SdkComponentType::LEG_R), 0, 3, -0.35f,  0.0f, 0.0f, 160.0f, 6.0f },  // 髋俯仰：从站立 -0.10 小幅前伸
    { static_cast<int>(SdkComponentType::LEG_R), 1, 3,  0.0f,   0.0f, 0.0f, 160.0f, 6.0f },  // 髋横滚：保持站立
    { static_cast<int>(SdkComponentType::LEG_R), 2, 3,  0.0f,   0.0f, 0.0f, 160.0f, 6.0f },  // 髋偏航：保持站立
    { static_cast<int>(SdkComponentType::LEG_R), 3, 3,  0.35f,  0.0f, 0.0f, 160.0f, 6.0f },  // 膝：轻微屈膝，给前伸留余量
    { static_cast<int>(SdkComponentType::LEG_R), 4, 3, -0.22f,  0.0f, 0.0f,  60.0f, 0.8f },  // 踝俯仰：配合髋/膝，保持脚姿态温和变化
    { static_cast<int>(SdkComponentType::LEG_R), 5, 3,  0.0f,   0.0f, 0.0f,  60.0f, 0.8f },  // 踝横滚：保持站立
};

// ═══════════════════════════════════════════════════════════════════
// 将 JointTarget 组装为 SdkJointCmd
// ═══════════════════════════════════════════════════════════════════
static SdkJointCmd make_cmd(const JointTarget& t) {
    SdkJointCmd cmd;
    cmd.component_type = t.component_type;
    cmd.joint_id       = t.joint_id;
    cmd.ctrlWord       = t.ctrlWord;
    cmd.tarPos         = t.tarPos;
    cmd.tarVel         = t.tarVel;
    cmd.tarTor         = t.tarTor;
    cmd.res1           = t.kp;
    cmd.res2           = t.kd;
    return cmd;
}

// ═══════════════════════════════════════════════════════════════════
// 根据宏选择要控制的关节集合
// ═══════════════════════════════════════════════════════════════════
static std::vector<SdkJointCmd> build_target_cmds() {
    std::vector<SdkJointCmd> cmds;

#if defined(CONTROL_ALL)
    for (auto& j : g_arm_l)  cmds.push_back(make_cmd(j));
    for (auto& j : g_arm_r)  cmds.push_back(make_cmd(j));
    for (auto& j : g_waist)  cmds.push_back(make_cmd(j));
    for (auto& j : g_leg_l)  cmds.push_back(make_cmd(j));
    for (auto& j : g_leg_r)  cmds.push_back(make_cmd(j));

#elif defined(CONTROL_COMPONENT)
    // CONTROL_COMPONENT 定义为 SdkComponentType 的整数值。
    #define XX(name, arr) if (CONTROL_COMPONENT == static_cast<int>(SdkComponentType::name)) { for (auto& j : arr) cmds.push_back(make_cmd(j)); }
    XX(ARM_L,  g_arm_l)
    XX(ARM_R,  g_arm_r)
    XX(WAIST,  g_waist)
    XX(LEG_L,  g_leg_l)
    XX(LEG_R,  g_leg_r)
    #undef XX

#elif defined(CONTROL_SINGLE)
    // CONTROL_SINGLE 定义为全局索引 0-20
    static JointTarget* all_joints[kTotalJoints] = {
        &g_arm_l[0], &g_arm_l[1], &g_arm_l[2], &g_arm_l[3],       // 0-3
        &g_arm_r[0], &g_arm_r[1], &g_arm_r[2], &g_arm_r[3],       // 4-7
        &g_waist[0],                                                // 8
        &g_leg_l[0], &g_leg_l[1], &g_leg_l[2], &g_leg_l[3], &g_leg_l[4], &g_leg_l[5], // 9-14
        &g_leg_r[0], &g_leg_r[1], &g_leg_r[2], &g_leg_r[3], &g_leg_r[4], &g_leg_r[5], // 15-20
    };
    int idx = CONTROL_SINGLE;
    if (idx >= 0 && idx < kTotalJoints) {
        cmds.push_back(make_cmd(*all_joints[idx]));
    }
#endif

    return cmds;
}
 

// ═══════════════════════════════════════════════════════════════════
// 状态名辅助
// ═══════════════════════════════════════════════════════════════════
static const char* state_name(int8_t s) {
    switch (s) {
        case 0:  return "NOT_A_STATE";
        case 1:  return "RESET";
        case 2:  return "STAND";
        case 3:  return "RL_WALK";
        case 5:  return "RL_LIEDOWN";
        case 6:  return "ST_MIMIC";
        case 11: return "RL_NAV";
        case 12: return "RL_WALK_AMP";
        case 20: return "BY_MIMIC";
        case 21: return "BFM_MIMIC";
        default: return "UNKNOWN";
    }
}

static bool is_main_robot_state(int state) {
    switch (state) {
        case 0:   // NOT_A_STATE
        case static_cast<int>(SdkStateType::RESET):
        case static_cast<int>(SdkStateType::STAND):
        case static_cast<int>(SdkStateType::RL_WALK):
        case static_cast<int>(SdkStateType::RL_LIEDOWN):
        case static_cast<int>(SdkStateType::RL_MIMIC):
        case static_cast<int>(SdkStateType::RL_NAV):
        case static_cast<int>(SdkStateType::RL_WALK_AMP):
        case static_cast<int>(SdkStateType::BY_MIMIC):
        case static_cast<int>(SdkStateType::BFM_MIMIC):
            return true;
        default:
            return false;
    }
}

// ═══════════════════════════════════════════════════════════════════
// 等待状态转移
// ═══════════════════════════════════════════════════════════════════
static std::atomic<int> g_robot_state{0};
static std::atomic<bool> g_got_status{false};
static std::atomic<int> g_imu_count{0};

static void on_robot_status(const robot_status_lcmt* msg) {
    if (!is_main_robot_state(msg->state)) {
        // lcm_robot_status 也承载语音、电量、手臂动作等通知；这些不是主状态机状态，
        // 不能覆盖 g_robot_state，否则 wait_state(STAND/RESET) 会被 UNKNOWN 污染。
        LOG_EVERY_N(WARNING, 50) << "Ignore non-main robot_status state="
                                 << static_cast<int>(msg->state)
                                 << ", type=" << static_cast<int>(msg->type)
                                 << ", audio_file='" << msg->audio_file << "'";
        return;
    }
    g_robot_state = msg->state;
    g_got_status = true;
}

static void on_imu_data(const microstrain_lcmt*) {
    g_imu_count++;
}

static bool wait_state(int target, int timeout_s) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
    LOG(INFO) << "Waiting for " << state_name((int8_t)target)
              << "(" << target << ") ... timeout=" << timeout_s << "s";
    while (std::chrono::steady_clock::now() < deadline) {
        usleep(50000);
        if (g_robot_state.load() == target) {
            LOG(INFO) << "Reached " << state_name((int8_t)target);
            return true;
        }
    }
    LOG(ERROR) << "Timeout waiting for " << state_name((int8_t)target)
               << ", current=" << state_name((int8_t)g_robot_state.load())
               << "(" << g_robot_state.load() << ")";
    return false;
}

static void ensure_reset_briefly(SdkRobotManager& manager) {
    if (g_robot_state.load() == static_cast<int>(SdkStateType::RESET)) {
        LOG(INFO) << "Robot is already RESET; skip RESET command.";
        return;
    }

    LOG(INFO) << "Robot is not reported as RESET"
              << (g_got_status.load() ? "" : " (no robot_status received yet)")
              << ", send RESET and wait 3s. current="
              << state_name((int8_t)g_robot_state.load())
              << "(" << g_robot_state.load() << ")";
    manager.SendRobotCmd(SdkStateType::RESET);
    sleep(3);
}

// ═══════════════════════════════════════════════════════════════════
// 打印当前关节目标
// ═══════════════════════════════════════════════════════════════════
static void print_cmds(const std::vector<SdkJointCmd>& cmds) {
    std::ostringstream oss;
    oss << "Sending " << cmds.size() << " joint(s):\n";
    for (auto& c : cmds) {
        oss << "  comp=" << (int)c.component_type
            << " jid="  << (int)c.joint_id
            << " ctrl=" << (int)c.ctrlWord
            << " pos="  << std::fixed << std::setprecision(3) << c.tarPos
            << " vel="  << c.tarVel
            << " tor="  << c.tarTor
            << " kp="   << c.res1
            << " kd="   << c.res2
            << "\n";
    }
    LOG(INFO) << oss.str();
}

// ═══════════════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    signal(SIGINT,  sigint_handler);
    signal(SIGTERM, sigint_handler);

    FLAGS_stderrthreshold = 0;
    FLAGS_minloglevel     = 0;
    google::InitGoogleLogging(argv[0]);

    int exit_code = 0;

    SdkRobotManager manager;
    manager.Init();
    manager.SetRobotStatusCb(on_robot_status);
    manager.SetImuDataCb(on_imu_data);
    manager.SetJointDataCb(on_joint_data);

    // 构建目标指令（由编译期宏选择 CONTROL_ALL/COMPONENT/SINGLE）
    auto cmds = build_target_cmds();
    if (cmds.empty()) {
        LOG(ERROR) << "No joints selected — check CONTROL_xxx macro.";
        return 1;
    }
    print_cmds(cmds);

    // 检查 LCM 是否通（IMU 高频发布，比 robot_status 更可靠）
    LOG(INFO) << "Waiting for LCM data (checking IMU)...";
    for (int i = 0; i < 30 && g_imu_count == 0; i++) usleep(100000);
    LOG(INFO) << "IMU messages received: " << g_imu_count.load();
    if (g_imu_count == 0) {
        LOG(ERROR) << "No LCM data received — check multicast route and network.";
        exit_code = 1;
        goto cleanup;
    }

    // ── RESET ─────────────────────────────────────────────────────
    LOG(INFO) << "=== Step 1: RESET ===";
    if (!g_running) goto cleanup;
    ensure_reset_briefly(manager);

    // ── STAND ─────────────────────────────────────────────────────
    LOG(INFO) << "=== Step 2: STAND ===";
    if (!g_running) goto cleanup;
    manager.SendRobotCmd(SdkStateType::STAND);
    if (!wait_state(2, 15)) goto cleanup;
    LOG(INFO) << "STAND state reported. Waiting " << kStandSettleSec
              << "s for stand interpolation to finish before SDK joint control.";
    for (int i = 0; i < kStandSettleSec && g_running; ++i) {
        sleep(1);
    }
    if (!g_running) goto cleanup;

    // ── 进入 SDK 模式 ─────────────────────────────────────────────
    // controller 端只允许在 STAND/RESET 状态，或手柄已授权 SDK 模式时接受
    // sdk_lcm_set_type_cmd。这里放在 STAND 成功之后，避免启动初期被拒绝。
    LOG(INFO) << "=== Step 3: Enter SDK mode ===";
    if (!g_running) goto cleanup;
    manager.SendModeCmd(1);
    sleep(1);

    // ── 下发关节指令 ──────────────────────────────────────────────
    LOG(INFO) << "=== Step 4: Send joint commands ===";

#ifdef MODE_ONESHOT
    // ── 从当前实际位置平滑插值到目标位置 ──────────────────────
    {
        // 拷贝目标值到可变数组，后续每帧覆盖 tarPos
        std::vector<SdkJointCmd> ramp_cmds = cmds;
        std::vector<float> start_pos(ramp_cmds.size());
        std::vector<float> target_pos(ramp_cmds.size());

        // 读取当前实际位置作为插值起点
        {
            std::lock_guard<std::mutex> lock(g_jpos_mutex);
            for (size_t i = 0; i < ramp_cmds.size(); i++) {
                auto key = std::make_pair((int16_t)ramp_cmds[i].component_type,
                                          (int16_t)ramp_cmds[i].joint_id);
                auto it = g_current_pos.find(key);
                start_pos[i] = (it != g_current_pos.end()) ? it->second : ramp_cmds[i].tarPos;
                target_pos[i] = ramp_cmds[i].tarPos;
            }
        }

        int    ramp_steps = static_cast<int>(kRampDuration * kControlHz);
        auto   t0         = std::chrono::steady_clock::now();

        LOG(INFO) << "Ramping from current position to target over "
                  << kRampDuration << "s (" << ramp_steps << " steps)";

        for (int step = 0; step < ramp_steps && g_running; step++) {
            float alpha = static_cast<float>(step + 1) / ramp_steps;
            // ease-in-out: smoother start and end
            float s = alpha * alpha * (3.0f - 2.0f * alpha);

            for (size_t i = 0; i < ramp_cmds.size(); i++) {
                ramp_cmds[i].tarPos = start_pos[i] + (target_pos[i] - start_pos[i]) * s;
            }
            manager.SendJointCmds(ramp_cmds);

            auto next = t0 + std::chrono::microseconds(
                static_cast<int>((step + 1) * kControlDt * 1e6f));
            auto now = std::chrono::steady_clock::now();
            if (next > now) {
                std::this_thread::sleep_until(next);
            }
        }
    }
    LOG(INFO) << "Ramp complete. Holding position. Press Ctrl-C to exit.";
    manager.SendJointCmds(cmds);  // 精确到达目标
    while (g_running) { usleep(100000); }

#elif defined(MODE_CONTINUOUS)
    LOG(INFO) << "Continuous loop at " << kControlHz << " Hz. Press Ctrl-C to stop.";
    auto next_tick = std::chrono::steady_clock::now();
    while (g_running) {
        manager.SendJointCmds(cmds);
        next_tick += std::chrono::microseconds(static_cast<int>(kControlDt * 1e6f));
        auto now = std::chrono::steady_clock::now();
        if (next_tick > now) {
            std::this_thread::sleep_until(next_tick);
        } else {
            // 掉帧，重新对齐
            next_tick = now;
        }
    }
#endif

    // ── 恢复 ──────────────────────────────────────────────────────
    LOG(INFO) << "=== Step 5: Return to safe state ===";
    manager.SendRobotCmd(SdkStateType::RESET);
    wait_state(1, 10);
    sleep(3);
    manager.SendModeCmd(0);
    sleep(1);

cleanup:
    if (!g_running) {
        LOG(WARNING) << "Interrupted, returning to safe state...";
        manager.SendRobotCmd(SdkStateType::STAND);
        sleep(2);
        manager.SendRobotCmd(SdkStateType::RESET);
        sleep(3);
        manager.SendModeCmd(0);
    }
    google::ShutdownGoogleLogging();
    return g_running ? exit_code : 1;
}
