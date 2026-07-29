#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <map>
#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <glog/logging.h>

/**
 * @brief   NIX2 DEBUG 关节级控制示例 / 策略轨迹回放工具
 *
 *         控制源（四选一，只能保留一个宏）:
 *           CONTROL_ALL            — 使用本文件示例目标值，控制全身 21 关节
 *           CONTROL_COMPONENT XXX  — 使用本文件示例目标值，控制指定组件
 *                                    (ARM_L / ARM_R / WAIST / LEG_L / LEG_R)
 *           CONTROL_SINGLE  <idx>  — 使用本文件示例目标值，控制单个全局索引关节
 *           CONTROL_REPLAY         — 读取 store_ref_motion.txt + kp_kd.yaml，
 *                                    按策略轨迹连续下发 21 个关节目标
 *
 *         下发模式:
 *           非回放控制源必须启用 MODE_ONESHOT:
 *             进入 STAND → 从当前关节位置插值到示例目标值 → 保持至 Ctrl-C
 *           CONTROL_REPLAY 不使用 MODE_ONESHOT / MODE_CONTINUOUS:
 *             argv[1] 指定回放遍数，默认 1 遍；<=0 表示无限循环直到 Ctrl-C
 *
 *         操作流程:
 *           1. 机器人端运行 lumos_controller
 *           2. 可选: 先运行 ./nix_lcm_sub 纯监听记录数据
 *           3. 本地 build 目录运行:
 *                ./nix_joint_cmd --hold-seconds 0.5  # 非回放示例自动结束
 *                ./nix_joint_cmd                     # 非回放示例保持到 Ctrl-C
 *                ./nix_joint_cmd 3                   # CONTROL_REPLAY 时回放 3 遍
 *                ./nix_joint_cmd 0                   # CONTROL_REPLAY 时无限回放
 *           4. 本程序负责进入/退出 DEBUG 状态；其他监听程序不要发送状态命令
 *
 *         全局关节索引 / SDK 下发顺序 (NIX2, 21 关节):
 *           0-5:   LEG_L  [髋俯仰, 髋横滚, 髋偏航, 膝, 踝俯仰, 踝横滚]
 *           6-11:  LEG_R  [髋俯仰, 髋横滚, 髋偏航, 膝, 踝俯仰, 踝横滚]
 *           12:    WAIST  [腰]
 *           13-16: ARM_L  [肩俯仰, 肩横滚, 肩偏航, 肘]
 *           17-20: ARM_R  [肩俯仰, 肩横滚, 肩偏航, 肘]
 *
 *         注意:
 *           - MODE_CONTINUOUS 已移除；回放是否循环由 argv[1] 控制。
 *           - CONTROL_REPLAY 的 store_ref_motion.txt 列顺序来自 ref_motion_fields.yaml，
 *             下发前通过 kReplayMapping 重排为 SDK 组件顺序。
 *
 * @author  jiangbin
 * @date    2026-05-26
 */

// ═══════════════════════════════════════════════════════════════════
// 控制源 — 四选一，只保留一个不注释的
// ═══════════════════════════════════════════════════════════════════
// 示例目标控制：使用下方 g_leg_l/g_arm_l/... 数组里的固定目标值。
//#define CONTROL_ALL          
//#define CONTROL_COMPONENT static_cast<int>(SdkComponentType::LEG_R) // ARM_L | ARM_R | WAIST | LEG_L | LEG_R
#define CONTROL_SINGLE  12        // 全局索引: 12 = WAIST 腰

// 策略轨迹回放：读取 store_ref_motion.txt + kp_kd.yaml，按 argv[1] 指定遍数回放。
// #define CONTROL_REPLAY

// 仅非 CONTROL_REPLAY 控制源使用；CONTROL_REPLAY 不要打开这个宏。
#define MODE_ONESHOT

#if (defined(CONTROL_ALL) + defined(CONTROL_COMPONENT) + defined(CONTROL_SINGLE) + defined(CONTROL_REPLAY)) != 1
#error "Define exactly one control source: CONTROL_ALL, CONTROL_COMPONENT, CONTROL_SINGLE, or CONTROL_REPLAY."
#endif

#ifdef MODE_CONTINUOUS
#error "MODE_CONTINUOUS has been removed. CONTROL_REPLAY loops continuously by frame count; non-replay modes use MODE_ONESHOT."
#endif

#ifdef CONTROL_REPLAY
#ifdef MODE_ONESHOT
#error "CONTROL_REPLAY does not use MODE_ONESHOT. Use argv[1] to set replay loop count."
#endif
#else
#ifndef MODE_ONESHOT
#error "Non-replay control requires MODE_ONESHOT."
#endif
#endif

// ── 常量 ─────────────────────────────────────────────────────────
static constexpr int    kControlHz        = 100;    // 控制频率 (Hz) 
static constexpr int    kTotalJoints      = 21;     // NIX2 关节总数
static constexpr float  kControlDt        = 1.0f / kControlHz;
static constexpr float  kRampDuration     = 2.0f;   // 插值时间 (秒)
static constexpr int    kStandSettleSec   = 11;     // controller StandState 插值约 10s，留 1s 余量
static constexpr int    kReplayFieldNum   = 45;     // store_ref_motion.txt 每帧列数
static constexpr int    kDefaultReplayLoops = 1;    // <=0 表示无限循环，直到 Ctrl-C

// ── 轨迹回放文件路径（程序从 build/ 目录运行，所以这里相对 build/）─
static const char* kReplayPolicyDir  = "../models/nix2_policy/sanlin_04101426";
static const char* kReplayMotionFile = "store_ref_motion.txt";
static const char* kReplayKpKdFile   = "kp_kd.yaml";

// ── 全局 ─────────────────────────────────────────────────────────
static volatile std::sig_atomic_t g_running = 1;
static void sigint_handler(int) { g_running = 0; }

// ── 实时关节位置缓存 (component_type,joint_id) → pos_high ──────
static std::mutex g_jpos_mutex;
static std::map<std::pair<int16_t, int16_t>, float> g_current_pos;

static void on_joint_data(const joint_datasets_lcmt* msg) {
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

// --- ARM_L (全局索引 13-16) -----------------------------------------
static JointTarget g_arm_l[4] = {
    // component_type, joint_id, ctrlWord, tarPos, tarVel, tarTor, kp,  kd
    // 左臂前伸验证姿态。幅度保守，先用于确认单肢体覆盖链路；如方向相反，
    // 优先只调整 shoulder_pitch 的符号，不要一次扩大多个关节幅度。
    { static_cast<int>(SdkComponentType::ARM_L), 0, 3, -0.35f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肩俯仰：前伸主关节
    { static_cast<int>(SdkComponentType::ARM_L), 1, 3,  0.20f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肩横滚：接近站立外展
    { static_cast<int>(SdkComponentType::ARM_L), 2, 3, -0.20f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肩偏航：轻微内收对齐前方
    { static_cast<int>(SdkComponentType::ARM_L), 3, 3, -0.45f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肘：轻微弯曲，避免完全伸直
};

// --- ARM_R (全局索引 17-20) -----------------------------------------
static JointTarget g_arm_r[4] = {
    // 右臂前伸验证姿态。肩俯仰与左臂同号；横滚/偏航按左右镜像取相反号。
    { static_cast<int>(SdkComponentType::ARM_R), 0, 3, -0.35f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肩俯仰：前伸主关节
    { static_cast<int>(SdkComponentType::ARM_R), 1, 3, -0.20f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肩横滚：接近站立外展
    { static_cast<int>(SdkComponentType::ARM_R), 2, 3,  0.20f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肩偏航：轻微内收对齐前方
    { static_cast<int>(SdkComponentType::ARM_R), 3, 3, -0.45f,  0.0f, 0.0f, 120.0f, 5.0f },  // 肘：轻微弯曲，避免完全伸直
};

// --- WAIST (全局索引 12) --------------------------------------------
static JointTarget g_waist[1] = {
    // 默认单关节测试给腰一个小角度，便于确认 SDK 指令确实覆盖了 stand 状态命令。
    { static_cast<int>(SdkComponentType::WAIST), 0, 3,  0.30f,  0.0f, 0.0f, 160.0f, 6.0f },
};

// --- LEG_L (全局索引 0-5) -------------------------------------------
static JointTarget g_leg_l[6] = {
    // 左腿前伸验证姿态。只改矢状面三个关节，横滚/偏航保持站立值，降低侧向失稳风险。
    { static_cast<int>(SdkComponentType::LEG_L), 0, 3, -0.35f,  0.0f, 0.0f, 160.0f, 6.0f },  // 髋俯仰：从站立 -0.10 小幅前伸
    { static_cast<int>(SdkComponentType::LEG_L), 1, 3,  0.0f,   0.0f, 0.0f, 160.0f, 6.0f },  // 髋横滚：保持站立
    { static_cast<int>(SdkComponentType::LEG_L), 2, 3,  0.0f,   0.0f, 0.0f, 160.0f, 6.0f },  // 髋偏航：保持站立
    { static_cast<int>(SdkComponentType::LEG_L), 3, 3,  0.35f,  0.0f, 0.0f, 160.0f, 6.0f },  // 膝：轻微屈膝，给前伸留余量
    { static_cast<int>(SdkComponentType::LEG_L), 4, 3, -0.22f,  0.0f, 0.0f,  60.0f, 0.8f },  // 踝俯仰：配合髋/膝，保持脚姿态温和变化
    { static_cast<int>(SdkComponentType::LEG_L), 5, 3,  0.0f,   0.0f, 0.0f,  60.0f, 0.8f },  // 踝横滚：保持站立
};

// --- LEG_R (全局索引 6-11) ------------------------------------------
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
    // 全身示例目标按 controller robot_joint_names / SDK 下发顺序组装。
    for (auto& j : g_leg_l)  cmds.push_back(make_cmd(j));
    for (auto& j : g_leg_r)  cmds.push_back(make_cmd(j));
    for (auto& j : g_waist)  cmds.push_back(make_cmd(j));
    for (auto& j : g_arm_l)  cmds.push_back(make_cmd(j));
    for (auto& j : g_arm_r)  cmds.push_back(make_cmd(j));

#elif defined(CONTROL_COMPONENT)
    // CONTROL_COMPONENT 定义为 SdkComponentType 的整数值。
    #define XX(name, arr) if (CONTROL_COMPONENT == static_cast<int>(SdkComponentType::name)) { for (auto& j : arr) cmds.push_back(make_cmd(j)); }
    XX(ARM_L,  g_arm_l)
    XX(ARM_R,  g_arm_r)
    XX(WAIST,  g_waist)
    XX(LEG_L,  g_leg_l)
    XX(LEG_R,  g_leg_r)
    #undef XX

#elif defined(CONTROL_REPLAY)
    // 轨迹回放模式不使用 build_target_cmds，在 main() 中独立处理
    return cmds;  // 返回空列表，避免和示例目标控制混用

#elif defined(CONTROL_SINGLE)
    // CONTROL_SINGLE 定义为全局索引 0-20
    static JointTarget* all_joints[kTotalJoints] = {
        &g_leg_l[0], &g_leg_l[1], &g_leg_l[2], &g_leg_l[3], &g_leg_l[4], &g_leg_l[5], // 0-5
        &g_leg_r[0], &g_leg_r[1], &g_leg_r[2], &g_leg_r[3], &g_leg_r[4], &g_leg_r[5], // 6-11
        &g_waist[0],                                                                // 12
        &g_arm_l[0], &g_arm_l[1], &g_arm_l[2], &g_arm_l[3],                         // 13-16
        &g_arm_r[0], &g_arm_r[1], &g_arm_r[2], &g_arm_r[3],                         // 17-20
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
        case 6:  return "RL_MIMIC";
        case 10: return "DEBUG";
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
        case static_cast<int>(SdkStateType::DEBUG):
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

static void on_imu_data(const imu_data_lcmt*) {
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
// 轨迹回放 (CONTROL_REPLAY) — 辅助数据结构与函数
// ═══════════════════════════════════════════════════════════════════

struct ReplayJointDesc {
    int component_type;   // SdkComponentType 整数值
    int joint_id;         // 组件内索引
    int txt_col;          // store_ref_motion.txt 中 dof_pos 的列号 (0-44)
};

// 21 关节映射表:
// - 表项顺序按 controller robot_joint_names / SDK 下发顺序排列。
// - txt_col 是 store_ref_motion.txt 中的位置列，依据 ref_motion_fields.yaml。
// - txt_col 不是 SDK 全局关节索引，下发前必须通过这张表重排。
// SDK 下发顺序: LEG_L(6) → LEG_R(6) → WAIST(1) → ARM_L(4) → ARM_R(4)
static const ReplayJointDesc kReplayMapping[21] = {
    // LEG_L (kp/kd idx 0-5)
    {static_cast<int>(SdkComponentType::LEG_L), 0, 12},  // left_hip_pitch
    {static_cast<int>(SdkComponentType::LEG_L), 1, 16},  // left_hip_roll
    {static_cast<int>(SdkComponentType::LEG_L), 2, 14},  // left_hip_yaw
    {static_cast<int>(SdkComponentType::LEG_L), 3, 18},  // left_knee
    {static_cast<int>(SdkComponentType::LEG_L), 4, 20},  // left_ankle_pitch
    {static_cast<int>(SdkComponentType::LEG_L), 5, 22},  // left_ankle_roll
    // LEG_R (kp/kd idx 6-11)
    {static_cast<int>(SdkComponentType::LEG_R), 0, 13},  // right_hip_pitch
    {static_cast<int>(SdkComponentType::LEG_R), 1, 17},  // right_hip_roll
    {static_cast<int>(SdkComponentType::LEG_R), 2, 15},  // right_hip_yaw
    {static_cast<int>(SdkComponentType::LEG_R), 3, 19},  // right_knee
    {static_cast<int>(SdkComponentType::LEG_R), 4, 21},  // right_ankle_pitch
    {static_cast<int>(SdkComponentType::LEG_R), 5, 23},  // right_ankle_roll
    // WAIST (kp/kd idx 12)
    {static_cast<int>(SdkComponentType::WAIST), 0, 3},   // torso_joint
    // ARM_L (kp/kd idx 13-16)
    {static_cast<int>(SdkComponentType::ARM_L), 0, 4},   // left_shoulder_pitch
    {static_cast<int>(SdkComponentType::ARM_L), 1, 6},   // left_shoulder_roll
    {static_cast<int>(SdkComponentType::ARM_L), 2, 8},   // left_shoulder_yaw
    {static_cast<int>(SdkComponentType::ARM_L), 3, 10},  // left_elbow
    // ARM_R (kp/kd idx 17-20)
    {static_cast<int>(SdkComponentType::ARM_R), 0, 5},   // right_shoulder_pitch
    {static_cast<int>(SdkComponentType::ARM_R), 1, 7},   // right_shoulder_roll
    {static_cast<int>(SdkComponentType::ARM_R), 2, 9},   // right_shoulder_yaw
    {static_cast<int>(SdkComponentType::ARM_R), 3, 11},  // right_elbow
};

// 轻量 YAML 解析：只针对 kp_kd.yaml 格式，读取 kps[21]/kds[21]
static bool parseKpKdYaml(const std::string& path, float kp[21], float kd[21]) {
    std::ifstream fin(path);
    if (!fin.is_open()) {
        LOG(ERROR) << "Cannot open kp_kd.yaml: " << path;
        return false;
    }
    int ki = 0, di = 0;
    bool in_kps = false, in_kds = false;
    std::string line;
    while (std::getline(fin, line)) {
        // trim leading whitespace
        const char* s = line.c_str();
        while (*s == ' ' || *s == '\t') ++s;

        if (strncmp(s, "kps:", 4) == 0)  { in_kps = true; in_kds = false; continue; }
        if (strncmp(s, "kds:", 4) == 0)  { in_kds = true; in_kps = false; continue; }
        if (strncmp(s, "scales:", 7) == 0) { in_kps = false; in_kds = false; continue; }

        if (in_kps || in_kds) {
            // 进入下一个 YAML 段时停止读取当前 kps/kds 列表。
            if (s[0] != '-' && *s != '\0' && s[0] != '#') {
                in_kps = false;
                in_kds = false;
                continue;
            }
            if (s[0] == '-') {
                const char* val = s + 1;
                while (*val == ' ') ++val;
                float f = std::strtof(val, nullptr);
                if (in_kps && ki < 21) kp[ki++] = f;
                if (in_kds && di < 21) kd[di++] = f;
                continue;
            }
        }
    }
    if (ki != 21 || di != 21) {
        LOG(ERROR) << "Invalid kp_kd.yaml counts: kps=" << ki << ", kds=" << di
                   << ", expected 21 each. path=" << path;
        return false;
    }
    return true;
}

// 读取 store_ref_motion.txt，每行 45 个 float
static std::vector<std::array<float, kReplayFieldNum>> loadRefMotion(const std::string& path) {
    std::vector<std::array<float, kReplayFieldNum>> frames;
    std::ifstream fin(path);
    if (!fin.is_open()) {
        LOG(ERROR) << "Cannot open motion file: " << path;
        return frames;
    }
    std::string line;
    while (std::getline(fin, line)) {
        if (line.empty()) continue;
        std::array<float, kReplayFieldNum> row{};
        const char* p = line.c_str();
        char* end = nullptr;
        for (int i = 0; i < kReplayFieldNum; ++i) {
            row[i] = std::strtof(p, &end);
            p = end;
        }
        frames.push_back(row);
    }
    LOG(INFO) << "Loaded " << frames.size() << " frames from " << path;
    return frames;
}

static bool parseReplayLoops(int argc, char* argv[], int& loops) {
    loops = kDefaultReplayLoops;
    if (argc < 2) {
        return true;
    }

    char* end = nullptr;
    long value = std::strtol(argv[1], &end, 10);
    if (end == argv[1] || *end != '\0') {
        LOG(ERROR) << "Invalid replay loop count: '" << argv[1]
                   << "'. Use a positive integer, or <=0 for infinite replay.";
        return false;
    }

    loops = static_cast<int>(value);
    return true;
}

#ifndef CONTROL_REPLAY
static bool parseOneshotArgs(int argc, char* argv[], double& hold_seconds) {
    hold_seconds = 0.0;  // <=0 keeps the historical "hold until Ctrl-C" behavior.
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--hold-seconds") == 0) {
            if (i + 1 >= argc) {
                LOG(ERROR) << "--hold-seconds requires a numeric value.";
                return false;
            }
            char* end = nullptr;
            double value = std::strtod(argv[++i], &end);
            if (end == argv[i] || *end != '\0' || value < 0.0) {
                LOG(ERROR) << "Invalid --hold-seconds value: '" << argv[i] << "'";
                return false;
            }
            hold_seconds = value;
        } else {
            LOG(ERROR) << "Unknown argument: '" << argv[i] << "'. Use --help for usage.";
            return false;
        }
    }
    return true;
}
#endif

// 从一帧 ref_motion 构建 21 个 SdkJointCmd
static std::vector<SdkJointCmd> buildReplayCmds(
        const std::array<float, kReplayFieldNum>& frame,
        const float kp[21], const float kd[21]) {
    std::vector<SdkJointCmd> cmds;
    cmds.reserve(21);
    for (int i = 0; i < 21; ++i) {
        SdkJointCmd cmd;
        cmd.component_type = kReplayMapping[i].component_type;
        cmd.joint_id       = kReplayMapping[i].joint_id;
        cmd.ctrlWord       = 3;
        cmd.tarPos         = frame[kReplayMapping[i].txt_col];
        cmd.tarVel         = 0.0f;
        cmd.tarTor         = 0.0f;
        cmd.res1           = kp[i];
        cmd.res2           = kd[i];
        cmds.push_back(cmd);
    }
    return cmds;
}


// ═══════════════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    if (argc > 1 && (std::strcmp(argv[1], "--help") == 0 || std::strcmp(argv[1], "-h") == 0)) {
        std::cout << "Usage: " << argv[0] << " [replay_loops]\n"
                  << "NIX joint command demo. Enters DEBUG and publishes joint_cmds_lcmt.\n"
                  << "Current build-time control source is selected by CONTROL_* macros in example/nix_joint_cmd.cpp.\n"
                  << "Non-replay builds also support: --hold-seconds N (0 means hold until Ctrl-C).\n"
                  << "For day-to-day testing prefer python/nix_joint_cmd.py with --dry-run first.\n";
        return 0;
    }
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

    bool debug_state_entered = false;

#ifndef CONTROL_REPLAY
    // 构建目标指令（由编译期宏选择 CONTROL_ALL/COMPONENT/SINGLE）
    double hold_seconds = 0.0;
    if (!parseOneshotArgs(argc, argv, hold_seconds)) {
        return 2;
    }
    auto cmds = build_target_cmds();
    if (cmds.empty()) {
        LOG(ERROR) << "No joints selected — check CONTROL_xxx macro.";
        return 1;
    }
#endif

#ifdef CONTROL_REPLAY
    std::string motion_path;
    std::string kpkd_path;
    float replay_kp[21] = {};
    float replay_kd[21] = {};
    std::vector<std::array<float, kReplayFieldNum>> replay_frames;
    int replay_loops = kDefaultReplayLoops;
    if (!parseReplayLoops(argc, argv, replay_loops)) {
        return 1;
    }
#endif

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
    if (!g_running) {
        exit_code = 1;
        goto cleanup;
    }
    ensure_reset_briefly(manager);

    // ── STAND ─────────────────────────────────────────────────────
    LOG(INFO) << "=== Step 2: STAND ===";
    if (!g_running) {
        exit_code = 1;
        goto cleanup;
    }
    manager.SendRobotCmd(SdkStateType::STAND);
    if (!wait_state(2, 15)) {
        exit_code = 1;
        goto cleanup;
    }
    LOG(INFO) << "STAND state reported. Waiting " << kStandSettleSec
              << "s for stand interpolation to finish before DEBUG joint control.";
    for (int i = 0; i < kStandSettleSec && g_running; ++i) {
        sleep(1);
    }
    if (!g_running) {
        exit_code = 1;
        goto cleanup;
    }

    // ── 进入 DEBUG 状态 ─────────────────────────────────────────────
    // 当前 lumos_controller 通过 RobotStateType::DEBUG 接收 lcm_joint_cmd。
    LOG(INFO) << "=== Step 3: Enter DEBUG state ===";
    if (!g_running) {
        exit_code = 1;
        goto cleanup;
    }
    manager.SendRobotCmd(SdkStateType::DEBUG);
    debug_state_entered = true;
    sleep(1);

#ifdef CONTROL_REPLAY
    // ── 加载轨迹回放文件 ──────────────────────────────────────
    motion_path = std::string(kReplayPolicyDir) + "/" + kReplayMotionFile;
    kpkd_path   = std::string(kReplayPolicyDir) + "/" + kReplayKpKdFile;

    if (!parseKpKdYaml(kpkd_path, replay_kp, replay_kd)) {
        LOG(ERROR) << "Failed to load kp_kd.yaml";
        exit_code = 1;
        goto cleanup;
    }
    LOG(INFO) << "Loaded kp_kd.yaml: " << kpkd_path;

    replay_frames = loadRefMotion(motion_path);
    if (replay_frames.empty()) {
        LOG(ERROR) << "No frames loaded from " << motion_path;
        exit_code = 1;
        goto cleanup;
    }
#endif

    // ── 下发关节指令 ──────────────────────────────────────────────
    LOG(INFO) << "=== Step 4: Send joint commands ===";

#ifdef CONTROL_REPLAY
    LOG(INFO) << "Replay mode: " << replay_frames.size() << " frames at "
              << kControlHz << " Hz, loops="
              << (replay_loops > 0 ? std::to_string(replay_loops) : std::string("infinite"))
              << ". Press Ctrl-C to stop.";
    {
        size_t frame_idx = 0;
        int completed_loops = 0;
        auto next_tick = std::chrono::steady_clock::now();
        while (g_running && (replay_loops <= 0 || completed_loops < replay_loops)) {
            auto replay_cmds = buildReplayCmds(replay_frames[frame_idx], replay_kp, replay_kd);
            manager.SendJointCmds(replay_cmds);

            frame_idx = (frame_idx + 1) % replay_frames.size();
            if (frame_idx == 0) {
                ++completed_loops;
                LOG(INFO) << "Replay loop " << completed_loops << " complete.";
            }
            next_tick += std::chrono::microseconds(static_cast<int>(kControlDt * 1e6f));
            auto now = std::chrono::steady_clock::now();
            if (next_tick > now) {
                std::this_thread::sleep_until(next_tick);
            } else {
                next_tick = now;
            }
        }
    }

#elif defined(MODE_ONESHOT)
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
            // 平滑插值：起点和终点速度更缓，避免突变。
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
    LOG(INFO) << "Ramp complete. Holding position"
              << (hold_seconds > 0.0 ? " for requested duration." : ". Press Ctrl-C to exit.");
    manager.SendJointCmds(cmds);  // 精确到达目标
    if (hold_seconds > 0.0) {
        auto hold_until = std::chrono::steady_clock::now()
                        + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(hold_seconds));
        while (g_running && std::chrono::steady_clock::now() < hold_until) {
            manager.SendJointCmds(cmds);
            usleep(100000);
        }
    } else {
        while (g_running) { usleep(100000); }
    }

#endif

    // ── 恢复 ──────────────────────────────────────────────────────
    if (!g_running) {
        exit_code = 1;
        goto cleanup;
    }
    LOG(INFO) << "=== Step 5: Return to safe state ===";
    manager.SendRobotCmd(SdkStateType::RESET);
    wait_state(1, 10);
    sleep(3);
    manager.SendRobotCmd(SdkStateType::STAND);
    debug_state_entered = false;
    sleep(1);

cleanup:
    if (!g_running) {
        LOG(WARNING) << "Interrupted, returning to safe state...";
        manager.SendRobotCmd(SdkStateType::STAND);
        sleep(2);
        manager.SendRobotCmd(SdkStateType::RESET);
        sleep(3);
        manager.SendRobotCmd(SdkStateType::STAND);
        debug_state_entered = false;
    } else if (debug_state_entered) {
        LOG(WARNING) << "Leaving DEBUG state after early exit...";
        manager.SendRobotCmd(SdkStateType::STAND);
        sleep(1);
    }
    google::ShutdownGoogleLogging();
    return g_running ? exit_code : 1;
}
