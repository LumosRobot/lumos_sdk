#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <glog/logging.h>

/**
 * NIX 状态机 smoke test。
 *
 * 功能:
 *   默认验证 C++ SDK 的 RESET -> STAND 状态命令和 IMU/status 接收。
 *   默认不会进入 RL_WALK，也不会发送关节目标。
 *
 * 使用:
 *   cd lumos_sdk
 *   ./build/nix_robot_state --help
 *   ./build/nix_robot_state
 *   ./build/nix_robot_state --walk-test   # 显式行走测试
 *
 * 注意:
 *   --walk-test 会发送死区外的 RL_WALK 速度命令，只能在确认机器人安全站立、
 *   周围空间和急停可用时运行。
 */
static volatile bool g_running = true;
static std::atomic<int> g_robot_state{0};
static std::atomic<int> g_joint_count{0};
static std::atomic<int> g_imu_count{0};

static void sigint_handler(int) { g_running = false; }

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
        case 0:
        case 1:
        case 2:
        case 3:
        case 5:
        case 6:
        case 10:
        case 11:
        case 12:
        case 20:
        case 21:
            return true;
        default:
            return false;
    }
}

static void on_robot_status(const robot_status_lcmt* msg) {
    if (!is_main_robot_state(msg->state)) {
        LOG_EVERY_N(WARNING, 50) << "Ignore non-main robot_status state="
                                 << static_cast<int>(msg->state)
                                 << ", type=" << static_cast<int>(msg->type);
        return;
    }
    g_robot_state = msg->state;
}

static void on_joint_data(const joint_datasets_lcmt* /*msg*/) {
    g_joint_count++;
}

static void on_imu_data(const imu_data_lcmt* /*msg*/) {
    g_imu_count++;
}

static bool wait_state(int target, int timeout_s) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
    LOG(INFO) << "Waiting for " << state_name((int8_t)target)
              << " (" << target << ") ... timeout=" << timeout_s << "s";
    while (g_running && std::chrono::steady_clock::now() < deadline) {
        usleep(50000);
        const int current = g_robot_state.load();
        if (current == target) {
            LOG(INFO) << "Reached " << state_name((int8_t)target);
            return true;
        }
    }
    LOG(ERROR) << "Timeout waiting for " << state_name((int8_t)target)
               << ", current=" << state_name((int8_t)g_robot_state.load());
    return false;
}

static void print_rate_stats() {
    // Sample for 2 seconds.
    g_joint_count = 0;
    g_imu_count = 0;
    sleep(2);
    int jc = g_joint_count.load();
    int ic = g_imu_count.load();
    std::cout << "[RATE] lcm_joint_data: " << (jc / 2.0) << " Hz, lcm_imu_data: " << (ic / 2.0) << " Hz" << std::endl;
}

static bool is_known_state(int state) {
    switch (state) {
        case 1:   // RESET
        case 2:   // STAND
        case 3:   // RL_WALK
        case 5:   // RL_LIEDOWN
        case 6:   // RL_MIMIC
        case 10:  // DEBUG
        case 11:  // RL_NAV
        case 12:  // RL_WALK_AMP
        case 20:  // BY_MIMIC
        case 21:  // BFM_MIMIC
            return true;
        default:
            return false;
    }
}

static bool parse_state(const char* value, int& state) {
    const std::string raw(value);
    if (raw == "RESET") state = 1;
    else if (raw == "STAND") state = 2;
    else if (raw == "RL_WALK") state = 3;
    else if (raw == "RL_LIEDOWN") state = 5;
    else if (raw == "RL_MIMIC") state = 6;
    else if (raw == "DEBUG") state = 10;
    else if (raw == "RL_NAV") state = 11;
    else if (raw == "RL_WALK_AMP") state = 12;
    else if (raw == "BY_MIMIC") state = 20;
    else if (raw == "BFM_MIMIC") state = 21;
    else {
        char* end = nullptr;
        const long parsed = std::strtol(value, &end, 10);
        if (end == value || *end != '\0' || !is_known_state(static_cast<int>(parsed))) {
            return false;
        }
        state = static_cast<int>(parsed);
    }
    return true;
}

int main(int argc, char* argv[]) {
    bool walk_test = false;
    int requested_state = 0;
    if (argc > 1 && (std::strcmp(argv[1], "--help") == 0 || std::strcmp(argv[1], "-h") == 0)) {
        std::cout << "Usage: " << argv[0] << " [--walk-test]\n"
                      << "       " << argv[0] << " state STATE\n"
                      << "Default: RESET -> STAND smoke test only.\n"
                      << "--walk-test: enter RL_WALK with zero velocity, then send 0.25 m/s.\n"
                      << "state STATE: accept a state name or ID, publish zero velocity, and wait for confirmation.\n"
                      << "IDs: RESET=1 STAND=2 RL_WALK=3 RL_LIEDOWN=5 RL_MIMIC=6 "
                         "DEBUG=10 RL_NAV=11 RL_WALK_AMP=12 BY_MIMIC=20 BFM_MIMIC=21.\n";
        return 0;
    } else if (argc == 2 && std::strcmp(argv[1], "--walk-test") == 0) {
        walk_test = true;
    } else if (argc == 3 && std::strcmp(argv[1], "state") == 0) {
        if (!parse_state(argv[2], requested_state)) {
            std::cerr << "Unknown state name or ID: " << argv[2] << std::endl;
            return 2;
        }
    } else if (argc != 1) {
        std::cerr << "Invalid arguments. Use --help for usage." << std::endl;
        return 2;
    }
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    FLAGS_stderrthreshold = 0;
    FLAGS_minloglevel = 0;
    google::InitGoogleLogging(argv[0]);

    int exit_code = 0;
    SdkRobotManager manager;
    manager.Init();
    manager.SetRobotStatusCb(on_robot_status);
    manager.SetJointDataCb(on_joint_data);
    manager.SetImuDataCb(on_imu_data);

    // Wait for LCM to warm up and first status to arrive
    LOG(INFO) << "Waiting for LCM data...";
    for (int i = 0; i < 30 && g_robot_state == 0; i++) usleep(100000);
    LOG(INFO) << "Initial state: " << state_name((int8_t)g_robot_state.load());

    if (g_robot_state == 0) {
        LOG(WARNING) << "No robot status received, LCM may not be configured. Continuing anyway...";
    }

    if (requested_state != 0) {
        LOG(INFO) << "Sending single state command: "
                  << state_name(static_cast<int8_t>(requested_state))
                  << "(" << requested_state << ")";
        manager.SendRobotCmd(static_cast<SdkStateType>(requested_state));
        if (!wait_state(requested_state, 15)) {
            exit_code = 1;
            goto cleanup;
        }
        LOG(INFO) << "=== Single state command test SUCCESS ===";
        goto cleanup;
    }

    // ================================================================
    // Step 1: RESET
    // ================================================================
    LOG(INFO) << "=== Step 1: RESET ===";
    if (!g_running) { exit_code = 1; goto cleanup; }
    manager.SendRobotCmd(SdkStateType::RESET);
    if (!wait_state(1, 15)) { exit_code = 1; goto cleanup; }
    sleep(3);
    print_rate_stats();

    // ================================================================
    // Step 2: STAND
    // ================================================================
    LOG(INFO) << "=== Step 2: STAND ===";
    if (!g_running) { exit_code = 1; goto cleanup; }
    manager.SendRobotCmd(SdkStateType::STAND);
    if (!wait_state(2, 15)) { exit_code = 1; goto cleanup; }
    // controller 的 StandState 插值约 10 秒，留 1 秒余量后再进入 RL 策略。
    sleep(11);
    print_rate_stats();

    if (walk_test) {
        // ================================================================
        // Optional Step 3: RL_WALK
        // ================================================================
        LOG(INFO) << "=== Optional Step 3: RL_WALK (forward 0.25 m/s, 5s) ===";
        if (!g_running) { exit_code = 1; goto cleanup; }
        // 带速度的 robot_cmd 只作为速度命令，不触发状态切换。
        manager.SendRobotCmd(SdkStateType::RL_WALK);
        if (!wait_state(3, 10)) { exit_code = 1; goto cleanup; }
        print_rate_stats();

        // vx/vy 的 [-0.2, 0.2] 是 controller 死区，使用 0.25 m/s。
        for (int t = 0; t < 5 && g_running; t++) {
            manager.SendRobotCmd(SdkStateType::RL_WALK, t < 4 ? 0.25f : 0.0f);
            sleep(1);
        }

        manager.SendRobotCmd(SdkStateType::RL_WALK, 0.0f);
        sleep(2);

        // RL_WALK -> RESET is valid, STAND is not in RL_WALK transitions.
        LOG(INFO) << "=== Optional Step 4: RESET ===";
        if (!g_running) { exit_code = 1; goto cleanup; }
        manager.SendRobotCmd(SdkStateType::RESET);
        if (!wait_state(1, 15)) { exit_code = 1; goto cleanup; }
        sleep(3);
        print_rate_stats();
    } else {
        LOG(INFO) << "Skipping RL_WALK. Pass --walk-test to run velocity command test.";
    }

    LOG(INFO) << "=== Robot state command test SUCCESS ===";

cleanup:
    if (!g_running || exit_code != 0) {
        LOG(WARNING) << "Test interrupted or failed; requesting RESET -> STAND recovery.";
        manager.SendRobotCmd(SdkStateType::RESET);
        sleep(3);
        manager.SendRobotCmd(SdkStateType::STAND);
    }
    google::ShutdownGoogleLogging();
    return g_running ? exit_code : 1;
}
