#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <chrono>
#include <cstring>
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
 *   ./build/nix_robot_state --walk-test   # 显式低速行走测试
 *
 * 注意:
 *   --walk-test 会发送 RL_WALK 速度命令，只能在确认机器人安全站立、
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
        default: return "UNKNOWN";
    }
}

static void on_robot_status(const robot_status_lcmt* msg) {
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
        if (g_robot_state == target) {
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

int main(int argc, char* argv[]) {
    bool walk_test = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--walk-test") == 0) {
            walk_test = true;
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            std::cout << "Usage: " << argv[0] << " [--walk-test]\n"
                      << "Default: RESET -> STAND smoke test only.\n"
                      << "--walk-test: additionally enter RL_WALK and send low-speed velocity commands.\n";
            return 0;
        } else {
            std::cerr << "Unknown argument: " << argv[i] << std::endl;
            return 2;
        }
    }
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    FLAGS_stderrthreshold = 0;
    FLAGS_minloglevel = 0;
    google::InitGoogleLogging(argv[0]);

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

    // ================================================================
    // Step 1: RESET
    // ================================================================
    LOG(INFO) << "=== Step 1: RESET ===";
    if (!g_running) goto cleanup;
    manager.SendRobotCmd(SdkStateType::RESET);
    if (!wait_state(1, 15)) goto cleanup;
    sleep(3);
    print_rate_stats();

    // ================================================================
    // Step 2: STAND
    // ================================================================
    LOG(INFO) << "=== Step 2: STAND ===";
    if (!g_running) goto cleanup;
    manager.SendRobotCmd(SdkStateType::STAND);
    if (!wait_state(2, 15)) goto cleanup;
    sleep(5);
    print_rate_stats();

    if (walk_test) {
        // ================================================================
        // Optional Step 3: RL_WALK
        // ================================================================
        LOG(INFO) << "=== Optional Step 3: RL_WALK (forward 0.05 m/s, 5s) ===";
        if (!g_running) goto cleanup;
        manager.SendRobotCmd(SdkStateType::RL_WALK, 0.05f);
        if (!wait_state(3, 10)) goto cleanup;
        print_rate_stats();

        // Walk for 5 seconds with velocity control.
        for (int t = 0; t < 5 && g_running; t++) {
            float vx = (t < 2) ? 0.05f : ((t < 4) ? 0.1f : 0.0f);
            manager.SendRobotCmd(SdkStateType::RL_WALK, vx);
            sleep(1);
        }

        manager.SendRobotCmd(SdkStateType::RL_WALK, 0.0f);
        sleep(2);

        // RL_WALK -> RESET is valid, STAND is not in RL_WALK transitions.
        LOG(INFO) << "=== Optional Step 4: RESET ===";
        if (!g_running) goto cleanup;
        manager.SendRobotCmd(SdkStateType::RESET);
        if (!wait_state(1, 15)) goto cleanup;
        sleep(3);
        print_rate_stats();
    } else {
        LOG(INFO) << "Skipping RL_WALK. Pass --walk-test to run velocity command test.";
    }

    LOG(INFO) << "=== Robot state command test SUCCESS ===";

cleanup:
    if (!g_running) {
        LOG(WARNING) << "Interrupted, sending STAND...";
        manager.SendRobotCmd(SdkStateType::STAND);
        sleep(5);
    }
    google::ShutdownGoogleLogging();
    return g_running ? 0 : 1;
}
