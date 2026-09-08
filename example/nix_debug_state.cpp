#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <iostream>
#include <csignal>
#include <atomic>
#include <cstring>
#include <glog/logging.h>

/**
 * NIX DEBUG 状态 smoke test。
 *
 * 功能:
 *   验证 C++ SDK 能接收 IMU/status，并能按 controller 状态机执行
 *   RESET -> STAND -> DEBUG -> RESET -> STAND。
 *
 * 使用:
 *   cd lumos_sdk
 *   ./build/nix_debug_state --help
 *   ./build/nix_debug_state
 *
 * 注意:
 *   本程序会真实发送状态命令，但不会发送关节目标。日常进入/退出 DEBUG
 *   优先使用 python/nix_debug_state.py，便于设置 timeout 和 dry-run。
 */
static volatile bool g_running = true;
static std::atomic<int> g_robot_state{0};
static std::atomic<int> g_robot_type{0};
static std::atomic<int> g_imu_count{0};
static std::atomic<bool> g_pub_echo{false};

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
    g_robot_type  = msg->type;
    LOG(INFO) << "[STATUS] state=" << state_name(msg->state)
              << " (" << (int)msg->state << ")"
              << " type=" << (int)msg->type
              << " audio=" << msg->audio_file;
}

static bool wait_state(SdkRobotManager& manager, int target, int timeout_s) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
    auto next_retry = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    LOG(INFO) << "Waiting for " << state_name((int8_t)target)
              << " (" << target << ") ... (timeout " << timeout_s << "s)";
    while (g_running && std::chrono::steady_clock::now() < deadline) {
        usleep(100000);  // let recv thread dispatch LCM callbacks
        if (g_robot_state == target) {
            LOG(INFO) << "Reached " << state_name((int8_t)target);
            return true;
        }
        if (std::chrono::steady_clock::now() >= next_retry) {
            LOG(WARNING) << "No " << state_name((int8_t)target)
                         << " confirmation yet; resending state command.";
            manager.SendRobotCmd(static_cast<SdkStateType>(target));
            next_retry += std::chrono::seconds(1);
        }
    }
    LOG(ERROR) << "Timeout waiting for " << state_name((int8_t)target)
               << ", current state is " << state_name((int8_t)g_robot_state.load());
    return false;
}

int main(int argc, char* argv[]) {
    if (argc > 1 && (std::strcmp(argv[1], "--help") == 0 || std::strcmp(argv[1], "-h") == 0)) {
        std::cout << "Usage: " << argv[0] << "\n"
                  << "NIX DEBUG state smoke test. Sends RESET -> STAND -> DEBUG -> RESET -> STAND.\n"
                  << "Use python/nix_debug_state.py for the safer day-to-day DEBUG entry.\n";
        return 0;
    }
    signal(SIGINT, sigint_handler);

    FLAGS_stderrthreshold = 0;
    FLAGS_minloglevel = 0;
    google::InitGoogleLogging(argv[0]);

    int exit_code = 0;
    SdkRobotManager manager;
    manager.Init();
    manager.SetRobotStatusCb(on_robot_status);

    // Diagnostic: also subscribe to lcm_imu_data to verify LCM receive works.
    manager.SetImuDataCb([](const imu_data_lcmt*) { g_imu_count++; });
    LOG(INFO) << "LCM initialized, waiting for lcm_imu_data...";
    for (int i = 0; i < 30 && g_imu_count == 0; i++) usleep(100000);
    LOG(INFO) << "IMU messages received in 3s: " << g_imu_count.load();

    // Wait for first robot status
    for (int i = 0; i < 30 && g_robot_state == 0; i++) usleep(100000);
    LOG(INFO) << "Current robot state after 3s wait: " << state_name((int8_t)g_robot_state.load());

    // Self-echo: subscribe to our own publish channels on same lcm_ object
    // via a raw function pointer (SetGameHandlerCmdCb subscribes lcm_robot_cmd_echo)
    manager.SetGameHandlerCmdCb([](const robot_cmd_lcmt* msg) {
        g_pub_echo = true;
        LOG(INFO) << "[ECHO] Received cmd echo (via lcm_robot_cmd_echo) state="
                  << (int)msg->state << " vx=" << msg->x;
    });
    LOG(INFO) << "Echo subscription (lcm_robot_cmd_echo) setup done";

    // ================================================================
    // Step 1: RESET state (enter from NOT_A_STATE)
    // ================================================================
    LOG(INFO) << "=== Step 1: Transition to RESET ===";
    manager.SendRobotCmd(SdkStateType::RESET);
    if (!wait_state(manager, 1, 10)) { exit_code = 1; goto cleanup; }
    sleep(2);

    // ================================================================
    // Step 2: STAND state
    // ================================================================
    LOG(INFO) << "=== Step 2: Transition to STAND ===";
    if (!g_running) { exit_code = 1; goto cleanup; }
    manager.SendRobotCmd(SdkStateType::STAND);
    if (!wait_state(manager, 2, 10)) { exit_code = 1; goto cleanup; }
    sleep(10);

    // ================================================================
    // Step 3: DEBUG state, current controller joint-command entry
    // ================================================================
    LOG(INFO) << "=== Step 3: Enter DEBUG state ===";
    manager.SendRobotCmd(SdkStateType::DEBUG);
    if (!wait_state(manager, 10, 10)) { exit_code = 1; goto cleanup; }
    sleep(1);
    LOG(INFO) << "Echo check: pub_echo=" << g_pub_echo.load();

    // ================================================================
    // Step 4: RESET before leaving DEBUG
    // ================================================================
    LOG(INFO) << "=== Step 4: Return to RESET ===";
    if (!g_running) { exit_code = 1; goto cleanup; }
    manager.SendRobotCmd(SdkStateType::RESET);
    if (!wait_state(manager, 1, 10)) { exit_code = 1; goto cleanup; }
    sleep(2);

    // ================================================================
    // Step 5: Back to STAND
    // ================================================================
    LOG(INFO) << "=== Step 5: Return to STAND ===";
    if (!g_running) { exit_code = 1; goto cleanup; }
    manager.SendRobotCmd(SdkStateType::STAND);
    if (!wait_state(manager, 2, 10)) { exit_code = 1; goto cleanup; }
    sleep(1);

    if (!g_pub_echo.load()) {
        LOG(WARNING) << "No lcm_robot_cmd_echo received; controller may not publish this optional channel.";
    }
    LOG(INFO) << "=== NIX DEBUG state test SUCCESS ===";

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
