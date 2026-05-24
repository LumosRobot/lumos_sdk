#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <iostream>
#include <csignal>
#include <atomic>
#include <glog/logging.h>

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

static bool wait_state(SdkRobotManager& /*mgr*/, int target, int timeout_s) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
    LOG(INFO) << "Waiting for " << state_name((int8_t)target)
              << " (" << target << ") ... (timeout " << timeout_s << "s)";
    while (g_running && std::chrono::steady_clock::now() < deadline) {
        usleep(100000);  // let recv thread dispatch LCM callbacks
        if (g_robot_state == target) {
            LOG(INFO) << "Reached " << state_name((int8_t)target);
            return true;
        }
    }
    LOG(ERROR) << "Timeout waiting for " << state_name((int8_t)target)
               << ", current state is " << state_name((int8_t)g_robot_state.load());
    return false;
}

int main(int argc, char* argv[]) {
    (void)(argc);
    signal(SIGINT, sigint_handler);

    FLAGS_stderrthreshold = 0;
    FLAGS_minloglevel = 0;
    google::InitGoogleLogging(argv[0]);

    SdkRobotManager manager;
    manager.Init();
    manager.SetRobotStatusCb(on_robot_status);

    // Diagnostic: also subscribe to myIMU to verify LCM receive works
    manager.SetImuDataCb([](const microstrain_lcmt*) { g_imu_count++; });
    LOG(INFO) << "SDK initialized, waiting for LCM (checking IMU)...";
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
    // Step 1: Enter SDK control mode
    // ================================================================
    LOG(INFO) << "=== Step 1: Enter SDK control mode ===";
    manager.SendModeCmd(1);
    sleep(1);
    LOG(INFO) << "Echo check: pub_echo=" << g_pub_echo.load();

    // ================================================================
    // Step 2: RESET state (enter from NOT_A_STATE)
    // ================================================================
    LOG(INFO) << "=== Step 2: Transition to RESET ===";
    manager.SendRobotCmd(SdkStateType::RESET);
    if (!wait_state(manager, 1, 10)) goto cleanup;
    sleep(1);

    // ================================================================
    // Step 3: STAND state
    // ================================================================
    LOG(INFO) << "=== Step 3: Transition to STAND ===";
    if (!g_running) goto cleanup;
    manager.SendRobotCmd(SdkStateType::STAND);
    if (!wait_state(manager, 2, 10)) goto cleanup;
    sleep(2);

    // ================================================================
    // Step 4: RL_WALK_AMP - walk forward at 0.1 m/s for 3s
    // ================================================================
    LOG(INFO) << "=== Step 4: Transition to RL_WALK_AMP (forward 0.1m/s) ===";
    if (!g_running) goto cleanup;
    manager.SendRobotCmd(SdkStateType::RL_WALK_AMP, 0.1f);
    if (!wait_state(manager, 12, 10)) goto cleanup;
    sleep(3);

    // Stop walking
    manager.SendRobotCmd(SdkStateType::RL_WALK_AMP, 0.0f);
    sleep(1);

    // ================================================================
    // Step 5: Back to RESET
    // ================================================================
    LOG(INFO) << "=== Step 5: Return to RESET ===";
    if (!g_running) goto cleanup;
    manager.SendRobotCmd(SdkStateType::RESET);
    if (!wait_state(manager, 1, 10)) goto cleanup;
    sleep(1);

    // ================================================================
    // Step 6: Back to STAND then exit SDK mode
    // ================================================================
    LOG(INFO) << "=== Step 6: Return to STAND ===";
    if (!g_running) goto cleanup;
    manager.SendRobotCmd(SdkStateType::STAND);
    if (!wait_state(manager, 2, 10)) goto cleanup;
    sleep(1);

    manager.SendModeCmd(0);
    sleep(1);

    LOG(INFO) << "=== SDK mode test SUCCESS ===";

cleanup:
    if (!g_running) {
        manager.SendRobotCmd(SdkStateType::STAND);
        sleep(2);
        manager.SendModeCmd(0);
        LOG(WARNING) << "Test interrupted by user.";
    }
    google::ShutdownGoogleLogging();
    return g_running ? 0 : 1;
}
