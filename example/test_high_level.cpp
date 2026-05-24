#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <iostream>
#include <csignal>
#include <glog/logging.h>

static volatile bool g_running = true;

static void sigint_handler(int) { g_running = false; }

static void wait_enter(const char* prompt) {
    if (!g_running) return;
    std::cout << "\n[WAIT] " << prompt << " (press ENTER to continue, Ctrl+C to quit)" << std::endl;
    std::string line;
    std::getline(std::cin, line);
}

static void wait_enter_or_skip(const char* prompt, int timeout_sec) {
    if (!g_running) return;
    std::cout << "\n[WAIT] " << prompt << " (ENTER to skip wait, Ctrl+C to quit)" << std::endl;
    std::cout << "       auto-continue in " << timeout_sec << "s..." << std::endl;
    // Simple non-blocking wait: the user can press ENTER to skip
    std::string line;
    std::getline(std::cin, line);
}

int main(int argc, char* argv[]) {
    (void)(argc);
    signal(SIGINT, sigint_handler);

    FLAGS_stderrthreshold = 0;
    FLAGS_minloglevel = 0;
    google::InitGoogleLogging(argv[0]);

    SdkRobotManager manager;
    manager.Init();
    LOG(INFO) << "SDK initialized, waiting for LCM to be ready...";
    sleep(3);

    // ================================================================
    // Phase 1: RESET
    // ================================================================
    LOG(INFO) << "=== Phase 1: Enter RESET state ===";
    std::cout << "\n>>> Make sure the gamepad/handle is DISCONNECTED <<<\n" << std::endl;
    wait_enter("Ready to enter RESET state?");
    if (!g_running) goto cleanup;

    manager.SendRobotCmd(SdkStateType::RESET);
    LOG(INFO) << "RESET command sent, waiting 8s...";
    sleep(8);

    if (!g_running) goto cleanup;

    // ================================================================
    // Phase 2: STAND
    // ================================================================
    LOG(INFO) << "=== Phase 2: Enter STAND state ===";
    wait_enter("Ready to stand up?");
    if (!g_running) goto cleanup;

    manager.SendRobotCmd(SdkStateType::STAND);
    LOG(INFO) << "STAND command sent, waiting 15s for robot to stabilize...";
    sleep(15);

    if (!g_running) goto cleanup;

    // ================================================================
    // Phase 3: RL_WALK + velocity tests
    // ================================================================
    LOG(INFO) << "=== Phase 3: RL_WALK + velocity commands ===";

    // 3a: Enter walk mode
    wait_enter("Ready to enter RL_WALK mode?");
    if (!g_running) goto cleanup;
    manager.SendRobotCmd(SdkStateType::RL_WALK);
    sleep(5);

    // 3b: Walk forward at 0.1 m/s (slow, safe)
    wait_enter("Walk FORWARD at 0.1 m/s for 5s?");
    if (!g_running) goto cleanup;
    LOG(INFO) << "Walking forward at vx=0.1 m/s";
    manager.SendRobotCmd(SdkStateType::RL_WALK, 0.1f);
    sleep(5);

    // 3c: Walk backward at 0.1 m/s
    wait_enter("Walk BACKWARD at 0.1 m/s for 5s?");
    if (!g_running) goto cleanup;
    LOG(INFO) << "Walking backward at vx=-0.1 m/s";
    manager.SendRobotCmd(SdkStateType::RL_WALK, -0.1f);
    sleep(5);

    // 3d: Strafe left
    wait_enter("Strafe LEFT at 0.1 m/s for 5s?");
    if (!g_running) goto cleanup;
    LOG(INFO) << "Strafing left at vy=0.1 m/s";
    manager.SendRobotCmd(SdkStateType::RL_WALK, 0.0f, 0.1f);
    sleep(5);

    // 3e: Strafe right
    wait_enter("Strafe RIGHT at 0.1 m/s for 5s?");
    if (!g_running) goto cleanup;
    LOG(INFO) << "Strafing right at vy=-0.1 m/s";
    manager.SendRobotCmd(SdkStateType::RL_WALK, 0.0f, -0.1f);
    sleep(5);

    // 3f: Turn left
    wait_enter("Turn LEFT at 0.2 rad/s for 5s?");
    if (!g_running) goto cleanup;
    LOG(INFO) << "Turning left at vyaw=0.2 rad/s";
    manager.SendRobotCmd(SdkStateType::RL_WALK, 0.0f, 0.0f, 0.2f);
    sleep(5);

    // 3g: Turn right
    wait_enter("Turn RIGHT at 0.2 rad/s for 5s?");
    if (!g_running) goto cleanup;
    LOG(INFO) << "Turning right at vyaw=-0.2 rad/s";
    manager.SendRobotCmd(SdkStateType::RL_WALK, 0.0f, 0.0f, -0.2f);
    sleep(5);

    // 3h: Combined motion (forward + turn)
    wait_enter("Combined: forward 0.1 m/s + turn 0.1 rad/s for 5s?");
    if (!g_running) goto cleanup;
    LOG(INFO) << "Combined motion: vx=0.1, vyaw=0.1";
    manager.SendRobotCmd(SdkStateType::RL_WALK, 0.1f, 0.0f, 0.1f);
    sleep(5);

    // 3i: Stop
    LOG(INFO) << "Stopping...";
    manager.SendRobotCmd(SdkStateType::RL_WALK, 0.0f, 0.0f, 0.0f);
    sleep(2);

    if (!g_running) goto cleanup;

    // ================================================================
    // Phase 4: Teardown
    // ================================================================
    LOG(INFO) << "=== Phase 4: Teardown ===";

    wait_enter("Return to STAND?");
    if (!g_running) goto cleanup;
    manager.SendRobotCmd(SdkStateType::STAND);
    sleep(8);

    wait_enter("Return to RESET (disengage motors)?");
    if (!g_running) goto cleanup;
    manager.SendRobotCmd(SdkStateType::RESET);
    sleep(3);

cleanup:
    LOG(INFO) << "=== High-level control test complete ===";
    google::ShutdownGoogleLogging();
    return 0;
}
