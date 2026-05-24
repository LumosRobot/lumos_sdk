#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <iostream>
#include <csignal>
#include <iomanip>
#include <cmath>
#include <glog/logging.h>
#include "sdk_lcmt_joint_datasets.hpp"

static volatile bool g_running = true;
static void sigint_handler(int) { g_running = false; }

static void wait_enter(const char* prompt) {
    if (!g_running) return;
    std::cout << "\n[WAIT] " << prompt << " (ENTER to continue, Ctrl+C to quit)" << std::endl;
    std::string line;
    std::getline(std::cin, line);
}

// ── Joint feedback callback ──────────────────────────────────────
static void joint_data_handler(const sdk_lcmt_joint_datasets* data) {
    static uint64_t counter = 0;
    if (++counter % 50 == 0) {
        std::cout << "\n[JOINT FEEDBACK #" << counter << "] " << data->datasets.size() << " joints" << std::endl;
        for (size_t i = 0; i < data->datasets.size() && i < 6; ++i) {
            auto& d = data->datasets[i];
            std::cout << "  [" << i << "] comp=" << d.component_type
                      << " id=" << d.joint_id
                      << " pos=" << std::fixed << std::setprecision(3) << d.pos_high
                      << " vel=" << d.vel
                      << " cur=" << d.cur << std::endl;
        }
        if (data->datasets.size() > 6) {
            std::cout << "  ... ("
                      << (data->datasets.size() - 6) << " more joints)" << std::endl;
        }
    }
}

// ── Joint parameter definitions (NIX2 platform) ──────────────────
struct JointParam { float lower; float upper; float kp; float kd; };

static const JointParam s_joint_params[] = {
    { -3.0,   3.0,   60, 2 }, // 0: torso/waist
    { -1.3,   1.3,   60, 2 }, // 1: head
    { -3.0,   3.0,   60, 2 }, // 2: L shoulder pitch
    { -0.12,  2.3,   60, 2 }, // 3: L shoulder roll
    { -1.9,   1.9,   60, 2 }, // 4: L shoulder yaw
    { -0.83,  1.57,  60, 2 }, // 5: L elbow
    { -2.3,   2.3,   60, 2 }, // 6: L wrist yaw
    { -1.2,   1.2,   60, 2 }, // 7: L wrist pitch
    { -1.2,   1.2,   60, 2 }, // 8: L wrist roll
    { -3.0,   3.0,   60, 2 }, // 9: R shoulder pitch
    { -2.3,   0.12,  60, 2 }, // 10: R shoulder roll
    { -1.9,   1.9,   60, 2 }, // 11: R shoulder yaw
    { -0.83,  1.57,  60, 2 }, // 12: R elbow
    { -2.3,   2.3,   60, 2 }, // 13: R wrist yaw
    { -1.2,   1.2,   60, 2 }, // 14: R wrist pitch
    { -1.2,   1.2,   60, 2 }, // 15: R wrist roll
    { -1.5,   1.5,  200, 6 }, // 16: L hip pitch
    { -0.25,  2.8,  200, 6 }, // 17: L hip roll
    { -1.2,   2.6,  200, 6 }, // 18: L hip yaw
    {  0.0,   2.27, 200, 6 }, // 19: L knee
    { -1.0,   0.43,  80, 2 }, // 20: L ankle pitch
    { -0.43,  0.43,  80, 2 }, // 21: L ankle roll
    { -1.5,   1.5,  200, 6 }, // 22: R hip pitch
    { -2.8,   0.25, 200, 6 }, // 23: R hip roll
    { -2.6,   1.2,  200, 6 }, // 24: R hip yaw
    {  0.0,   2.27, 200, 6 }, // 25: R knee
    { -1.0,   0.43,  80, 2 }, // 26: R ankle pitch
    { -0.43,  0.43,  80, 2 }, // 27: R ankle roll
};

#include <map>
static const std::map<SdkComponentType, std::vector<int>> s_component_map = {
    { SdkComponentType::WAIST, { 0 } },
    { SdkComponentType::HEAD,  { 1 } },
    { SdkComponentType::ARM_L, { 2, 3, 4, 5, 6, 7, 8 } },
    { SdkComponentType::ARM_R, { 9, 10, 11, 12, 13, 14, 15 } },
    { SdkComponentType::LEG_L, { 16, 17, 18, 19, 20, 21 } },
    { SdkComponentType::LEG_R, { 22, 23, 24, 25, 26, 27 } },
};

static SdkJointCmd make_joint_cmd(SdkComponentType comp, int local_id,
                                   float pos, float vel = 0, float tor = 0) {
    SdkJointCmd cmd;
    auto it = s_component_map.find(comp);
    if (it == s_component_map.end() || local_id >= (int)it->second.size()) {
        LOG(ERROR) << "Invalid joint: comp=" << (int)comp << " id=" << local_id;
        return cmd;
    }
    int global_id = it->second[local_id];
    auto& p = s_joint_params[global_id];

    cmd.component_type = static_cast<int16_t>(comp);
    cmd.joint_id = local_id;
    cmd.ctrlWord = 3;        // position control
    cmd.tarPos = pos;
    cmd.tarVel = vel;
    cmd.tarTor = tor;
    cmd.res1 = p.kp;
    cmd.res2 = p.kd;
    return cmd;
}

static void send_and_hold(SdkRobotManager& mgr,
                           const std::vector<SdkJointCmd>& cmds,
                           int hold_sec) {
    if (!g_running) return;
    LOG(INFO) << "Sending " << cmds.size() << " joint commands, hold " << hold_sec << "s";
    mgr.SendJointCmds(cmds);
    sleep(hold_sec);
}

// ── Main ─────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    (void)(argc);
    signal(SIGINT, sigint_handler);

    FLAGS_stderrthreshold = 0;
    FLAGS_minloglevel = 0;
    google::InitGoogleLogging(argv[0]);

    SdkRobotManager manager;
    manager.Init();
    sleep(3);

    std::cout << "\n==================================================" << std::endl;
    std::cout << "  Joint-Level SDK Control Test (Gamepad Required)" << std::endl;
    std::cout << "==================================================" << std::endl;

    // ── Step 0: Gamepad authorization + SDK mode ─────────────────
    wait_enter("Step 0a: Press Home+RB on gamepad to enter SDK JointLevel mode");
    if (!g_running) goto cleanup;

    wait_enter("Step 0b: Send SendModeCmd(1) to enable SDK control?");
    if (!g_running) goto cleanup;
    manager.SendModeCmd(1);
    LOG(INFO) << "Switched to SDK control mode";
    sleep(2);

    // ── Step 1: Ensure STAND ─────────────────────────────────────
    wait_enter("Step 1: Enter STAND state (if not already)");
    if (!g_running) goto cleanup;
    manager.SendRobotCmd(SdkStateType::STAND);
    sleep(5);
    if (!g_running) goto cleanup;

    // Register joint feedback
    manager.SetJointDataCb(joint_data_handler);
    LOG(INFO) << "Joint feedback callback registered";

    // ── Step 2: Single joint test - right knee ───────────────────
    {
        wait_enter("Step 2a: Move RIGHT KNEE to ~45 deg (0.785 rad), hold 3s");
        if (!g_running) goto cleanup;
        std::vector<SdkJointCmd> cmds;
        cmds.push_back(make_joint_cmd(SdkComponentType::LEG_R, 3, 0.785f));
        send_and_hold(manager, cmds, 3);

        wait_enter("Step 2b: Return RIGHT KNEE to 0");
        if (!g_running) goto cleanup;
        cmds.clear();
        cmds.push_back(make_joint_cmd(SdkComponentType::LEG_R, 3, 0.0f));
        send_and_hold(manager, cmds, 3);
    }

    // ── Step 3: Left knee test ───────────────────────────────────
    {
        wait_enter("Step 3a: Move LEFT KNEE to ~45 deg (0.785 rad), hold 3s");
        if (!g_running) goto cleanup;
        std::vector<SdkJointCmd> cmds;
        cmds.push_back(make_joint_cmd(SdkComponentType::LEG_L, 3, 0.785f));
        send_and_hold(manager, cmds, 3);

        wait_enter("Step 3b: Return LEFT KNEE to 0");
        if (!g_running) goto cleanup;
        cmds.clear();
        cmds.push_back(make_joint_cmd(SdkComponentType::LEG_L, 3, 0.0f));
        send_and_hold(manager, cmds, 3);
    }

    // ── Step 4: Right leg multi-joint ────────────────────────────
    {
        wait_enter("Step 4: Right leg squat (hip pitch + knee), hold 3s");
        if (!g_running) goto cleanup;
        std::vector<SdkJointCmd> cmds;
        cmds.push_back(make_joint_cmd(SdkComponentType::LEG_R, 0, -0.3f));
        cmds.push_back(make_joint_cmd(SdkComponentType::LEG_R, 3,  0.5f));
        cmds.push_back(make_joint_cmd(SdkComponentType::LEG_R, 4, -0.2f));
        send_and_hold(manager, cmds, 3);

        wait_enter("Step 4b: Return right leg to zero");
        if (!g_running) goto cleanup;
        cmds.clear();
        cmds.push_back(make_joint_cmd(SdkComponentType::LEG_R, 0, 0.0f));
        cmds.push_back(make_joint_cmd(SdkComponentType::LEG_R, 3, 0.0f));
        cmds.push_back(make_joint_cmd(SdkComponentType::LEG_R, 4, 0.0f));
        send_and_hold(manager, cmds, 3);
    }

    // ── Step 5: Left arm ─────────────────────────────────────────
    {
        wait_enter("Step 5: Left arm raise shoulder + bend elbow, hold 3s");
        if (!g_running) goto cleanup;
        std::vector<SdkJointCmd> cmds;
        cmds.push_back(make_joint_cmd(SdkComponentType::ARM_L, 0,  0.8f));
        cmds.push_back(make_joint_cmd(SdkComponentType::ARM_L, 1,  1.0f));
        cmds.push_back(make_joint_cmd(SdkComponentType::ARM_L, 3,  0.4f));
        send_and_hold(manager, cmds, 3);

        wait_enter("Step 5b: Return left arm to zero");
        if (!g_running) goto cleanup;
        cmds.clear();
        cmds.push_back(make_joint_cmd(SdkComponentType::ARM_L, 0, 0.0f));
        cmds.push_back(make_joint_cmd(SdkComponentType::ARM_L, 1, 0.0f));
        cmds.push_back(make_joint_cmd(SdkComponentType::ARM_L, 3, 0.0f));
        send_and_hold(manager, cmds, 3);
    }

    // ── Step 6: Waist twist ──────────────────────────────────────
    {
        wait_enter("Step 6: Waist twist to 0.5 rad, hold 3s");
        if (!g_running) goto cleanup;
        std::vector<SdkJointCmd> cmds;
        cmds.push_back(make_joint_cmd(SdkComponentType::WAIST, 0, 0.5f));
        send_and_hold(manager, cmds, 3);

        wait_enter("Step 6b: Return waist to 0");
        if (!g_running) goto cleanup;
        cmds.clear();
        cmds.push_back(make_joint_cmd(SdkComponentType::WAIST, 0, 0.0f));
        send_and_hold(manager, cmds, 3);
    }

    // ── Step 7: Teardown ─────────────────────────────────────────
    wait_enter("Step 7: Return to STAND, exit SDK mode (press Start on gamepad)");
    if (!g_running) goto cleanup;
    manager.SendRobotCmd(SdkStateType::STAND);
    sleep(5);

    manager.SendModeCmd(0);
    LOG(INFO) << "Switched back to RL control mode";
    sleep(2);

cleanup:
    LOG(INFO) << "=== Joint-level control test complete ===";
    google::ShutdownGoogleLogging();
    return 0;
}
