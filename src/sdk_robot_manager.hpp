#pragma once
#include "sdk_base_define.hpp"
#include <vector>
#include <thread>
#include <atomic>
#include <lcm/lcm-cpp.hpp>
#include <sdk_lcmt_joint_datasets.hpp>
#include <microstrain_lcmt.hpp>
#include <robot_cmd_lcmt.hpp>

using JointDateCb = void (*)(const sdk_lcmt_joint_datasets*);
using ImuDateCb = void (*)(const microstrain_lcmt*);
using GameHandlerCmdCb = void (*)(const robot_cmd_lcmt*);

class SdkRobotManager {
public:
    SdkRobotManager();
    ~SdkRobotManager();

    void Init();

    bool SendRobotCmd(SdkStateType state, float vx = 0, float vy = 0, float vyaw = 0);
    bool SendJointCmds(const std::vector<SdkJointCmd>& joint_cmds);
    bool SendModeCmd(int mode); // 0: RL control mode,  1: SDK control mode

    bool SetJointDataCb(JointDateCb cb);
    bool SetImuDataCb(ImuDateCb cb);
    bool SetGameHandlerCmdCb(GameHandlerCmdCb cb);

    // Call in control loop to dispatch LCM messages (triggers callbacks)
    int HandleLcmTimeout(int timeout_ms = 0) { return lcm_.handleTimeout(timeout_ms); }

private:
    void HandleLcmRecv();

private:
    lcm::LCM lcm_;
    std::thread lcm_thread_;
    std::atomic<bool> stop_flag_{false};
};
