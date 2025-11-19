#pragma once
#include "sdk_base_define.hpp"
#include <vector>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include <sdk_lcmt_joint_datasets.hpp>
#include <microstrain_lcmt.hpp>

using JointDateCb = void (*)(const sdk_lcmt_joint_datasets*);
using ImuDateCb = void (*)(const microstrain_lcmt*);

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

private:
    void HandleLcmRecv();

private:
    lcm::LCM lcm_;
    std::thread lcm_thread_;
};
