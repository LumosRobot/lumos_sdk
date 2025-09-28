#pragma once
#include "sdk_base_define.hpp"
#include <vector>
#include <lcm/lcm-cpp.hpp>

class SdkRobotManager {
public:
    SdkRobotManager();
    ~SdkRobotManager();

    void Init();

    bool SendRobotCmd(SdkStateType state, float x = 0, float y = 0, float yaw = 0);
    bool SendJointCmds(const std::vector<SdkJointCmd>& joint_cmds);

private:
    lcm::LCM lcm_;
};
