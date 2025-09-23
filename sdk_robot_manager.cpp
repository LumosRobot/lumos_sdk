#include "sdk_robot_manager.hpp"
#include <chrono>
#include <thread>
#include <glog/logging.h>
#include "lumos_lcm_control.hpp"
#include "sdk_lcmt_joint_cmds.hpp"


SdkRobotManager::SdkRobotManager() : lcm_(LUMOS_LCM_URL_PORT) {

}

SdkRobotManager::~SdkRobotManager() {

}

void SdkRobotManager::Init() {
    while (!lcm_.good()) {
        LOG(WARNING) << "waiting for lcm ready...";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    LOG(INFO) << "SdkRobotManager init done";
}

bool SdkRobotManager::SendRobotCmd(SdkStateType state, float x, float y, float yaw) {
    if (!lcm_.good()) {
        LOG(ERROR) << "failed to SendRobotCmd, lcm not good";
        return false;
    }

    lumos_lcm_control robot_cmd;
    robot_cmd.state = state;
    robot_cmd.x = x;
    robot_cmd.y = y;
    robot_cmd.yaw = yaw;

    bool ret = (0 == lcm_.publish("lcm_controller", &robot_cmd));
    LOG(INFO) << "SendRobotCmd, state=" << state << ", x=" << x << ", y=" << y << ", yaw=" << yaw << ", ret=" << ret;
    return ret;
}

bool SdkRobotManager::SendJointCmds(const std::vector<SdkJointCmd>& joint_cmds) {
    if (!lcm_.good() || joint_cmds.empty()) {
        LOG(ERROR) << "failed to SendJointCmds, lcm not good or joint_cmds empty, size is " << joint_cmds.size();
        return false;
    }

    sdk_lcmt_joint_cmds lcmt_joint_cmds;
    lcmt_joint_cmds.cmds_num = joint_cmds.size();
    lcmt_joint_cmds.cmds.resize(lcmt_joint_cmds.cmds_num);

    for (size_t i = 0; i < joint_cmds.size(); ++i) {
        sdk_lcmt_joint_cmd lcmt_joint_cmd;
        lcmt_joint_cmd.component_type = joint_cmds[i].component_type;
        lcmt_joint_cmd.joint_id = joint_cmds[i].joint_id;
        lcmt_joint_cmd.ctrlWord = joint_cmds[i].ctrlWord;
        lcmt_joint_cmd.tarPos = joint_cmds[i].tarPos;
        lcmt_joint_cmd.tarVel = joint_cmds[i].tarVel;
        lcmt_joint_cmd.tarCur = joint_cmds[i].tarCur;
        lcmt_joint_cmd.tarTor = joint_cmds[i].tarTor;
        lcmt_joint_cmd.res1 = joint_cmds[i].res1;
        lcmt_joint_cmd.res2 = joint_cmds[i].res2;
        lcmt_joint_cmd.res3 = joint_cmds[i].res3;
        lcmt_joint_cmd.res4 = joint_cmds[i].res4;
        lcmt_joint_cmds.cmds[i] = lcmt_joint_cmd;
    }

    bool ret = (0 == lcm_.publish("sdk_lcm_joint_cmds", &lcmt_joint_cmds));
    LOG(INFO) << "SendJointCmds, cmds size is " << joint_cmds.size() << ", ret=" << ret;
    return ret;
}
