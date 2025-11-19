#include "sdk_robot_manager.hpp"
#include <chrono>
#include <glog/logging.h>
#include "lumos_lcm_control.hpp"
#include "sdk_lcmt_joint_cmds.hpp"
#include "sdk_lcmt_type.hpp"


#define LUMOS_LCM_URL_PORT "udpm://239.255.76.67:7667?ttl=255"

static void ensure_range(float& val, float left, float right) {
    if (val < left) {
        val = left;
    } else if (val > right) {
        val = right;
    }
}

SdkRobotManager::SdkRobotManager() : lcm_(LUMOS_LCM_URL_PORT) {

}

SdkRobotManager::~SdkRobotManager() {

}

void SdkRobotManager::Init() {
    while (!lcm_.good()) {
        LOG(WARNING) << "waiting for lcm ready...";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    lcm_thread_ = std::thread(&SdkRobotManager::HandleLcmRecv, this);
    LOG(INFO) << "SdkRobotManager init done";
}

bool SdkRobotManager::SendRobotCmd(SdkStateType state, float vx, float vy, float vyaw) {
    if (!lcm_.good()) {
        LOG(ERROR) << "failed to SendRobotCmd, lcm not good";
        return false;
    }

    ensure_range(vx, -0.5, 0.5);
    ensure_range(vy, -0.3, 0.3);
    ensure_range(vyaw, -0.5, 0.5);

    lumos_lcm_control robot_cmd;
    robot_cmd.state = static_cast<int8_t>(state);
    robot_cmd.x = vx;
    robot_cmd.y = vy;
    robot_cmd.yaw = vyaw;

    bool ret = (0 == lcm_.publish("lcm_controller", &robot_cmd));
    LOG(INFO) << "SendRobotCmd, state=" << (int)state << ", vx=" << vx << ", vy=" << vy << ", vyaw=" << vyaw << ", ret=" << ret;
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

bool SdkRobotManager::SendModeCmd(int mode) {
    if (!lcm_.good()) {
        LOG(ERROR) << "failed to SendModeCmd, lcm not good";
        return false;
    }

    sdk_lcmt_type mode_cmd;
    mode_cmd.controller_type = mode;
    bool ret = (0 == lcm_.publish("sdk_lcm_set_type_cmd", &mode_cmd));
    LOG(INFO) << "SendModeCmd mode is " << mode;
    return ret;
}

bool SdkRobotManager::SetJointDataCb(JointDateCb cb) {
    if (!lcm_.good()) {
        LOG(ERROR) << "failed to SetJointDataCb, lcm not good";
        return false;
    }
    if (!cb) {
        LOG(ERROR) << "failed to SetJointDataCb, callback function is nullptr";
        return false;
    }

    lcm::LCM::HandlerFunction<sdk_lcmt_joint_datasets> handle_func;
    handle_func = [cb](const lcm::ReceiveBuffer* rbuf, const std::string& channel, const sdk_lcmt_joint_datasets* msg) {
        cb(msg);
    };
    lcm_.subscribe("JointsData", handle_func);

    LOG(INFO) << "set joint data callback success";
    return true;
}

bool SdkRobotManager::SetImuDataCb(ImuDateCb cb) {
    if (!lcm_.good()) {
        LOG(ERROR) << "failed to SetImuDataCb, lcm not good";
        return false;
    }
    if (!cb) {
        LOG(ERROR) << "failed to SetImuDataCb, callback function is nullptr";
        return false;
    }

    lcm::LCM::HandlerFunction<microstrain_lcmt> handle_func;
    handle_func = [cb](const lcm::ReceiveBuffer* rbuf, const std::string& channel, const microstrain_lcmt* msg) {
        cb(msg);
    };
    lcm_.subscribe("myIMU", handle_func);

    LOG(INFO) << "set imu data callback success";
    return true;
}

void SdkRobotManager::HandleLcmRecv() {
    LOG(INFO) << "handle lcm recv thread running...";
    while (true) {
        lcm_.handleTimeout(1000);
    }
}
