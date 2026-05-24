#include "sdk_robot_manager.hpp"
#include <iostream>
#include <map>
#include <random>
#include <unistd.h>
#include <glog/logging.h>


struct JointParam {
    float lower;
    float upper;
    float kp;
    float kd;
};

static const JointParam s_joint_param_ary[] = {
    { -3, 3, 60.0, 2.0 },// torso_joint
    { -1.3, 1.3, 60.0, 2.0 },// head_joint
    { -3, 3, 60.0, 2.0 },// left_shoulder_pitch_joint
    { -0.12, 2.3, 60.0, 2.0 },// left_shoulder_roll_joint
    { -1.9, 1.9, 60.0, 2.0 },// left_shoulder_yaw_joint
    { -0.83, 1.57, 60.0, 2.0 },// left_elbow_joint
    { -2.3, 2.3, 60.0, 2.0 },// left_wrist_yaw_joint
    { -1.2, 1.2, 60.0, 2.0 },// left_wrist_pitch_joint
    { -1.2, 1.2, 60.0, 2.0 },// left_wrist_roll_joint
    { -3, 3, 60.0, 2.0 },// right_shoulder_pitch_joint
    { -2.3, 0.12, 60.0, 2.0 },// right_shoulder_roll_joint
    { -1.9, 1.9, 60.0, 2.0 },// right_shoulder_yaw_joint
    { -0.83, 1.57, 60.0, 2.0 },// right_elbow_joint
    { -2.3, 2.3, 60.0, 2.0 },// right_wrist_yaw_joint
    { -1.2, 1.2, 60.0, 2.0 },// right_wrist_pitch_joint
    { -1.2, 1.2, 60.0, 2.0 },// right_wrist_roll_joint
    { -1.5, 1.5, 200.0, 6.0 },// left_hip_pitch_joint
    { -0.25, 2.8, 200.0, 6.0 },// left_hip_roll_joint
    { -1.2, 2.6, 200.0, 6.0 },// left_hip_yaw_joint
    { 0.0, 2.27, 200.0, 6.0 },// left_knee_joint
    { -1.0, 0.43, 80.0, 2.0 },// left_ankle_pitch_joint
    { -0.43, 0.43, 80.0, 2.0 },// left_ankle_roll_link
    { -1.5, 1.5, 200.0, 6.0 },// right_hip_pitch_joint
    { -2.8, 0.25, 200.0, 6.0 },// right_hip_roll_joint
    { -2.6, 1.2, 200.0, 6.0 },// right_hip_yaw_joint
    { 0.0, 2.27, 200.0, 6.0 },// right_knee_joint
    { -1.0, 0.43, 80.0, 2.0 },// right_ankle_pitch_joint
    { -0.43, 0.43, 80.0, 2.0 },// right_ankle_roll_joint
};

static const std::map<SdkComponentType, std::vector<int>> s_component_limits_map = {
    { SdkComponentType::WAIST, { 0 } },
    { SdkComponentType::HEAD, { 1 } },
    { SdkComponentType::ARM_L, { 2, 3, 4, 5, 6, 7, 8 } },
    { SdkComponentType::ARM_R, { 9, 10, 11, 12, 13, 14, 15 } },
    { SdkComponentType::LEG_L, { 16, 17, 18, 19, 20, 21 } },
    { SdkComponentType::LEG_R, { 22, 23, 24, 25, 26, 27 } },
};

static float random_float(float min, float max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    std::uniform_real_distribution<float> dis(min + 0.1, max - 0.1);
    return dis(gen);
}

static SdkJointCmd generate_joint_cmd(SdkComponentType component_type, int local_joint_id, float target_pos) {
    LOG(INFO) << "generate_joint_cmd, type=" << (int)component_type << ", id=" << local_joint_id << ", pos=" << target_pos;
    SdkJointCmd joint_cmd;
    if (local_joint_id < 0 || local_joint_id >= s_component_limits_map.at(component_type).size()) {
        LOG(FATAL) << "generate_joint_cmd failed! invalid local_joint_id, type=" << (int)component_type << ", id=" << local_joint_id;
        return joint_cmd;
    }

    int global_joint_id = s_component_limits_map.at(component_type)[local_joint_id];
    float min_limit = s_joint_param_ary[global_joint_id].lower;
    float max_limit = s_joint_param_ary[global_joint_id].upper;
    if (target_pos < min_limit || target_pos > max_limit) {
        LOG(FATAL) << "generate_joint_cmd failed! invalid target_pos";
        return joint_cmd;
    }

    joint_cmd.component_type = static_cast<int16_t>(component_type);
    joint_cmd.joint_id = local_joint_id;
    joint_cmd.ctrlWord = 3;
    joint_cmd.tarPos = target_pos;
    joint_cmd.tarVel = 0.0f;
    joint_cmd.tarTor = 0;
    joint_cmd.res1 = 400;
    joint_cmd.res2 = 2;

    return joint_cmd;
}

static SdkJointCmd generate_joint_cmd(SdkComponentType component_type, int local_joint_id) {
    SdkJointCmd joint_cmd;
    if (local_joint_id < 0 || local_joint_id >= s_component_limits_map.at(component_type).size()) {
        LOG(FATAL) << "generate_joint_cmd failed! invalid local_joint_id, type=" << (int)component_type << ", id=" << local_joint_id;
        return joint_cmd;
    }

    int global_joint_id = s_component_limits_map.at(component_type)[local_joint_id];
    float min_limit = s_joint_param_ary[global_joint_id].lower;
    float max_limit = s_joint_param_ary[global_joint_id].upper;
    float target_pos = random_float(min_limit, max_limit);


    std::cout << "generate_joint_cmd success!";

    return generate_joint_cmd(component_type, local_joint_id, target_pos);
}

template <typename... Args>
static void send_joint_cmd(SdkRobotManager& manager, Args&&... args) {
    SdkJointCmd joint_cmd = generate_joint_cmd(std::forward<Args>(args)...);
    manager.SendJointCmds({joint_cmd});
    sleep(3);

    joint_cmd.tarPos = 0;
    manager.SendJointCmds({joint_cmd});
    sleep(3);

    LOG(INFO) << "send_joint_cmd done! press ENTER to continue...";
    std::string input;
    std::getline(std::cin, input);
}

static void send_joint_cmds(SdkRobotManager& manager, const std::vector<SdkJointCmd>& joint_cmds, bool need_set_zero = true) {
    manager.SendJointCmds(joint_cmds);
    sleep(5);

    if (need_set_zero) {
        std::vector<SdkJointCmd> joint_cmds_zero = joint_cmds;
        for (auto& joint_cmd : joint_cmds_zero) {
            joint_cmd.tarPos = 0;
        }

        manager.SendJointCmds(joint_cmds_zero);
        sleep(5);
    }

    LOG(INFO) << "send_joint_cmds done! press ENTER to continue...";
    std::string input;
    std::getline(std::cin, input);
}

int main(int argc, char* argv[])
{
    (void)(argc);
    FLAGS_stderrthreshold = 0;      // 将所有级别的日志都输出到标准错误
    FLAGS_minloglevel = 0;          // 设置最小日志级别，0=INFO, 1=WARNING, 2=ERROR, 3=FATAL
    google::InitGoogleLogging(argv[0]);

    std::string input;
    SdkRobotManager manager;
    manager.Init();
    sleep(3);

    LOG(INFO) << "try switching to RESET state...";
    manager.SendRobotCmd(SdkStateType::RESET);
    sleep(10);

    LOG(INFO) << "try switching to STAND state...";
    manager.SendRobotCmd(SdkStateType::STAND);
    sleep(10);

    LOG(INFO) << "try switching to SDK control mode...";
    manager.SendModeCmd(1);
    sleep(2);

    LOG(INFO) << "请确定机器人已经准备就绪! 即将开始单个关节随机测试！";
    LOG(INFO) << "press ENTER to continue...";
    std::getline(std::cin, input);

    // 每个关节都动一下，动的位置在有效范围内随机，动完后自动设置为零位
    // 动完一个关节后，按回车开始动下一个关节
    send_joint_cmd(manager, SdkComponentType::WAIST, 0);
    send_joint_cmd(manager, SdkComponentType::HEAD, 0);
    // ARM_L
    send_joint_cmd(manager, SdkComponentType::ARM_L, 0);
    send_joint_cmd(manager, SdkComponentType::ARM_L, 1);
    send_joint_cmd(manager, SdkComponentType::ARM_L, 2);
    send_joint_cmd(manager, SdkComponentType::ARM_L, 3);
    send_joint_cmd(manager, SdkComponentType::ARM_L, 4);
    send_joint_cmd(manager, SdkComponentType::ARM_L, 5);
    send_joint_cmd(manager, SdkComponentType::ARM_L, 6);
    // ARM_R
    send_joint_cmd(manager, SdkComponentType::ARM_R, 0);
    send_joint_cmd(manager, SdkComponentType::ARM_R, 1);
    send_joint_cmd(manager, SdkComponentType::ARM_R, 2);
    send_joint_cmd(manager, SdkComponentType::ARM_R, 3);
    send_joint_cmd(manager, SdkComponentType::ARM_R, 4);
    send_joint_cmd(manager, SdkComponentType::ARM_R, 5);
    send_joint_cmd(manager, SdkComponentType::ARM_R, 6);
    // LEG_L
    send_joint_cmd(manager, SdkComponentType::LEG_L, 0);
    send_joint_cmd(manager, SdkComponentType::LEG_L, 1);
    send_joint_cmd(manager, SdkComponentType::LEG_L, 2);
    send_joint_cmd(manager, SdkComponentType::LEG_L, 3);
    send_joint_cmd(manager, SdkComponentType::LEG_L, 4);
    send_joint_cmd(manager, SdkComponentType::LEG_L, 5);
    // LEG_R
    send_joint_cmd(manager, SdkComponentType::LEG_R, 0);
    send_joint_cmd(manager, SdkComponentType::LEG_R, 1);
    send_joint_cmd(manager, SdkComponentType::LEG_R, 2);
    send_joint_cmd(manager, SdkComponentType::LEG_R, 3);
    send_joint_cmd(manager, SdkComponentType::LEG_R, 4);
    send_joint_cmd(manager, SdkComponentType::LEG_R, 5);

    LOG(INFO) << "单个关节随机测试完成! 即将开始单个关节精确测试！";
    LOG(INFO) << "press ENTER to continue...";
    std::getline(std::cin, input);

    // 单个关节精确动到指定的位置，需要传入位置值，动完后自动设置为零位
    // 左膝关节动到90度
    send_joint_cmd(manager, SdkComponentType::LEG_L, 3, 1.5708);
    // 右肘关节，动到120度，之后成零位是90度夹角
    send_joint_cmd(manager, SdkComponentType::ARM_R, 3, -0.5236);

    LOG(INFO) << "单个关节精确测试完成! 即将开始组合关节测试！";
    LOG(INFO) << "press ENTER to continue...";
    std::getline(std::cin, input);
    
    std::vector<SdkJointCmd> joint_cmds;
    // 关节组合运动，动完后每个关节都自动设置为零位
    // 左手的这3个关节，设置到有效范围内的中间位置
    joint_cmds.clear();
    joint_cmds.emplace_back(generate_joint_cmd(SdkComponentType::ARM_L, 0, 0.8));
    joint_cmds.emplace_back(generate_joint_cmd(SdkComponentType::ARM_L, 1, 1.1));
    joint_cmds.emplace_back(generate_joint_cmd(SdkComponentType::ARM_L, 3, 0.4));
    send_joint_cmds(manager, joint_cmds);
    // 右腿的这4个关节，设置到有效范围内的中间位置
    joint_cmds.clear();
    joint_cmds.emplace_back(generate_joint_cmd(SdkComponentType::LEG_R, 1, -1.3));
    joint_cmds.emplace_back(generate_joint_cmd(SdkComponentType::LEG_R, 2, -0.7));
    joint_cmds.emplace_back(generate_joint_cmd(SdkComponentType::LEG_R, 3, 1.3));
    joint_cmds.emplace_back(generate_joint_cmd(SdkComponentType::LEG_R, 4, -1));
    send_joint_cmds(manager, joint_cmds);
    // 左手2个关节，右腿3个关节，设置到有效范围内的中间位置
    joint_cmds.clear();
    joint_cmds.emplace_back(generate_joint_cmd(SdkComponentType::ARM_L, 0, 0.8));
    joint_cmds.emplace_back(generate_joint_cmd(SdkComponentType::ARM_L, 1, 1.1));
    joint_cmds.emplace_back(generate_joint_cmd(SdkComponentType::LEG_R, 2, -0.7));
    joint_cmds.emplace_back(generate_joint_cmd(SdkComponentType::LEG_R, 3, 1.3));
    joint_cmds.emplace_back(generate_joint_cmd(SdkComponentType::LEG_R, 4, -1));

    LOG(INFO) << "try switching to RL control mode...";
    manager.SendModeCmd(0);
    sleep(2);

    LOG(INFO) << "测试结束！！！";
    google::ShutdownGoogleLogging();
    return 0;
}