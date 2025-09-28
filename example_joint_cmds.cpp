#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <glog/logging.h>


int main(int argc, char* argv[])
{
    (void)(argc);

    FLAGS_stderrthreshold = 0;      // 将所有级别的日志都输出到标准错误
    FLAGS_minloglevel = 0;          // 设置最小日志级别，0=INFO, 1=WARNING, 2=ERROR, 3=FATAL
    google::InitGoogleLogging(argv[0]);

    SdkRobotManager manager;
    manager.Init();
    sleep(5);

    LOG(INFO) << "try switching to RESET mode...";
    manager.SendRobotCmd(SdkStateType::RESET);
    sleep(5);

    LOG(INFO) << "try switching to DEBUG mode...";
    manager.SendRobotCmd(SdkStateType::DEBUG);
    sleep(5);

    int leg_joints_num = 6;
    float leg_stand_poses[] = { -0.1, 0, 0, 0.2, -0.1, 0 };
    float leg_stand_kps[] = { 600, 600, 600, 600, 400, 400 };
    float leg_stand_kds[] = { 6, 2, 6, 6, 6, 6 };

    std::vector<SdkJointCmd> reset_cmds(leg_joints_num);
    std::vector<SdkJointCmd> stand_cmds(leg_joints_num);
    for (size_t i = 0; i < stand_cmds.size(); ++i) {
        reset_cmds[i].component_type = SdkComponentType::LEG_R;
        reset_cmds[i].joint_id = i;
        reset_cmds[i].ctrlWord = 200;

        stand_cmds[i].component_type = SdkComponentType::LEG_R;
        stand_cmds[i].joint_id = i;
        stand_cmds[i].ctrlWord = 3;
        stand_cmds[i].tarPos = leg_stand_poses[i];
        stand_cmds[i].tarVel = 0.0f;
        stand_cmds[i].tarTor = 0;
        stand_cmds[i].res1 = leg_stand_kps[i];
        stand_cmds[i].res2 = leg_stand_kds[i];
    }

    for (int i = 0; i < 10; ++i) {
        LOG(INFO) << "try SendJointCmds by stand_cmds...";
        manager.SendJointCmds(stand_cmds);
        sleep(5);

        LOG(INFO) << "try SendJointCmds by reset_cmds...";
        manager.SendJointCmds(reset_cmds);
        sleep(5);
    }

    google::ShutdownGoogleLogging();
    return 0;
}
