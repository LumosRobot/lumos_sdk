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

    // 控制机器人切换到RESET状态
    LOG(INFO) << "try switching to RESET mode...";
    manager.SendRobotCmd(SdkStateType::RESET);
    sleep(5);

    // 控制机器人切换到STAND状态
    LOG(INFO) << "try switching to STAND mode...";
    manager.SendRobotCmd(SdkStateType::STAND);
    sleep(5);

    // 控制机器人切换到RL_WALK状态
    LOG(INFO) << "try switching to RL_WALK mode...";
    manager.SendRobotCmd(SdkStateType::RL_WALK);
    sleep(5);

    // 在RL_WALK状态下，控制机器人向前行走
    LOG(INFO) << "in RL_WALK mode, try moving forwark...";
    manager.SendRobotCmd(SdkStateType::RL_WALK, 0.2);
    sleep(5);

    // 在RL_WALK状态下，控制机器人停止行走
    LOG(INFO) << "in RL_WALK mode, try stopping moving...";
    manager.SendRobotCmd(SdkStateType::RL_WALK);
    sleep(5);

    google::ShutdownGoogleLogging();
    return 0;
}
