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
    LOG(INFO) << "try switching to RESET state...";
    manager.SendRobotCmd(SdkStateType::RESET);
    sleep(10);

    // 控制机器人切换到STAND状态
    LOG(INFO) << "try switching to STAND state...";
    manager.SendRobotCmd(SdkStateType::STAND);
    sleep(20);

    // 控制机器人切换到RL_WALK状态
    LOG(INFO) << "try switching to RL_WALK state...";
    manager.SendRobotCmd(SdkStateType::RL_WALK);
    sleep(5);

    // 在RL_WALK状态下，控制机器人前后行走
    // 通过SendRobotCmd的x参数控制，大小表示速度(m/s)，正负表示前后
    LOG(INFO) << "in RL_WALK state, try moving forwark/backforward...";
    manager.SendRobotCmd(SdkStateType::RL_WALK, 0.2);
    sleep(10);

    // 在RL_WALK状态下，控制机器人左右移动
    // 通过SendRobotCmd的y参数控制，大小表示速度(m/s)，正负表示左右
    // LOG(INFO) << "in RL_WALK state, try moving left/right...";
    // manager.SendRobotCmd(SdkStateType::RL_WALK, 0, 0.1);
    // sleep(10);

    // 在RL_WALK状态下，控制机器人转动
    // 通过SendRobotCmd的yaw参数控制，大小表示速度(rad/s)，正负表示左右
    // LOG(INFO) << "in RL_WALK state, try turning left/right...";
    // manager.SendRobotCmd(SdkStateType::RL_WALK, 0, 0, 0.2);
    // sleep(10);

    // 在RL_WALK状态下，控制机器人停止行走
    LOG(INFO) << "in RL_WALK state, try stopping moving...";
    manager.SendRobotCmd(SdkStateType::RL_WALK);
    sleep(5);

    google::ShutdownGoogleLogging();
    return 0;
}
