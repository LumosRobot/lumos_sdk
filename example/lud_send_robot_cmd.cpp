#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <glog/logging.h>

/**
 * LUD 状态/速度命令示例。
 *
 * 功能:
 *   依次发送 RESET -> STAND -> RL_WALK_AMP，并下发一个 yaw 速度示例，
 *   最后发送 RL_LIEDOWN。
 *
 * 使用:
 *   cd lumos_sdk
 *   ./build/lud_send_robot_cmd --help
 *   ./build/lud_send_robot_cmd
 *
 * 注意:
 *   这是 LUD 机器人动作示例，不是 NIX 真机 smoke test。运行前必须确认
 *   机器人型号、周围空间、急停和 controller 状态。
 */
int main(int argc, char *argv[])
{
  if (argc > 1 && (std::strcmp(argv[1], "--help") == 0 || std::strcmp(argv[1], "-h") == 0)) {
    std::cout << "Usage: " << argv[0] << "\n"
              << "LUD robot command demo. Sends RESET -> STAND -> RL_WALK_AMP yaw -> RL_LIEDOWN.\n"
              << "Run only on a prepared LUD robot with clear space.\n";
    return 0;
  }

  FLAGS_stderrthreshold = 0; // 将所有级别的日志都输出到标准错误
  FLAGS_minloglevel = 0;     // 设置最小日志级别，0=INFO, 1=WARNING, 2=ERROR, 3=FATAL
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
  LOG(INFO) << "try switching to RL_WALK_AMP state...";
  manager.SendRobotCmd(SdkStateType::RL_WALK_AMP);
  sleep(5);

  // 在RL_WALK状态下，控制机器人前后行走
  // 通过SendRobotCmd的x参数控制，大小表示速度(m/s)，正负表示前后
  // LOG(INFO) << "in RL_AMP_WALK state, try moving forwark/backforward...";
  // manager.SendRobotCmd(SdkStateType::RL_WALK_AMP, -0.2);
  // sleep(3);   

  // 在RL_WALK状态下，控制机器人左右移动
  // 通过SendRobotCmd的y参数控制，大小表示速度(m/s)，正负表示左右
  // LOG(INFO) << "in RL_WALK state, try moving left/right...";
  // manager.SendRobotCmd(SdkStateType::RL_WALK_AMP, 0, -0.2);
  // sleep(3);

  // 在RL_WALK状态下，控制机器人转动
  // 通过SendRobotCmd的yaw参数控制，大小表示速度(rad/s)，正负表示左右
  LOG(INFO) << "in RL_WALK state, try turning left/right...";
  manager.SendRobotCmd(SdkStateType::RL_WALK_AMP, 0, 0, 0.2);
  sleep(3);

  // 在RL_WALK状态下，控制机器人停止行走
  LOG(INFO) << "in RL_WALK_AMP state, try stopping moving...";
  manager.SendRobotCmd(SdkStateType::RL_WALK_AMP);
  sleep(5);
  //?所有的都需要转为amp_walk去验证
  LOG(INFO) << "change state to liedown ...";
  manager.SendRobotCmd(SdkStateType::RL_LIEDOWN);
  google::ShutdownGoogleLogging();
  return 0;
}
