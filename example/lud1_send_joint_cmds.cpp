#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <glog/logging.h>

std::vector<SdkJointCmd> set_joint_cmds(int &joint_num, const float *poses, const float *kps,
                                        const float *kds, LudSdkComponentType type)
{
  std::vector<SdkJointCmd> cmds(joint_num);
  for (int i = 0; i < joint_num; ++i)
  {
    cmds[i].component_type = static_cast<int16_t>(type);
    cmds[i].joint_id = i;
    cmds[i].ctrlWord = 3;
    cmds[i].tarPos = poses[i];
    cmds[i].tarVel = 0.0f;
    cmds[i].tarTor = 0;
    cmds[i].res1 = kps[i];
    cmds[i].res2 = kds[i];
  }
  return cmds;
}

int main(int argc, char *argv[])
{
  (void)(argc);

  FLAGS_stderrthreshold = 0; // 将所有级别的日志都输出到标准错误
  FLAGS_minloglevel = 0;     // 设置最小日志级别，0=INFO, 1=WARNING, 2=ERROR, 3=FATAL
  google::InitGoogleLogging(argv[0]);

  SdkRobotManager manager;
  manager.Init();
  sleep(5);

  LOG(INFO) << "try switching to RESET state...";
  manager.SendRobotCmd(SdkStateType::RESET);
  sleep(10);

  LOG(INFO) << "try switching to STAND state...";
  manager.SendRobotCmd(SdkStateType::STAND);
  sleep(10);

  LOG(INFO) << "try switching to SDK control mode...";
  manager.SendModeCmd(1);
  sleep(10);

  int arm_joints_num = 4;
  float arm_stand_poses[] = {0.0, 0.74, -1.49, 0.0}; // 默认站立位置
  float arm_stand_kps[] = {400, 400, 300, 0};
  float arm_stand_kds[] = {6, 6, 6, 2};

  int leg_joints_num = 4;
  float leg_stand_poses[] = {0.0, -0.74, 1.49, 0.0}; // 默认站立位置
  float leg_stand_kps[] = {400, 400, 300, 0};
  float leg_stand_kds[] = {6, 6, 6, 2};


  float arm_cmd_fl[] = {0.0, 0.73, -1.48, 0.0};
  float arm_cmd_fr[] = {0.0, 0.73, -1.48, 0.0};
  float leg_cmd_hl[] = {0.0, -0.73, 1.48, 0.0};
  float leg_cmd_hr[] = {0.0, -0.73, 1.48, 0.0};

  std::vector<SdkJointCmd> set_cmds_FL(arm_joints_num);
  std::vector<SdkJointCmd> set_cmds_FR(arm_joints_num);
  std::vector<SdkJointCmd> set_cmds_HL(leg_joints_num);
  std::vector<SdkJointCmd> set_cmds_HR(leg_joints_num);

  std::vector<SdkJointCmd> stand_cmds_FL(arm_joints_num);
  std::vector<SdkJointCmd> stand_cmds_FR(arm_joints_num);
  std::vector<SdkJointCmd> stand_cmds_HL(leg_joints_num);
  std::vector<SdkJointCmd> stand_cmds_HR(leg_joints_num);

  set_cmds_FL = set_joint_cmds(arm_joints_num, arm_cmd_fl, arm_stand_kps, arm_stand_kds, LudSdkComponentType::ARM_L);
  set_cmds_FR = set_joint_cmds(arm_joints_num, arm_cmd_fr, arm_stand_kps, arm_stand_kds, LudSdkComponentType::ARM_R);
  set_cmds_HL = set_joint_cmds(leg_joints_num, leg_cmd_hl, leg_stand_kps, leg_stand_kds, LudSdkComponentType::LEG_L);
  set_cmds_HR = set_joint_cmds(leg_joints_num, leg_cmd_hr, leg_stand_kps, leg_stand_kds, LudSdkComponentType::LEG_R);

  stand_cmds_FL = set_joint_cmds(arm_joints_num, arm_stand_poses, arm_stand_kps, arm_stand_kds, LudSdkComponentType::ARM_L);
  stand_cmds_FR = set_joint_cmds(arm_joints_num, arm_stand_poses, arm_stand_kps, arm_stand_kds, LudSdkComponentType::ARM_R);
  stand_cmds_HL = set_joint_cmds(leg_joints_num, leg_stand_poses, leg_stand_kps, leg_stand_kds, LudSdkComponentType::LEG_L);
  stand_cmds_HR = set_joint_cmds(leg_joints_num, leg_stand_poses, leg_stand_kps, leg_stand_kds, LudSdkComponentType::LEG_R);

  LOG(INFO) << "try SendJointCmds by set_cmds...";
  std::vector<SdkJointCmd> all_commands; 
  all_commands.insert(all_commands.end(),set_cmds_FL.begin(),set_cmds_FL.end());
  all_commands.insert(all_commands.end(),set_cmds_FR.begin(),set_cmds_FR.end());
  all_commands.insert(all_commands.end(),set_cmds_HL.begin(),set_cmds_HL.end());
  all_commands.insert(all_commands.end(),set_cmds_HR.begin(),set_cmds_HR.end());
  
  manager.SendJointCmds(all_commands);

  sleep(20);

  LOG(INFO) << "try SendJointCmds by stand_cmds...";
  std::vector<SdkJointCmd> stand_all_cmds;
  stand_all_cmds.insert(stand_all_cmds.end(),stand_cmds_FL.begin(),stand_cmds_FL.end());
  stand_all_cmds.insert(stand_all_cmds.end(),stand_cmds_FR.begin(),stand_cmds_FR.end());
  stand_all_cmds.insert(stand_all_cmds.end(),stand_cmds_HL.begin(),stand_cmds_HL.end());
  stand_all_cmds.insert(stand_all_cmds.end(),stand_cmds_HR.begin(),stand_cmds_HR.end());
  
  manager.SendJointCmds(stand_all_cmds);
  sleep(20);
  
  manager.SendRobotCmd(SdkStateType::RL_LIEDOWN);
  LOG(INFO) << "try switching to RL_LIEDOWN state...";
  sleep(30);

  // LOG(INFO) << "try switching to RL control mode...";
  // manager.SendModeCmd(0);
  // sleep(2);

  google::ShutdownGoogleLogging();
  return 0;
}
