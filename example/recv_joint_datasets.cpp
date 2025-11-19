#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <glog/logging.h>
#include "sdk_lcmt_joint_datasets.hpp"

static void joint_data_handler(const sdk_lcmt_joint_datasets* joint_datasets) {
    static uint64_t s_counter = 0;
    if (++s_counter % 100 == 0) {
        LOG(INFO) << "------ joint datasets received, seq " << s_counter << "------";
        LOG(INFO) << "joint datasets num: " << joint_datasets->datasets_num;
        for (size_t i = 0; i < joint_datasets->datasets.size(); ++i) {
            LOG(INFO) << "joint " << i << ": comp_type=" << joint_datasets->datasets[i].component_type 
                << ", joint_id=" << joint_datasets->datasets[i].joint_id
                << ", pos_high=" << joint_datasets->datasets[i].pos_high
                << ", pos_low=" << joint_datasets->datasets[i].pos_low;
        }
        LOG(INFO) << "-------------------------------------------------";
    }
}

int main(int argc, char* argv[])
{
    (void)(argc);

    FLAGS_stderrthreshold = 0;      // 将所有级别的日志都输出到标准错误
    FLAGS_minloglevel = 0;          // 设置最小日志级别，0=INFO, 1=WARNING, 2=ERROR, 3=FATAL
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
    sleep(2);

    manager.SetJointDataCb(joint_data_handler);
    LOG(INFO) << "SetJointDataCb done, waiting for handler...";
    sleep(1000000);

    google::ShutdownGoogleLogging();
    return 0;
}
