#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <glog/logging.h>
#include "microstrain_lcmt.hpp"

static void imu_data_handler(const microstrain_lcmt* imu_data) {
    static uint64_t s_counter = 0;
    if (++s_counter % 100 == 0) {
        LOG(INFO) << "--- imu data received, seq " << s_counter << "---";
        LOG(INFO) << "omega: " << imu_data->omega[0] << ", " << imu_data->omega[1] << ", " << imu_data->omega[2];
        LOG(INFO) << "acc: " << imu_data->acc[0] << ", " << imu_data->acc[1] << ", " << imu_data->acc[2];
        LOG(INFO) << "navQuat: " << imu_data->navQuat[0] << ", " << imu_data->navQuat[1] << ", " << imu_data->navQuat[2] << ", " << imu_data->navQuat[3];
        LOG(INFO) << "navOmega: " << imu_data->navOmega[0] << ", " << imu_data->navOmega[1] << ", " << imu_data->navOmega[2];
        LOG(INFO) << "navRPY: " << imu_data->navRPY[0] << ", " << imu_data->navRPY[1] << ", " << imu_data->navRPY[2];
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

    manager.SetImuDataCb(imu_data_handler);
    LOG(INFO) << "SetImuDataCb done, waiting for handler...";
    sleep(1000000);

    google::ShutdownGoogleLogging();
    return 0;
}
