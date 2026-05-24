#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <iostream>
#include <csignal>
#include <glog/logging.h>

static volatile bool g_running = true;
static void sigint_handler(int) { g_running = false; }

static void joint_data_handler(const sdk_lcmt_joint_datasets* data) {
    static uint64_t counter = 0;
    if (++counter % 50 == 0) {
        std::cout << "\n[JOINT #" << counter << "] " << data->datasets.size() << " joints";
        for (size_t i = 0; i < data->datasets.size() && i < 4; ++i) {
            std::cout << " | j" << i << ": pos=" << data->datasets[i].pos_high;
        }
        std::cout << std::endl;
    }
}

static void imu_data_handler(const microstrain_lcmt* data) {
    static uint64_t counter = 0;
    if (++counter % 100 == 0) {
        std::cout << "[IMU #" << counter << "] RPY: "
                  << data->navRPY[0] << ", " << data->navRPY[1] << ", " << data->navRPY[2]
                  << " | acc: " << data->acc[0] << ", " << data->acc[1] << ", " << data->acc[2]
                  << std::endl;
    }
}

int main(int argc, char* argv[]) {
    (void)(argc);
    signal(SIGINT, sigint_handler);

    FLAGS_stderrthreshold = 0;
    FLAGS_minloglevel = 0;
    google::InitGoogleLogging(argv[0]);

    SdkRobotManager manager;
    manager.Init();
    manager.SetJointDataCb(joint_data_handler);
    manager.SetImuDataCb(imu_data_handler);
    LOG(INFO) << "Listening for joint + IMU data (Ctrl+C to stop)...";

    while (g_running) {
        sleep(1);
    }

    LOG(INFO) << "Feedback test done";
    google::ShutdownGoogleLogging();
    return 0;
}
