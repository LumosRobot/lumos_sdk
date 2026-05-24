#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <csignal>
#include <chrono>
#include <glog/logging.h>

static volatile bool g_running = true;
static void sigint_handler(int) { g_running = false; }

struct FeedbackStats {
    uint64_t joint_cnt = 0;
    uint64_t imu_cnt   = 0;
    uint64_t last_joint_cnt = 0;
    uint64_t last_imu_cnt   = 0;
    std::chrono::steady_clock::time_point last_report;
};

static FeedbackStats g_stats;

static void joint_data_handler(const sdk_lcmt_joint_datasets* data) {
    g_stats.joint_cnt++;
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - g_stats.last_report).count();

    if (elapsed >= 2.0) {
        double joint_rate = (g_stats.joint_cnt - g_stats.last_joint_cnt) / elapsed;
        double imu_rate   = (g_stats.imu_cnt - g_stats.last_imu_cnt) / elapsed;

        std::cout << "\n=== FEEDBACK @ " << std::fixed << std::setprecision(1)
                  << elapsed << "s interval ===" << std::endl;
        std::cout << "  Joints: " << std::setw(6) << std::setprecision(1) << joint_rate << " Hz | "
                  << data->datasets.size() << " joints";
        for (size_t i = 0; i < data->datasets.size() && i < 4; ++i) {
            std::cout << " | j" << i << ": " << std::setprecision(4)
                      << data->datasets[i].pos_high;
        }
        std::cout << std::endl;

        g_stats.last_report    = now;
        g_stats.last_joint_cnt = g_stats.joint_cnt;
        g_stats.last_imu_cnt   = g_stats.imu_cnt;
    }
}

static void imu_data_handler(const microstrain_lcmt* data) {
    g_stats.imu_cnt++;
    static uint64_t print_counter = 0;
    if (++print_counter % 400 == 0) {
        std::cout << "  IMU    : " << std::fixed << std::setprecision(4)
                  << "RPY=[" << data->navRPY[0] << ", " << data->navRPY[1] << ", " << data->navRPY[2] << "]"
                  << " acc=[" << data->acc[0] << ", " << data->acc[1] << ", " << data->acc[2] << "]"
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
    g_stats.last_report = std::chrono::steady_clock::now();

    LOG(INFO) << "Listening for JointsData + myIMU (Ctrl+C to stop)...";

    while (g_running) {
        sleep(1);
    }

    std::cout << "\nTotal: " << g_stats.joint_cnt << " joint msgs, "
              << g_stats.imu_cnt << " IMU msgs" << std::endl;
    LOG(INFO) << "Feedback test done";
    google::ShutdownGoogleLogging();
    return 0;
}
