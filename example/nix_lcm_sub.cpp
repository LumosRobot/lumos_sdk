#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <glog/logging.h>

/**
 * @brief NIX2 SDK 被动数据记录程序。
 *
 *        本程序只订阅 LCM 数据并写出 CSV 文件，不会发送状态命令、
 *        SendRobotCmd() 或任何关节控制指令，因此不会抢占 nix_joint_cmd
 *        对 DEBUG 状态和机器人运动的控制权。
 *
 *        推荐运行流程:
 *          1. 机器人端先启动 lumos_controller。
 *          2. 在本地 build/ 目录运行: ./nix_lcm_sub
 *          3. 另开终端运行: ./nix_joint_cmd
 *             由 nix_joint_cmd 负责进入 DEBUG 状态并下发动作。
 *          4. 动作结束后，在 nix_lcm_sub 终端按 Ctrl-C 保存数据。
 *
 *        数据说明:
 *          - lcm_imu_data 通常在进入 DEBUG 状态前就能收到。
 *          - lcm_joint_data 和周期性的 lcm_robot_status 通常在 controller 进入
 *            DEBUG 关节级控制后开始发布，因此一般会在 nix_joint_cmd
 *            进入 DEBUG 状态后才持续记录。
 */

static volatile std::sig_atomic_t g_running = 1;
static std::atomic<int> g_robot_state{0};
static std::atomic<int> g_robot_type{0};
static std::atomic<int> g_joint_msg_count{0};
static std::atomic<int> g_imu_msg_count{0};
static std::atomic<int> g_status_msg_count{0};

static std::mutex g_data_mutex;

struct JointRecord {
    std::string ts;
    int16_t component_type;
    int16_t joint_id;
    int16_t stat;
    float pos_high;
    float pos_low;
    float vel;
    float tor;
};

struct ImuRecord {
    std::string ts;
    float omega[3];
    float acc[3];
    float navQuat[4];
    float navRPY[3];
    float temp;
};

struct StatusRecord {
    std::string ts;
    int8_t state;
    int8_t type;
};

static std::vector<JointRecord> g_joint_records;
static std::vector<ImuRecord> g_imu_records;
static std::vector<StatusRecord> g_status_records;

static void sigint_handler(int) { g_running = 0; }

static std::string wall_time_string() {
    auto now = std::chrono::system_clock::now();
    auto sec = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto usec = std::chrono::duration_cast<std::chrono::microseconds>(now - sec).count();
    std::time_t t = std::chrono::system_clock::to_time_t(now);

    std::tm tm_now;
    localtime_r(&t, &tm_now);

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S")
        << "." << std::setw(6) << std::setfill('0') << usec;
    return oss.str();
}

static const char* state_name(int8_t s) {
    switch (s) {
        case 0:  return "NOT_A_STATE";
        case 1:  return "RESET";
        case 2:  return "STAND";
        case 3:  return "RL_WALK";
        case 5:  return "RL_LIEDOWN";
        case 6:  return "RL_MIMIC";
        case 11: return "RL_NAV";
        case 12: return "RL_WALK_AMP";
        case 20: return "BY_MIMIC";
        case 21: return "BFM_MIMIC";
        default: return "UNKNOWN";
    }
}

static const char* component_name(int16_t t) {
    switch (t) {
        case 1: return "ARM_L";
        case 2: return "ARM_R";
        case 7: return "WAIST";
        case 8: return "LEG_L";
        case 9: return "LEG_R";
        default: return "UNKNOWN";
    }
}

static void on_robot_status(const robot_status_lcmt* msg) {
    g_robot_state = msg->state;
    g_robot_type = msg->type;
    g_status_msg_count++;

    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_status_records.push_back({wall_time_string(), msg->state, msg->type});
}

static void on_joint_data(const joint_datasets_lcmt* msg) {
    const std::string ts = wall_time_string();
    g_joint_msg_count++;

    std::lock_guard<std::mutex> lock(g_data_mutex);
    for (int i = 0; i < msg->datasets_num; i++) {
        const auto& d = msg->datasets[i];
        g_joint_records.push_back({
            ts,
            d.component_type,
            d.joint_id,
            d.stat,
            d.pos_high,
            d.pos_low,
            d.vel,
            d.tor
        });
    }
}

static void on_imu_data(const imu_data_lcmt* msg) {
    g_imu_msg_count++;

    std::lock_guard<std::mutex> lock(g_data_mutex);
    ImuRecord r;
    r.ts = wall_time_string();
    r.omega[0] = msg->omega[0];
    r.omega[1] = msg->omega[1];
    r.omega[2] = msg->omega[2];
    r.acc[0] = msg->acc[0];
    r.acc[1] = msg->acc[1];
    r.acc[2] = msg->acc[2];
    r.navQuat[0] = msg->navQuat[0];
    r.navQuat[1] = msg->navQuat[1];
    r.navQuat[2] = msg->navQuat[2];
    r.navQuat[3] = msg->navQuat[3];
    r.navRPY[0] = msg->navRPY[0];
    r.navRPY[1] = msg->navRPY[1];
    r.navRPY[2] = msg->navRPY[2];
    r.temp = msg->temp;
    g_imu_records.push_back(r);
}

static bool file_has_content(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return f.good() && f.peek() != std::ifstream::traits_type::eof();
}

static bool file_ends_with_blank_line(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f.good()) return false;

    f.seekg(0, std::ios::end);
    std::streamoff size = f.tellg();
    if (size < 2) return false;

    char last = '\0';
    char prev = '\0';
    f.seekg(-1, std::ios::end);
    f.get(last);
    f.seekg(-2, std::ios::end);
    f.get(prev);
    return prev == '\n' && last == '\n';
}

static std::ofstream open_csv_for_append(const std::string& path, const std::string& header) {
    const bool has_content = file_has_content(path);
    std::ofstream f(path, std::ios::app);

    if (!has_content) {
        f << header << "\n";
    } else if (!file_ends_with_blank_line(path)) {
        f << "\n";
    }

    return f;
}

static void write_joint_csv(const std::string& path) {
    std::ofstream f = open_csv_for_append(
        path,
        "timestamp,component_type,component_name,joint_id,stat,"
        "pos_high,pos_low,vel,tor");

    std::lock_guard<std::mutex> lock(g_data_mutex);
    for (const auto& r : g_joint_records) {
        f << r.ts << ","
          << r.component_type << ","
          << component_name(r.component_type) << ","
          << r.joint_id << ","
          << r.stat << ","
          << r.pos_high << ","
          << r.pos_low << ","
          << r.vel << ","
          << r.tor << "\n";
    }
    f << "\n";
    LOG(INFO) << "Joint data appended: " << path << " (" << g_joint_records.size() << " records)";
}

static void write_imu_csv(const std::string& path) {
    std::ofstream f = open_csv_for_append(
        path,
        "timestamp,omega_x,omega_y,omega_z,acc_x,acc_y,acc_z,"
        "quat_w,quat_x,quat_y,quat_z,roll,pitch,yaw,temp");

    std::lock_guard<std::mutex> lock(g_data_mutex);
    for (const auto& r : g_imu_records) {
        f << r.ts << ","
          << r.omega[0] << "," << r.omega[1] << "," << r.omega[2] << ","
          << r.acc[0] << "," << r.acc[1] << "," << r.acc[2] << ","
          << r.navQuat[0] << "," << r.navQuat[1] << ","
          << r.navQuat[2] << "," << r.navQuat[3] << ","
          << r.navRPY[0] << "," << r.navRPY[1] << "," << r.navRPY[2] << ","
          << r.temp << "\n";
    }
    f << "\n";
    LOG(INFO) << "IMU data appended: " << path << " (" << g_imu_records.size() << " records)";
}

static void write_status_csv(const std::string& path) {
    std::ofstream f = open_csv_for_append(path, "timestamp,state_code,state_name,type");

    std::lock_guard<std::mutex> lock(g_data_mutex);
    for (const auto& r : g_status_records) {
        f << r.ts << ","
          << static_cast<int>(r.state) << ","
          << state_name(r.state) << ","
          << static_cast<int>(r.type) << "\n";
    }
    f << "\n";
    LOG(INFO) << "Status data appended: " << path << " (" << g_status_records.size() << " records)";
}

static void write_all_csv() {
    const std::filesystem::path output_dir = "nix_lcm_sub_dir";
    std::filesystem::create_directories(output_dir);

    write_joint_csv((output_dir / "joint_data.csv").string());
    write_imu_csv((output_dir / "imu_data.csv").string());
    write_status_csv((output_dir / "robot_status.csv").string());

    LOG(INFO) << "CSV files written to " << output_dir.string();
}

int main(int argc, char* argv[]) {
    if (argc > 1 && (std::strcmp(argv[1], "--help") == 0 || std::strcmp(argv[1], "-h") == 0)) {
        std::cout << "Usage: " << argv[0] << "\n"
                  << "Passive NIX LCM subscriber. Does not send robot or joint commands.\n"
                  << "Writes CSV files to nix_lcm_sub_dir when stopped with Ctrl-C.\n";
        return 0;
    }
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    FLAGS_stderrthreshold = 0;
    FLAGS_minloglevel = 0;
    google::InitGoogleLogging(argv[0]);

    SdkRobotManager manager;
    manager.Init();
    manager.SetRobotStatusCb(on_robot_status);
    manager.SetJointDataCb(on_joint_data);
    manager.SetImuDataCb(on_imu_data);

    LOG(INFO) << "Passive recorder started. It will NOT enter or exit DEBUG state.";
    LOG(INFO) << "Start nix_joint_cmd in another terminal, then press Ctrl-C here to save CSV.";

    int last_joint = 0;
    int last_imu = 0;
    int last_status = 0;
    while (g_running) {
        sleep(1);

        const int joint = g_joint_msg_count.load();
        const int imu = g_imu_msg_count.load();
        const int status = g_status_msg_count.load();

        LOG(INFO) << "recv rate approx: lcm_joint_data=" << (joint - last_joint)
                  << " msg/s, lcm_imu_data=" << (imu - last_imu)
                  << " msg/s, status=" << (status - last_status)
                  << " msg/s, state=" << state_name(static_cast<int8_t>(g_robot_state.load()))
                  << "(" << g_robot_state.load() << ")"
                  << ", type=" << g_robot_type.load();

        last_joint = joint;
        last_imu = imu;
        last_status = status;
    }

    LOG(WARNING) << "Stopping passive recorder and writing CSV files...";
    write_all_csv();
    google::ShutdownGoogleLogging();
    return 0;
}
