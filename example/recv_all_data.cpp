#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <chrono>
#include <ctime>
#include <mutex>
#include <sstream>
#include <vector>
#include <cmath>
#include <glog/logging.h>
#include <filesystem>


/**
 * @brief   用于从LCM接收所有数据并保存到CSV文件中，方便后续分析。包括机器人状态、关节数据和IMU数据。
 *          操作流程：
 *              1. (本地)运行config_network_lcm.sh配置路由
 *              2. (ssh远端)修改lumos_controller中enable_publish_thread_为true，编译运行
 *              3. (本地)编译并运行recv_all_data(在build目录下执行)
 *              3. 会有三个csv文件生成到build/recv_all_data_dir目录下 
 * @author  jiangbin
 * @date    2026-05-26
 */



static volatile bool g_running = true;
static std::atomic<int> g_robot_state{0};
static std::atomic<bool> g_got_status{false};
static int g_robot_type = 0;

// ── thread-safe data storage ─────────────────────────────────────
static std::mutex g_data_mutex;

struct JointRecord {
    std::string ts;
    int16_t component_type;
    int16_t joint_id;
    int16_t stat;
    float pos_high;
    int16_t num_cycles_high;
    float pos_low;
    int16_t num_cycles_low;
    float vel;
    float cur;
    float tor;
};
static std::vector<JointRecord> g_joint_records;

struct ImuRecord {
    std::string ts;
    float omega[3];
    float acc[3];
    float navQuat[4];
    float navRPY[3];
    float temp;
};
static std::vector<ImuRecord> g_imu_records;

struct StatusRecord {
    std::string ts;
    int8_t state;
    int8_t type;
};
static std::vector<StatusRecord> g_status_records;

static std::string wall_time_string() {
    // Use the desktop computer's wall-clock time, so CSV samples can be
    // compared directly with other logs instead of starting from 0.
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

// ── signal handler ────────────────────────────────────────────────
static void sigint_handler(int) { g_running = false; }

// ── state name helper ─────────────────────────────────────────────
static const char* state_name(int8_t s) {
    switch (s) {
        case 0:  return "NOT_A_STATE";
        case 1:  return "RESET";
        case 2:  return "STAND";
        case 3:  return "RL_WALK";
        case 5:  return "RL_LIEDOWN";
        case 6:  return "ST_MIMIC";
        case 11: return "RL_NAV";
        case 12: return "RL_WALK_AMP";
        case 16: return "BY_MIMIC";
        case 17: return "BFM_MIMIC";
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

// ── LCM callbacks ─────────────────────────────────────────────────
static void on_robot_status(const robot_status_lcmt* msg) {
    g_got_status = true;
    g_robot_state = msg->state;
    g_robot_type  = msg->type;
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_status_records.push_back({wall_time_string(), msg->state, msg->type});
}

static void on_joint_data(const sdk_lcmt_joint_datasets* msg) {
    std::string ts = wall_time_string();
    std::lock_guard<std::mutex> lock(g_data_mutex);
    for (int i = 0; i < msg->datasets_num; i++) {
        const auto& d = msg->datasets[i];
        g_joint_records.push_back({
            ts,
            d.component_type,
            d.joint_id,
            d.stat,
            d.pos_high,
            d.num_cycles_high,
            d.pos_low,
            d.num_cycles_low,
            d.vel,
            d.cur,
            d.tor
        });
    }
}

static void on_imu_data(const microstrain_lcmt* msg) {
    static int dbg_count = 0;
    if (++dbg_count <= 3) {
        LOG(INFO) << "[IMU DEBUG #" << dbg_count << "]"
                  << " omega=[" << msg->omega[0] << "," << msg->omega[1] << "," << msg->omega[2] << "]"
                  << " good_packets=" << msg->good_packets
                  << " temp=" << msg->temp;
    }
    std::lock_guard<std::mutex> lock(g_data_mutex);
    ImuRecord r;
    r.ts = wall_time_string();
    r.omega[0] = msg->omega[0]; r.omega[1] = msg->omega[1]; r.omega[2] = msg->omega[2];
    r.acc[0]   = msg->acc[0];   r.acc[1]   = msg->acc[1];   r.acc[2]   = msg->acc[2];
    r.navQuat[0] = msg->navQuat[0]; r.navQuat[1] = msg->navQuat[1];
    r.navQuat[2] = msg->navQuat[2]; r.navQuat[3] = msg->navQuat[3];
    r.navRPY[0] = msg->navRPY[0]; r.navRPY[1] = msg->navRPY[1]; r.navRPY[2] = msg->navRPY[2];
    r.temp = msg->temp;
    g_imu_records.push_back(r);
}

// ── wait for state transition ─────────────────────────────────────
static bool wait_state(int target, int timeout_s) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
    LOG(INFO) << "Waiting for " << state_name((int8_t)target)
              << "(" << target << ") ... timeout=" << timeout_s << "s";
    while (g_running && std::chrono::steady_clock::now() < deadline) {
        usleep(50000);
        if (g_robot_state == target) {
            LOG(INFO) << "Reached " << state_name((int8_t)target);
            return true;
        }
    }
    LOG(ERROR) << "Timeout waiting for " << state_name((int8_t)target)
               << ", current=" << state_name((int8_t)g_robot_state.load());
    return false;
}

// ── CSV writers ───────────────────────────────────────────────────
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
        // Existing logs are kept as history. Before appending this run's data,
        // make sure there is one blank line between the previous run and this run.
        f << "\n";
    }

    return f;
}

static void write_joint_csv(const std::string& path) {
    std::ofstream f = open_csv_for_append(
        path,
        "timestamp,component_type,component_name,joint_id,stat,"
        "pos_high,num_cycles_high,pos_low,num_cycles_low,vel,cur,tor");
    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        for (const auto& r : g_joint_records) {
            f << r.ts << ","
              << r.component_type << ","
              << component_name(r.component_type) << ","
              << r.joint_id << ","
              << r.stat << ","
              << r.pos_high << ","
              << r.num_cycles_high << ","
              << r.pos_low << ","
              << r.num_cycles_low << ","
              << r.vel << ","
              << r.cur << ","
              << r.tor << "\n";
        }
    }
    // Keep one blank line at the end, so the next run can append as a new block.
    f << "\n";
    LOG(INFO) << "Joint data appended: " << path << " (" << g_joint_records.size() << " records)";
}

static void write_imu_csv(const std::string& path) {
    std::ofstream f = open_csv_for_append(
        path,
        "timestamp,omega_x,omega_y,omega_z,acc_x,acc_y,acc_z,"
        "quat_w,quat_x,quat_y,quat_z,roll,pitch,yaw,temp");
    {
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
    }
    f << "\n";
    LOG(INFO) << "IMU data appended: " << path << " (" << g_imu_records.size() << " records)";
}

static void write_status_csv(const std::string& path) {
    std::ofstream f = open_csv_for_append(path, "timestamp,state_code,state_name,type");
    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        for (const auto& r : g_status_records) {
            f << r.ts << ","
              << (int)r.state << ","
              << state_name(r.state) << ","
              << (int)r.type << "\n";
        }
    }
    f << "\n";
    LOG(INFO) << "Status data appended: " << path << " (" << g_status_records.size() << " records)";
}

// ── main ──────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    (void)(argc);
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    FLAGS_stderrthreshold = 0;
    FLAGS_minloglevel = 0;
    google::InitGoogleLogging(argv[0]);

    int exit_code = 0;

    SdkRobotManager manager;
    manager.Init();
    manager.SetRobotStatusCb(on_robot_status);
    manager.SetJointDataCb(on_joint_data);
    manager.SetImuDataCb(on_imu_data);

    // Wait for LCM data.
    // In RL mode, only myIMU is published continuously.
    // lcm_robot_status and JointsData are only published in SDK mode.
    LOG(INFO) << "Waiting for LCM data (checking IMU)...";
    int imu_before = g_imu_records.size();
    for (int i = 0; i < 30; i++) { usleep(100000); }
    int imu_after = g_imu_records.size();
    LOG(INFO) << "IMU messages received: " << (imu_after - imu_before);
    LOG(INFO) << "Current robot state: " << state_name((int8_t)g_robot_state.load());
    if (imu_after == 0) {
        LOG(ERROR) << "No LCM data received — check multicast route and network.";
        exit_code = 1;
        goto save;
    }

    // ================================================================
    // Step 1: Enter SDK control mode
    // ================================================================
    LOG(INFO) << "=== Step 1: Enter SDK mode ===";
    manager.SendModeCmd(1);
    sleep(1);

    // ================================================================
    // Step 2: RESET
    // ================================================================
    LOG(INFO) << "=== Step 2: RESET ===";
    if (!g_running) goto save;
    manager.SendRobotCmd(SdkStateType::RESET);
    if (!wait_state(1, 15)) goto save;
    sleep(3);

    // ================================================================
    // Step 3: STAND
    // ================================================================
    LOG(INFO) << "=== Step 3: STAND ===";
    if (!g_running) goto save;
    manager.SendRobotCmd(SdkStateType::STAND);
    if (!wait_state(2, 15)) goto save;
    sleep(3);

    // ================================================================
    // Step 4: RL_WALK — walk and collect data
    // ================================================================
    LOG(INFO) << "=== Step 4: RL_WALK (forward 0.1 m/s, 10s) ===";
    if (!g_running) goto save;
    manager.SendRobotCmd(SdkStateType::RL_WALK, 0.1f);
    if (!wait_state(3, 10)) goto save;

    // Walk 10 seconds, collecting data
    for (int t = 0; t < 10 && g_running; t++) {
        // Safety: if robot fell out of RL_WALK, stop sending walk commands
        if (g_robot_state != 3) {
            LOG(WARNING) << "Robot left RL_WALK (state=" << state_name((int8_t)g_robot_state.load())
                         << "), stopping walk loop";
            break;
        }
        float vx = (t < 8) ? 0.1f : 0.0f;
        manager.SendRobotCmd(SdkStateType::RL_WALK, vx);
        sleep(1);
    }
    manager.SendRobotCmd(SdkStateType::RL_WALK, 0.0f);
    sleep(2);

    // ================================================================
    // Step 5: BY_MIMIC — dance #2 (policy_type=2)
    // RL_WALK → RESET → STAND → BY_MIMIC
    // ================================================================
    LOG(INFO) << "=== Step 5: BY_MIMIC (dance #2, 10s) ===";
    if (!g_running) goto save;
    manager.SendRobotCmd(SdkStateType::RESET);
    if (!wait_state(1, 15)) goto save;
    sleep(3);
    manager.SendRobotCmd(SdkStateType::STAND);
    if (!wait_state(2, 15)) goto save;
    sleep(2);
    manager.SendRobotCmd(SdkStateType::BY_MIMIC, 0, 0, 0, 2); // 修改舞蹈在这里设置！！！
    // After entering MIMIC, the robot stays in that state until we switch out
    sleep(10);

    // ================================================================
    // Step 6: STAND
    // ================================================================
    LOG(INFO) << "=== Step 6: STAND ===";
    if (!g_running) goto save;
    // RL_WALK -> RESET first (safe transition)
    manager.SendRobotCmd(SdkStateType::RESET);
    if (!wait_state(1, 15)) goto save;
    sleep(3);
    manager.SendRobotCmd(SdkStateType::STAND);
    if (!wait_state(2, 15)) goto save;
    sleep(2);

    // ================================================================
    // Step 7: RESET and exit SDK mode
    // ================================================================
    LOG(INFO) << "=== Step 7: RESET + exit SDK mode ===";
    if (!g_running) goto save;
    manager.SendRobotCmd(SdkStateType::RESET);
    if (!wait_state(1, 15)) goto save;
    sleep(3);
    manager.SendModeCmd(0);
    sleep(1);

    LOG(INFO) << "=== Data collection SUCCESS ===";

save:
    if (!g_running) {
        LOG(WARNING) << "Interrupted, returning to safe state...";
        manager.SendRobotCmd(SdkStateType::STAND);
        sleep(3);
        manager.SendRobotCmd(SdkStateType::RESET);
        sleep(3);
        manager.SendModeCmd(0);
    }

    // ── Write CSV files ────────────────────────────────────────────
    const std::filesystem::path output_dir = "recv_all_data_dir";
    std::filesystem::create_directories(output_dir);

    write_joint_csv((output_dir / "joint_data.csv").string());
    write_imu_csv((output_dir / "imu_data.csv").string());
    write_status_csv((output_dir / "robot_status.csv").string());

    LOG(INFO) << "CSV files written to " << output_dir.string();

    google::ShutdownGoogleLogging();
    return g_running ? exit_code : 1;
}
