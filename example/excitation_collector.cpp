#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <glog/logging.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <cstring>
#include <mutex>
#include <cmath>

// ── Joint mapping: CSV name → (SdkComponentType, local_joint_id, Kp, Kd) ──
struct JointMapping {
    SdkComponentType component;
    int local_id;
    float kp;
    float kd;
};

// From send_joint_cmds2.cpp — joint limits and gains indexed by global_joint_id
struct JointParam { float lower; float upper; float kp; float kd; };

static const JointParam s_joint_params[] = {
    { -3, 3, 60.0, 2.0 },     // 0:  torso_joint
    { -1.3, 1.3, 60.0, 2.0 }, // 1:  head_joint
    { -3, 3, 60.0, 2.0 },     // 2:  left_shoulder_pitch_joint
    { -0.12, 2.3, 60.0, 2.0 },// 3:  left_shoulder_roll_joint
    { -1.9, 1.9, 60.0, 2.0 }, // 4:  left_shoulder_yaw_joint
    { -0.83, 1.57, 60.0, 2.0 },//5:  left_elbow_joint
    { -2.3, 2.3, 60.0, 2.0 }, // 6:  left_wrist_yaw_joint
    { -1.2, 1.2, 60.0, 2.0 }, // 7:  left_wrist_pitch_joint
    { -1.2, 1.2, 60.0, 2.0 }, // 8:  left_wrist_roll_joint
    { -3, 3, 60.0, 2.0 },     // 9:  right_shoulder_pitch_joint
    { -2.3, 0.12, 60.0, 2.0 },//10:  right_shoulder_roll_joint
    { -1.9, 1.9, 60.0, 2.0 }, //11:  right_shoulder_yaw_joint
    { -0.83, 1.57, 60.0, 2.0 },//12: right_elbow_joint
    { -2.3, 2.3, 60.0, 2.0 }, //13:  right_wrist_yaw_joint
    { -1.2, 1.2, 60.0, 2.0 }, //14:  right_wrist_pitch_joint
    { -1.2, 1.2, 60.0, 2.0 }, //15:  right_wrist_roll_joint
    { -1.5, 1.5, 200.0, 6.0 },//16:  left_hip_pitch_joint
    { -0.25, 2.8, 200.0, 6.0 },//17: left_hip_roll_joint
    { -1.2, 2.6, 200.0, 6.0 },//18: left_hip_yaw_joint
    { 0.0, 2.27, 200.0, 6.0 },//19: left_knee_joint
    { -1.0, 0.43, 80.0, 2.0 },//20: left_ankle_pitch_joint
    { -0.43, 0.43, 80.0, 2.0 },//21: left_ankle_roll_joint
    { -1.5, 1.5, 200.0, 6.0 },//22: right_hip_pitch_joint
    { -2.8, 0.25, 200.0, 6.0 },//23: right_hip_roll_joint
    { -2.6, 1.2, 200.0, 6.0 },//24: right_hip_yaw_joint
    { 0.0, 2.27, 200.0, 6.0 },//25: right_knee_joint
    { -1.0, 0.43, 80.0, 2.0 },//26: right_ankle_pitch_joint
    { -0.43, 0.43, 80.0, 2.0 },//27: right_ankle_roll_joint
};

// CSV joint name → (global_joint_id, mapping)
static const std::map<std::string, std::pair<int, JointMapping>> s_csv_joint_map = {
    {"torso_joint",                 {0,  {SdkComponentType::WAIST, 0, 60.0,  2.0}}},
    {"left_shoulder_pitch_joint",   {2,  {SdkComponentType::ARM_L,  0, 60.0,  2.0}}},
    {"left_shoulder_roll_joint",    {3,  {SdkComponentType::ARM_L,  1, 60.0,  2.0}}},
    {"left_shoulder_yaw_joint",     {4,  {SdkComponentType::ARM_L,  2, 60.0,  2.0}}},
    {"left_elbow_joint",            {5,  {SdkComponentType::ARM_L,  3, 60.0,  2.0}}},
    {"right_shoulder_pitch_joint",  {9,  {SdkComponentType::ARM_R,  0, 60.0,  2.0}}},
    {"right_shoulder_roll_joint",   {10, {SdkComponentType::ARM_R,  1, 60.0,  2.0}}},
    {"right_shoulder_yaw_joint",    {11, {SdkComponentType::ARM_R,  2, 60.0,  2.0}}},
    {"right_elbow_joint",           {12, {SdkComponentType::ARM_R,  3, 60.0,  2.0}}},
    {"left_hip_pitch_joint",        {16, {SdkComponentType::LEG_L,  0, 200.0, 6.0}}},
    {"left_hip_roll_joint",         {17, {SdkComponentType::LEG_L,  1, 200.0, 6.0}}},
    {"left_hip_yaw_joint",          {18, {SdkComponentType::LEG_L,  2, 200.0, 6.0}}},
    {"left_knee_joint",             {19, {SdkComponentType::LEG_L,  3, 200.0, 6.0}}},
    {"left_ankle_pitch_joint",      {20, {SdkComponentType::LEG_L,  4, 80.0,  2.0}}},
    {"left_ankle_roll_joint",       {21, {SdkComponentType::LEG_L,  5, 80.0,  2.0}}},
    {"right_hip_pitch_joint",       {22, {SdkComponentType::LEG_R,  0, 200.0, 6.0}}},
    {"right_hip_roll_joint",        {23, {SdkComponentType::LEG_R,  1, 200.0, 6.0}}},
    {"right_hip_yaw_joint",         {24, {SdkComponentType::LEG_R,  2, 200.0, 6.0}}},
    {"right_knee_joint",            {25, {SdkComponentType::LEG_R,  3, 200.0, 6.0}}},
    {"right_ankle_pitch_joint",     {26, {SdkComponentType::LEG_R,  4, 80.0,  2.0}}},
    {"right_ankle_roll_joint",      {27, {SdkComponentType::LEG_R,  5, 80.0,  2.0}}},
};

// ── Trajectory data ──
struct TrajectoryPoint {
    double time;
    std::vector<float> positions;  // one per joint (in s_csv_joint_map order)
    std::vector<float> velocities;
    std::vector<float> accelerations;
};

// ── Feedback storage ──
struct FeedbackRecord {
    double timestamp;
    int16_t component_type;
    int16_t joint_id;
    float pos;
    float vel;
    float tor;
};

static std::mutex s_feedback_mutex;
static std::vector<FeedbackRecord> s_feedback_data;

static void feedback_callback(const sdk_lcmt_joint_datasets* datasets) {
    std::lock_guard<std::mutex> lock(s_feedback_mutex);
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    double t = std::chrono::duration<double>(now).count();
    for (int i = 0; i < datasets->datasets_num; ++i) {
        const auto& d = datasets->datasets[i];
        s_feedback_data.push_back({t, d.component_type, d.joint_id,
                                   d.pos_low, d.vel, d.tor});
    }
}

// ── CSV parsing ──
static std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    for (size_t i = 0; i < line.size(); ++i) {
        if (line[i] == ',') {
            fields.push_back(field);
            field.clear();
        } else {
            field += line[i];
        }
    }
    if (!field.empty()) fields.push_back(field);
    return fields;
}

static double parse_double(const std::string& s) {
    return std::stod(s);
}

static bool load_trajectory(const std::string& path,
                            std::vector<TrajectoryPoint>& trajectory,
                            std::vector<std::string>& joint_names) {
    std::ifstream file(path);
    if (!file.is_open()) {
        LOG(ERROR) << "Cannot open trajectory file: " << path;
        return false;
    }

    std::string header_line;
    if (!std::getline(file, header_line)) {
        LOG(ERROR) << "Empty trajectory file";
        return false;
    }

    std::vector<std::string> headers = split_csv_line(header_line);
    if (headers.empty() || headers[0] != "time") {
        LOG(ERROR) << "Invalid header: first column must be 'time'";
        return false;
    }

    // Parse header to identify joints and their column indices
    // Each joint has 3 columns: q_<joint>, dq_<joint>, ddq_<joint>
    struct JointCols { int q_idx; int dq_idx; int ddq_idx; std::string name; };
    std::vector<JointCols> joint_cols;

    for (size_t i = 1; i < headers.size(); ++i) {
        const auto& h = headers[i];
        // Extract joint name: "q_left_hip_pitch_joint" → "left_hip_pitch_joint"
        //                        "dq_left_hip_pitch_joint" → "left_hip_pitch_joint"
        std::string suffix;
        std::string prefix;
        if (h.compare(0, 2, "q_") == 0) {
            prefix = "q_";
            suffix = h.substr(2);
        } else if (h.compare(0, 3, "dq_") == 0) {
            prefix = "dq_";
            suffix = h.substr(3);
        } else if (h.compare(0, 4, "ddq_") == 0) {
            prefix = "ddq_";
            suffix = h.substr(4);
        } else {
            LOG(WARNING) << "Skipping unrecognized column: " << h;
            continue;
        }

        // Find or create entry for this joint
        auto it = std::find_if(joint_cols.begin(), joint_cols.end(),
            [&suffix](const JointCols& jc) { return jc.name == suffix; });
        if (it == joint_cols.end()) {
            JointCols jc;
            jc.name = suffix;
            jc.q_idx = -1; jc.dq_idx = -1; jc.ddq_idx = -1;
            joint_cols.push_back(jc);
            it = joint_cols.end() - 1;
        }

        if (prefix == "q_")       it->q_idx = i;
        else if (prefix == "dq_")  it->dq_idx = i;
        else if (prefix == "ddq_") it->ddq_idx = i;
    }

    joint_names.clear();
    for (const auto& jc : joint_cols) {
        joint_names.push_back(jc.name);
    }
    LOG(INFO) << "Found " << joint_names.size() << " joints in CSV";

    // Parse data rows
    std::string line;
    while (std::getline(file, line)) {
        auto fields = split_csv_line(line);
        if (fields.size() < headers.size()) continue;

        TrajectoryPoint pt;
        pt.time = parse_double(fields[0]);
        pt.positions.resize(joint_cols.size());
        pt.velocities.resize(joint_cols.size());
        pt.accelerations.resize(joint_cols.size());

        for (size_t j = 0; j < joint_cols.size(); ++j) {
            if (joint_cols[j].q_idx > 0)
                pt.positions[j] = parse_double(fields[joint_cols[j].q_idx]);
            if (joint_cols[j].dq_idx > 0)
                pt.velocities[j] = parse_double(fields[joint_cols[j].dq_idx]);
            if (joint_cols[j].ddq_idx > 0)
                pt.accelerations[j] = parse_double(fields[joint_cols[j].ddq_idx]);
        }
        trajectory.push_back(pt);
    }

    LOG(INFO) << "Loaded " << trajectory.size() << " trajectory points";
    return true;
}

// ── Save feedback ──
static void save_feedback(const std::string& path) {
    std::lock_guard<std::mutex> lock(s_feedback_mutex);
    std::ofstream file(path);
    if (!file.is_open()) {
        LOG(ERROR) << "Cannot write feedback file: " << path;
        return;
    }
    file << "timestamp,ComponentType,JointID,Position,ActualVel,Torque\n";
    for (const auto& r : s_feedback_data) {
        file << r.timestamp << ","
             << (int)r.component_type << ","
             << (int)r.joint_id << ","
             << r.pos << ","
             << r.vel << ","
             << r.tor << "\n";
    }
    LOG(INFO) << "Feedback saved to " << path << " (" << s_feedback_data.size() << " records)";
}

// ── Main ──
int main(int argc, char* argv[]) {
    FLAGS_stderrthreshold = 0;
    FLAGS_minloglevel = 0;
    google::InitGoogleLogging(argv[0]);

    std::string robot_name = "nix2";
    if (argc > 1) {
        robot_name = argv[1];
    }

    std::string dataset_dir = "dataset/" + robot_name;
    std::string traj_path = dataset_dir + "/excitation_multi_motor.csv";
    std::string feedback_path = dataset_dir + "/feedback.csv";

    // Load trajectory
    std::vector<TrajectoryPoint> trajectory;
    std::vector<std::string> joint_names;
    if (!load_trajectory(traj_path, trajectory, joint_names)) {
        LOG(FATAL) << "Failed to load trajectory";
        return 1;
    }

    // Build joint mapping for each trajectory joint
    // Indexed by trajectory column index
    struct TrajJointInfo {
        int joint_idx;  // index into joint_cols
        int global_id;
        JointMapping mapping;
    };
    std::vector<TrajJointInfo> active_joints;
    for (size_t i = 0; i < joint_names.size(); ++i) {
        auto it = s_csv_joint_map.find(joint_names[i]);
        if (it != s_csv_joint_map.end()) {
            active_joints.push_back({(int)i, it->second.first, it->second.second});
        } else {
            LOG(WARNING) << "Joint not mapped to SDK: " << joint_names[i];
        }
    }
    LOG(INFO) << "Active joints: " << active_joints.size();

    // Init robot
    SdkRobotManager manager;
    manager.Init();
    sleep(3);

    LOG(INFO) << "Switching to RESET state...";
    manager.SendRobotCmd(SdkStateType::RESET);
    sleep(10);

    LOG(INFO) << "Switching to STAND state...";
    manager.SendRobotCmd(SdkStateType::STAND);
    sleep(10);

    LOG(INFO) << "Enabling SDK control mode...";
    manager.SendModeCmd(1);
    sleep(2);

    // Register feedback callback
    manager.SetJointDataCb(feedback_callback);
    LOG(INFO) << "Feedback callback registered";

    // Execute trajectory
    LOG(INFO) << "Starting trajectory execution (" << trajectory.size() << " points)...";
    double prev_time = 0.0;
    auto t0 = std::chrono::steady_clock::now();

    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& pt = trajectory[i];

        // Calculate time to wait
        double dt = (i == 0) ? 0.0 : (pt.time - trajectory[i-1].time);
        if (dt > 0 && dt < 1.0) {
            usleep(static_cast<useconds_t>(dt * 1e6));
        }

        // Build joint commands
        std::vector<SdkJointCmd> cmds;
        for (const auto& joint : active_joints) {
            SdkJointCmd cmd;
            cmd.component_type = static_cast<int16_t>(joint.mapping.component);
            cmd.joint_id = joint.mapping.local_id;
            cmd.ctrlWord = 3;  // position control
            cmd.tarPos = static_cast<float>(pt.positions[joint.joint_idx]);
            cmd.tarVel = 0.0f;
            cmd.tarTor = 0;
            cmd.res1 = joint.mapping.kp;
            cmd.res2 = joint.mapping.kd;
            cmds.push_back(cmd);
        }

        manager.SendJointCmds(cmds);

        if ((i + 1) % 100 == 0) {
            LOG(INFO) << "Progress: " << (i + 1) << "/" << trajectory.size();
        }
    }

    auto t1 = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(t1 - t0).count();
    LOG(INFO) << "Trajectory execution complete in " << elapsed << "s";

    // Wait for remaining feedback
    sleep(2);

    // Save feedback
    save_feedback(feedback_path);

    // Restore robot state
    LOG(INFO) << "Returning to RL control mode...";
    manager.SendModeCmd(0);
    sleep(2);

    manager.SendRobotCmd(SdkStateType::RL_LIEDOWN);
    sleep(10);

    google::ShutdownGoogleLogging();
    return 0;
}
