#include "sdk_robot_manager.hpp"
#include <unistd.h>
#include <glog/logging.h>
#include "lumos_lcm_control.hpp"

static void gamehandler_cmd_handler(const lumos_lcm_control* gamehandler_cmd) {
    static uint64_t s_counter = 0;
    if (++s_counter % 100 == 0) {
        LOG(INFO) << "--- gamehandler cmd received, seq " << s_counter << "---";
        LOG(INFO) << "state: " << gamehandler_cmd->state << ", x: " << gamehandler_cmd->x << ", y: " << gamehandler_cmd->y << ", yaw: " << gamehandler_cmd->yaw;
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

    manager.SetGameHandlerCmdCb(gamehandler_cmd_handler);
    LOG(INFO) << "SetGameHandlerCmdCb done, waiting for handler...";
    sleep(1000000);

    google::ShutdownGoogleLogging();
    return 0;
}
