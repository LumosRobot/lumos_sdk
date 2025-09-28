#pragma once

// #include <stdint>
#include "enum.h"


#define LUMOS_LCM_URL_PORT "udpm://239.255.76.67:7667?ttl=255"

// #ifndef LOG(level)
// #define LOG(level) std::cout << #level << "... "
// #endif

// struct SdkRobotCmd {
//     int8_t state;
//     float x;
//     float y;
//     float yaw;
// };


struct SdkJointCmd {
    int16_t component_type;
    int16_t joint_id;
    int16_t ctrlWord;
    float   tarPos;
    float   tarVel;
    float   tarCur;
    float   tarTor;
    float   res1;
    float   res2;
    float   res3;
    float   res4;

    SdkJointCmd() {
        memset(this, 0, sizeof(*this));
        ctrlWord = 200;
    }
};


BETTER_ENUM(SdkComponentType, uint8_t,
    HEAD,
    ARM_L,
    ARM_R,
    HAND_L,
    HAND_R,
    WAIST,
    LEG_L,
    LEG_R,

    MAX
);


BETTER_ENUM(SdkStateType, uint8_t, 
    // NOT_A_STATE=0, 
    RESET = 1,
    STAND = 2,
    RL_WALK = 3,
    // RL_SITUP,
    // RL_LIEDOWN,
    RL_MIMIC = 6,//
    // STAND_WALK,
    // RECOVER_RL_WALK, 
    // RECOVER_PREPARE, 
    DEBUG = 10,
    RL_NAV = 11
);
