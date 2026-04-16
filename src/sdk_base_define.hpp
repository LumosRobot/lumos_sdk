#pragma once
#include <cstdint>
#include <cstring>

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


enum class SdkComponentType {
    HEAD,
    ARM_L,
    ARM_R,
    // HAND_L,
    // HAND_R,
    // ARM_FT_L,
    // ARM_FT_R,
    WAIST = 7,
    LEG_L,
    LEG_R,

    MAX
};


enum class SdkStateType {
    // NOT_A_STATE=0,
    RESET = 1,
    STAND = 2,
    RL_WALK = 3,
    // RL_SITUP,
    // RL_LIEDOWN,
    RL_MIMIC = 6,
    // STAND_WALK,
    // RECOVER_RL_WALK,
    // RECOVER_PREPARE,
    // DEBUG = 10,
    RL_NAV = 11
};
