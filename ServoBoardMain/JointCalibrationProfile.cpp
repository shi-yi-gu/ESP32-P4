#include "JointCalibrationProfile.h"

// 默认校准参数
// 如某关节未单独调参 可先使用该默认值
static const JointCalibrationConfig kDefaultJointCalib = {
    4000,  // angleScope
    120,   // bottomReserved
    120,   // topReserved
    240,   // angleReserved
    180,   // loadThreshold
    10,    // tightenStep
    150,   // tightenSpeed
    10,    // tightenAcc
    20,    // settleMs
    5000   // maxSearchMs
};

// 关节参数表 0 到 20
// 你后续可逐项替换为真实机械参数
const JointCalibrationConfig kJointCalibrationProfile[ENCODER_TOTAL_NUM] = {
    // joint 0  ~ joint 6
    kDefaultJointCalib,  // 0
    kDefaultJointCalib,  // 1
    kDefaultJointCalib,  // 2
    kDefaultJointCalib,  // 3
    kDefaultJointCalib,  // 4
    kDefaultJointCalib,  // 5
    kDefaultJointCalib,  // 6

    // joint 7  ~ joint 13
    kDefaultJointCalib,  // 7
    kDefaultJointCalib,  // 8
    kDefaultJointCalib,  // 9
    kDefaultJointCalib,  // 10
    kDefaultJointCalib,  // 11
    kDefaultJointCalib,  // 12
    kDefaultJointCalib,  // 13

    // joint 14 ~ joint 20
    kDefaultJointCalib,  // 14
    kDefaultJointCalib,  // 15
    kDefaultJointCalib,  // 16
    kDefaultJointCalib,  // 17
    kDefaultJointCalib,  // 18
    kDefaultJointCalib,  // 19
    kDefaultJointCalib   // 20
};

void loadJointCalibrationProfile(JointCalibrationConfig* dst, uint8_t count)
{
    if (!dst) return;

    uint8_t n = count;
    if (n > ENCODER_TOTAL_NUM) n = ENCODER_TOTAL_NUM;

    for (uint8_t i = 0; i < n; i++) {
        dst[i] = kJointCalibrationProfile[i];
    }
}
