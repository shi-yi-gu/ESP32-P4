#include "JointCalibrationProfile.h"

// Default calibration values for joints without dedicated tuning.
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

// Joint 0~3: dedicated entries for real hardware tuning.
// You can manually update these four blocks with measured values.
static const JointCalibrationConfig kJoint0Calib = {
    4000, 120, 120, 240, 180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint1Calib = {
    4000, 120, 120, 240, 180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint2Calib = {
    4000, 120, 120, 240, 180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint3Calib = {
    4000, 120, 120, 240, 180, 10, 150, 10, 20, 5000
};

const JointCalibrationConfig kJointCalibrationProfile[ENCODER_TOTAL_NUM] = {
    // joint 0 ~ joint 6
    kJoint0Calib,       // 0
    kJoint1Calib,       // 1
    kJoint2Calib,       // 2
    kJoint3Calib,       // 3
    kDefaultJointCalib, // 4
    kDefaultJointCalib, // 5
    kDefaultJointCalib, // 6

    // joint 7 ~ joint 13
    kDefaultJointCalib, // 7
    kDefaultJointCalib, // 8
    kDefaultJointCalib, // 9
    kDefaultJointCalib, // 10
    kDefaultJointCalib, // 11
    kDefaultJointCalib, // 12
    kDefaultJointCalib, // 13

    // joint 14 ~ joint 20
    kDefaultJointCalib, // 14
    kDefaultJointCalib, // 15
    kDefaultJointCalib, // 16
    kDefaultJointCalib, // 17
    kDefaultJointCalib, // 18
    kDefaultJointCalib, // 19
    kDefaultJointCalib  // 20
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
