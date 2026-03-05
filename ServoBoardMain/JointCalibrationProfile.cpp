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

// Joints 0~3: dedicated entries for manual tuning.
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
    // joint 0 ~ 6
    kJoint0Calib,
    kJoint1Calib,
    kJoint2Calib,
    kJoint3Calib,
    kDefaultJointCalib,
    kDefaultJointCalib,
    kDefaultJointCalib,

    // joint 7 ~ 13
    kDefaultJointCalib,
    kDefaultJointCalib,
    kDefaultJointCalib,
    kDefaultJointCalib,
    kDefaultJointCalib,
    kDefaultJointCalib,
    kDefaultJointCalib,

    // joint 14 ~ 20
    kDefaultJointCalib,
    kDefaultJointCalib,
    kDefaultJointCalib,
    kDefaultJointCalib,
    kDefaultJointCalib,
    kDefaultJointCalib,
    kDefaultJointCalib
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
