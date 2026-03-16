#include "JointCalibrationProfile.h"

// 14-bit magnetic encoder: 16384 counts per revolution.
static constexpr float kEncoderCountsPerRev = 16384.0f;
static constexpr int32_t degToEnc(float deg) {
    return (int32_t)(deg * kEncoderCountsPerRev / 360.0f + (deg >= 0.0f ? 0.5f : -0.5f));
}

// Default calibration values for joints without dedicated tuning.
static const JointCalibrationConfig kDefaultJointCalib = {
    degToEnc(90.0f), // angleScope (deg -> count)
    degToEnc(3.0f),  // bottomReserved (deg -> count)
    degToEnc(3.0f),  // topReserved (deg -> count)
    degToEnc(6.0f),  // angleReserved (deg -> count)
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
    degToEnc(100.0f), degToEnc(2.0f), degToEnc(2.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint1Calib = {
    degToEnc(105.0f), degToEnc(2.0f), degToEnc(2.0f), degToEnc(40.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint2Calib = {
    degToEnc(100.0f), degToEnc(2.0f), degToEnc(2.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint3Calib = {
    degToEnc(90.0f), degToEnc(2.0f), degToEnc(2.0f), degToEnc(50.0f),
    180, 10, 150, 10, 20, 5000
};
// Joint 4~7: dedicated entries for real hardware tuning.
// You can manually update these four blocks with measured values.
static const JointCalibrationConfig kJoint4Calib = {
    degToEnc(100.0f), degToEnc(2.0f), degToEnc(2.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint5Calib = {
    degToEnc(105.0f), degToEnc(2.0f), degToEnc(2.0f), degToEnc(40.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint6Calib = {
    degToEnc(100.0f), degToEnc(2.0f), degToEnc(2.0f), degToEnc(45.0f),
    180, 10, 150, 10, 20, 5000
};

static const JointCalibrationConfig kJoint7Calib = {
    degToEnc(90.0f), degToEnc(2.0f), degToEnc(2.0f), degToEnc(50.0f),
    180, 10, 150, 10, 20, 5000
};

const JointCalibrationConfig kJointCalibrationProfile[ENCODER_TOTAL_NUM] = {
    // joint 0 ~ joint 6
    kJoint0Calib,       // 0
    kJoint1Calib,       // 1
    kJoint2Calib,       // 2
    kJoint3Calib,       // 3
    kJoint4Calib, // 4
    kJoint5Calib, // 5
    kJoint6Calib, // 6

    // joint 7 ~ joint 13
    kJoint7Calib, // 7
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
