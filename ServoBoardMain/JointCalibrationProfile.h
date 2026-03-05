#ifndef JOINT_CALIBRATION_PROFILE_H
#define JOINT_CALIBRATION_PROFILE_H

#include <Arduino.h>
#include "CalibrationTask.h"

extern const JointCalibrationConfig kJointCalibrationProfile[ENCODER_TOTAL_NUM];

void loadJointCalibrationProfile(JointCalibrationConfig* dst, uint8_t count);

#endif // JOINT_CALIBRATION_PROFILE_H
