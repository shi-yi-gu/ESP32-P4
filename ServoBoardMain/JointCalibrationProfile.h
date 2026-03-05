#ifndef JOINT_CALIBRATION_PROFILE_H
#define JOINT_CALIBRATION_PROFILE_H

#include <Arduino.h>
#include "CalibrationTask.h"

// 每个关节的校准参数模板
// 索引顺序为 0 到 20
extern const JointCalibrationConfig kJointCalibrationProfile[ENCODER_TOTAL_NUM];

// 将模板参数拷贝到运行时配置
void loadJointCalibrationProfile(JointCalibrationConfig* dst, uint8_t count);

#endif
