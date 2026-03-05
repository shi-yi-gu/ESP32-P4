#ifndef CALIBRATION_TASK_H
#define CALIBRATION_TASK_H

#include <Arduino.h>
#include "TaskSharedData.h"

#define CALIB_TASK_STACK_SIZE 6144
#define TASK_CALIB_PRIORITY   2

#define CALIB_STATUS_IDLE      0
#define CALIB_STATUS_RUNNING   1
#define CALIB_STATUS_SUCCESS   2
#define CALIB_STATUS_FAILED    3

struct JointCalibrationConfig {
    int32_t angleScope;
    int32_t bottomReserved;
    int32_t topReserved;
    int32_t angleReserved;

    int16_t loadThreshold;
    int16_t tightenStep;
    uint16_t tightenSpeed;
    uint8_t tightenAcc;

    uint32_t settleMs;
    uint32_t maxSearchMs;
};

struct JointCalibrationResult {
    bool success;
    int32_t encoderMax;
    int32_t offset;
    int32_t angleMax;
};

extern JointCalibrationConfig g_jointCalibConfig[ENCODER_TOTAL_NUM];
extern JointCalibrationResult g_jointCalibResult[ENCODER_TOTAL_NUM];

extern int8_t g_encoderDirection[ENCODER_TOTAL_NUM];
extern int32_t g_encoderOffsetManual[ENCODER_TOTAL_NUM];

void initDefaultCalibrationConfig(JointCalibrationConfig* cfg, uint8_t count);

// Test helper: use manual encoderMax for joints 0~3, no auto calibration movement.
void initManualCalibrationForTest(void);

bool runSingleJointCalibration(TaskSharedData_t* sharedData,
                               uint8_t jointIndex,
                               const JointCalibrationConfig& cfg,
                               JointCalibrationResult* out);

void taskServoCalibration(void* parameter);

#endif // CALIBRATION_TASK_H
