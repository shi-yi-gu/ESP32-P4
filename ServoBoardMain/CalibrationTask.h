#ifndef CALIBRATION_TASK_H
#define CALIBRATION_TASK_H

#include <Arduino.h>
#include "TaskSharedData.h"

// Generated calibration task is kept disabled by default in System_Init().
#define CALIB_TASK_STACK_SIZE 6144
#define TASK_CALIB_PRIORITY   2

/*
// Optional task creation snippet (intentionally commented out):
TaskHandle_t taskCalibrationHandle = NULL;
xTaskCreate(
    taskServoCalibration,
    "Calib",
    CALIB_TASK_STACK_SIZE,
    &sharedData,
    TASK_CALIB_PRIORITY,
    &taskCalibrationHandle
);
*/

// Calibration status for UI/host.
#define CALIB_STATUS_IDLE      0
#define CALIB_STATUS_RUNNING   1
#define CALIB_STATUS_SUCCESS   2
#define CALIB_STATUS_FAILED    3

struct JointCalibrationConfig {
    int32_t angleScope;       // Planned motion range from mechanism drawing
    int32_t bottomReserved;   // Reserved range at lower end
    int32_t topReserved;      // Reserved range at upper end
    int32_t angleReserved;    // Parking angle after current joint calibrated

    int16_t loadThreshold;    // Threshold to detect mechanical limit
    int16_t tightenStep;      // Servo command step during rope tightening
    uint16_t tightenSpeed;    // Servo command speed during tightening
    uint8_t tightenAcc;       // Servo command acceleration during tightening

    uint32_t settleMs;        // Delay between tightening steps
    uint32_t maxSearchMs;     // Per-joint timeout for threshold detection
};

struct JointCalibrationResult {
    bool success;
    int32_t encoderMax;
    int32_t offset;
    int32_t angleMax;
};

extern JointCalibrationConfig g_jointCalibConfig[ENCODER_TOTAL_NUM];
extern JointCalibrationResult g_jointCalibResult[ENCODER_TOTAL_NUM];

// Encoder direction: +1 normal, -1 reversed.
extern int8_t g_encoderDirection[ENCODER_TOTAL_NUM];
// Manual fallback offsets in encoder counts (used when calibration result is unavailable).
extern int32_t g_encoderOffsetManual[ENCODER_TOTAL_NUM];

void initDefaultCalibrationConfig(JointCalibrationConfig* cfg, uint8_t count);
bool runSingleJointCalibration(TaskSharedData_t* sharedData,
                               uint8_t jointIndex,
                               const JointCalibrationConfig& cfg,
                               JointCalibrationResult* out);
void taskServoCalibration(void* parameter);

#endif
