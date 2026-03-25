#ifndef TASK_SHARED_DATA_H
#define TASK_SHARED_DATA_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// Joint/magnetic-encoder channels.
#define ENCODER_TOTAL_NUM 21
// Servo channels (joint16 uses dual servos).
#define SERVO_TOTAL_NUM 22

// Control mode for solver output selection.
#define CONTROL_MODE_JOINT 0
#define CONTROL_MODE_DIRECT_MOTOR 1

// TWAI(CAN) pins.
#define TWAI_TX_PIN 47
#define TWAI_RX_PIN 48

// FreeRTOS priorities.
#define TASK_UPPER_COMM_PRIORITY 1
#define TASK_CAN_COMM_PRIORITY 3
#define TASK_SOLVER_PRIORITY 4

// FreeRTOS stack sizes (bytes).
#define UPPER_COMM_TASK_STACK_SIZE 8192
#define CAN_COMM_TASK_STACK_SIZE 4096
#define SOLVER_TASK_STACK_SIZE 8192

typedef struct {
    uint8_t cmdID;
    uint8_t payload[8];
    uint8_t len;
} RemoteCommand_t;

typedef struct {
    uint16_t encoderValues[ENCODER_TOTAL_NUM];
    // 1-byte error type per channel from CAN error-detail frames (0x00 means OK).
    uint8_t errorFlags[ENCODER_TOTAL_NUM];
    // Derived bitmap: bit i is set when errorFlags[i] != 0 (kept for quick checks).
    uint32_t errorBitmap;
    uint32_t timestamp;
    bool isValid;
} RemoteSensorData_t;

typedef struct {
    int16_t angleValues[ENCODER_TOTAL_NUM];
    uint8_t validFlags[ENCODER_TOTAL_NUM];
    uint32_t timestamp;
    bool isValid;
} MappedAngleData_t;

typedef struct {
    uint8_t cmdType;
    uint8_t servoId;
    int16_t position;
    uint16_t speed;
    uint8_t busIndex;
} ServoCommand_t;

typedef struct {
    uint8_t servoId;
    int16_t position;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
} ServoStatus_t;

typedef struct {
    int32_t servoAngles[SERVO_TOTAL_NUM];
    uint8_t onlineStatus[SERVO_TOTAL_NUM];
    uint32_t timestamp;
} ServoAngleData_t;

typedef struct {
    int16_t speed[SERVO_TOTAL_NUM];
    int16_t load[SERVO_TOTAL_NUM];
    uint8_t voltage[SERVO_TOTAL_NUM];
    uint8_t temperature[SERVO_TOTAL_NUM];
    uint8_t onlineStatus[SERVO_TOTAL_NUM];
    uint32_t timestamp;
} ServoTelemetryData_t;

typedef struct {
    float targetDeg;
    float magActualDeg;
    float loop1Output;
    float loop2Actual;
    float loop2Output;
    uint32_t timestamp;
    uint8_t jointIndex;
    uint8_t valid;
} JointDebugData_t;

typedef struct {
    QueueHandle_t cmdQueue;
    QueueHandle_t statusQueue;
    QueueHandle_t canTxQueue;
    QueueHandle_t canRxQueue;
    QueueHandle_t servoAngleQueue;
    QueueHandle_t servoTelemetryQueue;
    QueueHandle_t mappedAngleQueue;
    QueueHandle_t jointDebugQueue;

    float targetAngles[ENCODER_TOTAL_NUM];
    SemaphoreHandle_t targetAnglesMutex;

    // Direct-servo mode target cache, aligned with motor channel map (0..21).
    int32_t motorTargetRaw[SERVO_TOTAL_NUM];

    volatile uint8_t control_enabled;
    volatile uint8_t control_mode; // CONTROL_MODE_*
    volatile uint8_t joint16_dual_feedback_fault;

    int32_t calib_zero_raw_cache[ENCODER_TOTAL_NUM];
    volatile uint8_t calib_zero_raw_valid;
} TaskSharedData_t;

#endif // TASK_SHARED_DATA_H
