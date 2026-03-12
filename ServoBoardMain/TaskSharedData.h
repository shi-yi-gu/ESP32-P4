#ifndef TASK_SHARED_DATA_H
#define TASK_SHARED_DATA_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#define ENCODER_TOTAL_NUM 21

#define TWAI_TX_PIN 47
#define TWAI_RX_PIN 48

#define TASK_UPPER_COMM_PRIORITY 1
#define TASK_CAN_COMM_PRIORITY 3
#define TASK_SOLVER_PRIORITY 4

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
    uint8_t errorFlags[ENCODER_TOTAL_NUM];
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
    int32_t servoAngles[ENCODER_TOTAL_NUM];
    uint8_t onlineStatus[ENCODER_TOTAL_NUM];
    uint32_t timestamp;
} ServoAngleData_t;

typedef struct {
    int16_t speed[ENCODER_TOTAL_NUM];
    int16_t load[ENCODER_TOTAL_NUM];
    uint8_t voltage[ENCODER_TOTAL_NUM];
    uint8_t temperature[ENCODER_TOTAL_NUM];
    uint8_t onlineStatus[ENCODER_TOTAL_NUM];
    uint32_t timestamp;
} ServoTelemetryData_t;

typedef struct {
    float targetDeg;
    float magActualDeg;
    float loop1Output;
    float loop2Actual;
    float loop2Output;
    uint32_t timestamp;
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
} TaskSharedData_t;

#endif // TASK_SHARED_DATA_H
