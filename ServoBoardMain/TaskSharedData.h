#ifndef TASK_SHARED_DATA_H
#define TASK_SHARED_DATA_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "ServoManager.h"
#include <freertos/semphr.h>  // 【新增】互斥锁头文件
#include "CanCommTask.h"



// ============ 常量定义 ============
#define ENCODER_TOTAL_NUM       21




// --- 【新增】CAN / TWAI 配置 ---
#define TWAI_TX_PIN 47  // 请根据实际 P4 硬件连接修改
#define TWAI_RX_PIN 48  // 请根据实际 P4 硬件连接修改


// 任务优先级定义
#define TASK_UPPER_COMM_PRIORITY 1
// #define TASK_SERVO_CTRL_PRIORITY 2  //已弃用
#define TASK_CAN_COMM_PRIORITY   3  // 【新增】CAN通信优先级
#define TASK_SOLVER_PRIORITY      4   // 【新增】解算任务优先级（最高，保证实时性）

// ============ 任务堆栈 ============
#define UPPER_COMM_TASK_STACK_SIZE 8192
#define CAN_COMM_TASK_STACK_SIZE   4096
#define SOLVER_TASK_STACK_SIZE     8192  // 【新增】解算任务堆栈



// --- 【新增】发送给 ESP32-S3 的指令结构 ---
typedef struct {
    uint8_t cmdID;
    uint8_t payload[8];
    uint8_t len;
} RemoteCommand_t;



// --- 【新增】发送给 ESP32-S3 的指令结构 ---
typedef struct {
    uint16_t encoderValues[ENCODER_TOTAL_NUM];
    uint8_t  errorFlags[ENCODER_TOTAL_NUM];
    uint32_t errorBitmap;
    uint32_t timestamp;
    bool     isValid;
} RemoteSensorData_t;

// Mapped magnetic encoder angle (after direction/unwarp/offset).
typedef struct {
    int16_t angleValues[ENCODER_TOTAL_NUM];
    uint8_t validFlags[ENCODER_TOTAL_NUM];
    uint32_t timestamp;
    bool isValid;
} MappedAngleData_t;

// 舵机指令
typedef struct {
    uint8_t cmdType;
    uint8_t servoId;
    int16_t position;
    uint16_t speed;
    uint8_t busIndex;
} ServoCommand_t;

// 舵机状态
typedef struct {
    uint8_t servoId;
    int16_t position;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
} ServoStatus_t;

// 舵机角度数据结构体
typedef struct {
    int32_t servoAngles[ENCODER_TOTAL_NUM]; // 舵机绝对角度值
    uint8_t onlineStatus[ENCODER_TOTAL_NUM]; // 舵机在线状态
    uint32_t timestamp; // 数据时间戳
} ServoAngleData_t;

// 任务间共享的数据结构
typedef struct {
    QueueHandle_t cmdQueue;
    QueueHandle_t statusQueue;

        // 【新增】CAN 通信队列
    QueueHandle_t canTxQueue;    // 存放要发给 S3 的指令
    QueueHandle_t canRxQueue;    // 存放从 S3 收到的解包数据
    
    // 【新增】舵机角度数据队列
    QueueHandle_t servoAngleQueue; // 存放舵机角度数据
    QueueHandle_t mappedAngleQueue; // 存放映射后的磁编角度数据
    
     // 【新增】目标角度数组 + 互斥锁
    // 由 UpperCommTask 写入，由 taskSolver 读取
    float targetAngles[ENCODER_TOTAL_NUM];
    SemaphoreHandle_t targetAnglesMutex;
} TaskSharedData_t;

#endif
