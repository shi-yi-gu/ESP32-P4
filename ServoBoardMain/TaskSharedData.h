#ifndef TASK_SHARED_DATA_H
#define TASK_SHARED_DATA_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// 任务共享数据中心：
// 1) 统一定义跨任务消息结构；
// 2) 统一维护队列、互斥锁和控制状态；
// 3) 作为 SystemTask 初始化与各业务任务通信的公共边界。

// 系统总关节/编码器通道数。
#define ENCODER_TOTAL_NUM 21

// TWAI（CAN）引脚定义。
#define TWAI_TX_PIN 47
#define TWAI_RX_PIN 48

// FreeRTOS 任务优先级（数值越大优先级越高）。
#define TASK_UPPER_COMM_PRIORITY 1
#define TASK_CAN_COMM_PRIORITY 3
#define TASK_SOLVER_PRIORITY 4

// FreeRTOS 任务栈大小（字节）。
#define UPPER_COMM_TASK_STACK_SIZE 8192
#define CAN_COMM_TASK_STACK_SIZE 4096
#define SOLVER_TASK_STACK_SIZE 8192

// 下位机到 CAN 总线的控制命令（标准数据帧，payload 最大 8 字节）。
typedef struct {
    uint8_t cmdID;      // CAN 帧 ID
    uint8_t payload[8]; // 命令数据
    uint8_t len;        // 数据长度（len <= 8）
} RemoteCommand_t;

// 原始编码器快照（由 CanCommTask 聚合后发布）。
typedef struct {
    uint16_t encoderValues[ENCODER_TOTAL_NUM]; // 原始编码器计数值
    uint8_t errorFlags[ENCODER_TOTAL_NUM];     // 每路错误标志（0/1）
    uint32_t errorBitmap;                      // 汇总错误位图（按协议定义）
    uint32_t timestamp;                        // 时间戳（ms）
    bool isValid;                              // 快照有效位
} RemoteSensorData_t;

// 磁编码器值映射后的角度快照（供上位机与控制链路使用）。
typedef struct {
    int16_t angleValues[ENCODER_TOTAL_NUM]; // 映射角度值（协议单位）
    uint8_t validFlags[ENCODER_TOTAL_NUM];  // 每路角度有效位（0/1）
    uint32_t timestamp;                     // 时间戳（ms）
    bool isValid;                           // 整包有效位
} MappedAngleData_t;

// 舵机控制命令（Solver/业务任务下发给舵机总线）。
typedef struct {
    uint8_t cmdType;   // 命令类型
    uint8_t servoId;   // 舵机 ID
    int16_t position;  // 目标位置（舵机协议原始单位）
    uint16_t speed;    // 目标速度（舵机协议原始单位）
    uint8_t busIndex;  // 总线索引
} ServoCommand_t;

// 舵机状态回读（单舵机维度）。
typedef struct {
    uint8_t servoId;     // 舵机 ID
    int16_t position;    // 当前位置（原始单位）
    int16_t speed;       // 当前速度（原始单位）
    int16_t load;        // 当前负载（原始单位）
    uint8_t voltage;     // 电压（协议单位）
    uint8_t temperature; // 温度（摄氏度）
} ServoStatus_t;

// 舵机角度快照（全关节维度）。
typedef struct {
    int32_t servoAngles[ENCODER_TOTAL_NUM]; // 舵机角度（协议原始单位）
    uint8_t onlineStatus[ENCODER_TOTAL_NUM]; // 在线状态（0/1）
    uint32_t timestamp; // 时间戳（ms）
} ServoAngleData_t;

// 舵机遥测快照（全关节维度）。
typedef struct {
    int16_t speed[ENCODER_TOTAL_NUM];        // 速度（原始单位）
    int16_t load[ENCODER_TOTAL_NUM];         // 负载（原始单位）
    uint8_t voltage[ENCODER_TOTAL_NUM];      // 电压（协议单位）
    uint8_t temperature[ENCODER_TOTAL_NUM];  // 温度（摄氏度）
    uint8_t onlineStatus[ENCODER_TOTAL_NUM]; // 在线状态（0/1）
    uint32_t timestamp;                      // 时间戳（ms）
} ServoTelemetryData_t;

// 单关节调试数据（用于上位机观察控制链路关键中间量）。
typedef struct {
    float targetDeg;      // 目标角（deg）
    float magActualDeg;   // 磁编码器实际角（deg）
    float loop1Output;    // 外环输出
    float loop2Actual;    // 内环反馈量
    float loop2Output;    // 内环输出
    uint32_t timestamp;   // 时间戳（ms）
    uint8_t jointIndex;   // 关节索引
    uint8_t valid;        // 当前样本有效位（0/1）
} JointDebugData_t;

// 任务共享句柄与控制状态总表。
typedef struct {
    QueueHandle_t cmdQueue;    // 舵机命令队列
    QueueHandle_t statusQueue; // 舵机状态队列

    QueueHandle_t canTxQueue;          // CAN 下行命令队列
    QueueHandle_t canRxQueue;          // CAN 上行传感器队列
    QueueHandle_t servoAngleQueue;     // 舵机角度快照队列
    QueueHandle_t servoTelemetryQueue; // 舵机遥测快照队列
    QueueHandle_t mappedAngleQueue;    // 映射角快照队列
    QueueHandle_t jointDebugQueue;     // 单关节调试数据队列

    float targetAngles[ENCODER_TOTAL_NUM];      // 目标角数组（deg）
    SemaphoreHandle_t targetAnglesMutex;        // targetAngles 互斥锁

    volatile uint8_t control_enabled;               // 控制使能（0=停控，1=使能）
    int32_t calib_zero_raw_cache[ENCODER_TOTAL_NUM]; // 校准零点原始值缓存
    volatile uint8_t calib_zero_raw_valid;          // 校准缓存有效位（0/1）
} TaskSharedData_t;

#endif // TASK_SHARED_DATA_H
