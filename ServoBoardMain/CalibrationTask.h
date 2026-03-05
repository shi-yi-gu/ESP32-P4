#ifndef CALIBRATION_TASK_H
#define CALIBRATION_TASK_H

#include <Arduino.h>
#include "TaskSharedData.h"

// 校准任务职责：
// 1) 逐关节搜索机械极限点（基于舵机负载阈值）；
// 2) 计算每个关节的 encoder offset 与可用角度范围；
// 3) 将结果写入全局结果表，供 Solver 映射与上位机显示使用。

// 任务创建参数（默认在 System_Init() 中不自动启动）。
#define CALIB_TASK_STACK_SIZE 6144
#define TASK_CALIB_PRIORITY   2

/*
// 可选任务创建片段（默认注释）：
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

// UI/上位机状态机。
#define CALIB_STATUS_IDLE      0  // 未运行
#define CALIB_STATUS_RUNNING   1  // 运行中
#define CALIB_STATUS_SUCCESS   2  // 全部关节校准成功
#define CALIB_STATUS_FAILED    3  // 存在关节校准失败

// 单关节校准配置（单位默认均为编码器计数或毫秒）。
struct JointCalibrationConfig {
    int32_t angleScope;       // 设计角度总行程（编码器计数）
    int32_t bottomReserved;   // 下端预留（避免贴底运行）
    int32_t topReserved;      // 上端预留（避免贴顶运行）
    int32_t angleReserved;    // 本关节校准后停靠到的安全角度

    int16_t loadThreshold;    // 机械触底判定负载阈值
    int16_t tightenStep;      // 每轮收紧的舵机增量命令
    uint16_t tightenSpeed;    // 收紧阶段速度参数
    uint8_t tightenAcc;       // 收紧阶段加速度参数

    uint32_t settleMs;        // 每次收紧后等待稳定时间
    uint32_t maxSearchMs;     // 单关节搜索超时时间
};

// 单关节校准输出结果。
struct JointCalibrationResult {
    bool success;             // 本关节是否校准成功
    int32_t encoderMax;       // 极限点时的磁编值（已方向统一）
    int32_t offset;           // 供映射去零使用的 offset
    int32_t angleMax;         // 扣除上下预留后的有效角度范围
};

// 运行时配置与结果（按关节索引 0~20）。
extern JointCalibrationConfig g_jointCalibConfig[ENCODER_TOTAL_NUM];
extern JointCalibrationResult g_jointCalibResult[ENCODER_TOTAL_NUM];

// 磁编安装方向：+1 正向，-1 反向。
extern int8_t g_encoderDirection[ENCODER_TOTAL_NUM];
// 手工兜底 offset（当自动校准结果不可用时供 Solver 使用）。
extern int32_t g_encoderOffsetManual[ENCODER_TOTAL_NUM];

// 从参数模板加载默认校准配置。
void initDefaultCalibrationConfig(JointCalibrationConfig* cfg, uint8_t count);

// 执行单关节校准。
bool runSingleJointCalibration(TaskSharedData_t* sharedData,
                               uint8_t jointIndex,
                               const JointCalibrationConfig& cfg,
                               JointCalibrationResult* out);

// 校准任务入口：按 0~20 依次校准并上报最终状态。
void taskServoCalibration(void* parameter);

#endif
