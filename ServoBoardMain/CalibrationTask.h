#ifndef CALIBRATION_TASK_H
#define CALIBRATION_TASK_H

#include <Arduino.h>
#include "TaskSharedData.h"

// 标定任务资源配置：当前阶段任务本体为占位实现，保留参数用于后续恢复自动标定。
#define CALIB_TASK_STACK_SIZE 6144
#define TASK_CALIB_PRIORITY   2

// 上位机可见的标定状态码（与通信层约定保持一致）。
#define CALIB_STATUS_IDLE      0
#define CALIB_STATUS_RUNNING   1
#define CALIB_STATUS_SUCCESS   2
#define CALIB_STATUS_FAILED    3

// 单关节标定配置：
// - angleScope/bottomReserved/topReserved/angleReserved 以编码器计数为单位
// - loadThreshold 为舵机负载阈值，超过阈值视为触碰机械限位
// - tightenStep/speed/acc 控制逼近限位时的步长与运动参数
// - settleMs/maxSearchMs 控制每步等待和整体超时
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

// 单关节标定输出结果：
// - success 表示该关节标定是否有效
// - encoderMax 为触及限位时的编码器计数
// - offset 为后续映射使用的零偏
// - angleMax 为可用角度范围上限（计数单位）
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

// 加载默认标定参数（来源于 JointCalibrationProfile）。
void initDefaultCalibrationConfig(JointCalibrationConfig* cfg, uint8_t count);

// 测试辅助入口：
// 使用手工填入的 encoderMax 推导 offset，且不执行自动标定运动。
void initManualCalibrationForTest(void);

// 执行单关节自动标定（正式流程入口）：
// 返回 true 表示找到有效限位并回写 out，false 表示参数/通信/超时等失败。
bool runSingleJointCalibration(TaskSharedData_t* sharedData,
                               uint8_t jointIndex,
                               const JointCalibrationConfig& cfg,
                               JointCalibrationResult* out);

// 标定任务主函数：当前测试阶段为占位实现，后续可恢复完整自动标定调度。
void taskServoCalibration(void* parameter);

#endif // CALIBRATION_TASK_H
