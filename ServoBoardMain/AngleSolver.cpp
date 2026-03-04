#include "AngleSolver.h"
#include <string.h> // for memset if needed
#include "pid.h"
#include "TaskSharedData.h"
#ifndef SOLVER_DIAG_LOG_ENABLE
#define SOLVER_DIAG_LOG_ENABLE 0
#endif
// ============================================================
// 【新增】外部引用（定义在 SystemTask.cpp 中）
// ============================================================
extern ServoBusManager servoBus0;
extern ServoBusManager servoBus1;
extern ServoBusManager servoBus2;
extern ServoBusManager servoBus3;
extern AngleSolver angleSolver;
extern JointMapItem jointMap[ENCODER_TOTAL_NUM];

// ============================================================
// 原有 AngleSolver 类实现（保持不变）
// ============================================================

AngleSolver::AngleSolver() : _initialized(false)
{
    memset(_zeroOffsets, 0, sizeof(_zeroOffsets));
    memset(_gearRatios, 0, sizeof(_gearRatios));
    memset(_directions, 0, sizeof(_directions));
    memset(_pids, 0, sizeof(_pids));
}

void AngleSolver::init(int16_t *zeroOffsets, float *gearRatios, int8_t *directions)
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        _zeroOffsets[i] = zeroOffsets[i];
        _gearRatios[i] = gearRatios[i];
        _directions[i] = directions[i];
    }
    // resetAll();
    _initialized = true;
}



void AngleSolver::setPIDParams(float pidParams[][PID_PARAMETER_NUM])
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        PID_Init(&_pids[i][0], PID_POSITION, pidParams[0]);
        PID_Init(&_pids[i][1], PID_POSITION, pidParams[1]);
    }
}

// 核心批量解算逻辑（保持不变）
bool AngleSolver::compute(float *targetDegs, float *magActualDegs,
                          float *servoActualDegs, int16_t *outServoPulses)
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        // --- 第一环 (外环: 位置环) ---
        // 目标: 上位机规划角度
        // 实际: 磁编角度
        f_PID_Calculate(&_pids[i][0], targetDegs[i], magActualDegs[i]);

        // --- 第二环 (内环: 舵机环) ---
        float loop2_Target = _pids[i][0].Output + servoActualDegs[i];
        float loop2_Actual = servoActualDegs[i];
        f_PID_Calculate(&_pids[i][1], loop2_Target, loop2_Actual);

        // 输出脉冲
        outServoPulses[i] = (int16_t)_pids[i][1].Output;
    }
    return true;
}

// ============================================================
// 【新增】辅助函数：CAN 原始值转角度 (0-16383 -> 0-360°)
// ============================================================
static float convertCanToDeg(uint16_t rawValue)
{
    return (float)rawValue * 360.0f / 16384.0f;
}

// ============================================================
// 【新增】辅助函数：根据总线编号获取 ServoBusManager 指针
// ============================================================
static ServoBusManager *getBusByIndex(uint8_t busIndex)
{
    switch (busIndex)
    {
    case 0:
        return &servoBus0;
    case 1:
        return &servoBus1;
    case 2:
        return &servoBus2;
    case 3:
        return &servoBus3;
    default:
        return NULL;
    }
}

// ============================================================
// 【新增】taskSolver — 角度解算 + 电机控制任务
// ============================================================
// 数据流:
//   目标角度 ← sharedData.targetAngles[] (由 UpperCommTask 写入)
//   磁编角度 ← sharedData.canRxQueue     (由 CanCommTask 写入)
//   舵机反馈 ← ServoBusManager.readPosition()
//   输出脉冲 → ServoBusManager.writePosition()
// ============================================================

void taskSolver(void *parameter)
{

    // TaskSharedData_t *sharedData = (TaskSharedData_t *)parameter;
      TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;

    // 定义每条总线的舵机 ID 列表（根据你的配置：4+4+4+5）
    const uint8_t bus0_ids[] = {1};
    const uint8_t bus0_count = (uint8_t)(sizeof(bus0_ids) / sizeof(bus0_ids[0]));

    // 本地数据缓冲区
    float localTargets[ENCODER_TOTAL_NUM];
    float canAngles[ENCODER_TOTAL_NUM];
    float servoAngles[ENCODER_TOTAL_NUM];
    int16_t outPulses[ENCODER_TOTAL_NUM];
    RemoteSensorData_t sensorData;
#if SOLVER_DIAG_LOG_ENABLE
    uint32_t lastDiagLogMs = 0;
#endif
    while (1)
    {
        // ========================================
        // 步骤 1: 同步读取所有舵机位置（自动跨圈检测）
        // ========================================
        int bus0SuccessCount = servoBus0.syncReadPositions(bus0_ids, bus0_count);

#if SOLVER_DIAG_LOG_ENABLE
        uint32_t nowMs = millis();
        if (nowMs - lastDiagLogMs >= 500) {
            lastDiagLogMs = nowMs;
            Serial.printf("[Solver] bus0 read=%d online=%d abs=%ld\r\n",
                          bus0SuccessCount,
                          servoBus0.isOnline(1) ? 1 : 0,
                          (long)servoBus0.getAbsolutePosition(1));
        }
#endif

        // ========================================
        // 步骤 2: 获取多圈绝对位置并转换为角度
        // ========================================
        ServoAngleData_t servoData;
        servoData.timestamp = millis();
        
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            uint8_t bus = jointMap[i].busIndex;
            uint8_t id = jointMap[i].servoID;
            ServoBusManager *pBus = getBusByIndex(bus);

            if (pBus && pBus->isOnline(id))
            {
                // 使用多圈绝对位置（-30719 到 30719）
                int32_t absPos = pBus->getAbsolutePosition(id);
                // 转换为角度（每圈 4096 步 = 360°）
                servoAngles[i] = (float)absPos * 360.0f / 4096.0f;
                
                // 存储舵机角度数据和在线状态
                servoData.servoAngles[i] = absPos;
                servoData.onlineStatus[i] = 1;
            }
            else
            {
                servoAngles[i] = 0.0f;
                servoData.servoAngles[i] = 0;
                servoData.onlineStatus[i] = 0;
            }
        }
        
        // 发送舵机角度数据到队列
        xQueueOverwrite(sharedData->servoAngleQueue, &servoData);

        // ========================================
        // 步骤 3: 读取 CAN 磁编角度
        // ========================================
        if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE)
        {
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
            {
                canAngles[i] = convertCanToDeg(sensorData.encoderValues[i]);
            }
        }

        // ========================================
        // 步骤 4: 读取目标角度
        // ========================================
        if (xSemaphoreTake(sharedData->targetAnglesMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            memcpy(localTargets, sharedData->targetAngles, sizeof(localTargets));
            xSemaphoreGive(sharedData->targetAnglesMutex);
        }

        // ========================================
        // 步骤 5: PID 解算
        // ========================================
        angleSolver.compute(localTargets, canAngles, servoAngles, outPulses);

        // ========================================
        // 步骤 6: 同步写入所有舵机
        // ========================================
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            uint8_t bus = jointMap[i].busIndex;
            uint8_t id = jointMap[i].servoID;
            ServoBusManager *pBus = getBusByIndex(bus);

            if (pBus)
            {
                // outPulses[i] 范围：-30719 到 30719
                int16_t targetPos = constrain(outPulses[i], -30719, 30719);
                pBus->setTarget(id, targetPos, 1000, 50);
            }
        }

        // 统一发送
        servoBus0.syncWriteAll();
        servoBus1.syncWriteAll();
        servoBus2.syncWriteAll();
        servoBus3.syncWriteAll();

        // ========================================
        // 步骤 7: 控制周期延时（100Hz）
        // ========================================
        vTaskDelay(pdMS_TO_TICKS(10));

    }
}
