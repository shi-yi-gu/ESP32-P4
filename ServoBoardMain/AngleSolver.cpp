#include "AngleSolver.h"
#include <string.h> // for memset if needed
#include "pid.h"
#include "TaskSharedData.h"
#include "CalibrationTask.h"
#ifndef SOLVER_DIAG_LOG_ENABLE
#define SOLVER_DIAG_LOG_ENABLE 0
#endif

// 外部实例（定义于 SystemTask.cpp）。
extern ServoBusManager servoBus0;
extern ServoBusManager servoBus1;
extern ServoBusManager servoBus2;
extern ServoBusManager servoBus3;
extern AngleSolver angleSolver;
extern JointMapItem jointMap[ENCODER_TOTAL_NUM];

// AngleSolver 类核心实现（保持行为不变）。
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

// 核心批量解算逻辑（双环 PID）。
bool AngleSolver::compute(float *targetDegs, float *magActualDegs,
                          float *servoActualDegs, int16_t *outServoPulses)
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        // 第一环（外环：位置环）
        // 目标：来自上位机规划角度
        // 实际：磁编反馈角度
        f_PID_Calculate(&_pids[i][0], targetDegs[i], magActualDegs[i]);

        // 第二环（内环：舵机环）
        float loop2_Target = _pids[i][0].Output + servoActualDegs[i];
        float loop2_Actual = servoActualDegs[i];
        f_PID_Calculate(&_pids[i][1], loop2_Target, loop2_Actual);

        // 输出舵机脉冲。
        outServoPulses[i] = (int16_t)_pids[i][1].Output;
    }
    return true;
}

static const int32_t kEncoderModulo = 16384;
static const int32_t kEncoderHalfTurn = kEncoderModulo / 2;

// 映射计数发送前做 int16 饱和裁剪。
static int16_t clampToInt16(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

// 按安装方向统一磁编读数方向。
static int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction)
{
    int32_t oriented = (int32_t)rawValue & (kEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kEncoderModulo - oriented) & (kEncoderModulo - 1);
    }
    return oriented;
}

// 原始磁编值跨圈连续化，避免 0/16384 边界跳变导致角度突变。
static int32_t unwrapEncoderToContinuous(uint8_t jointIndex,
                                         int32_t orientedRaw,
                                         int32_t* prevOrientedRaw,
                                         int32_t* continuousRaw,
                                         bool* initialized)
{
    if (!initialized[jointIndex]) {
        initialized[jointIndex] = true;
        prevOrientedRaw[jointIndex] = orientedRaw;
        continuousRaw[jointIndex] = orientedRaw;
        return continuousRaw[jointIndex];
    }

    int32_t delta = orientedRaw - prevOrientedRaw[jointIndex];
    if (delta > kEncoderHalfTurn) delta -= kEncoderModulo;
    if (delta < -kEncoderHalfTurn) delta += kEncoderModulo;

    prevOrientedRaw[jointIndex] = orientedRaw;
    continuousRaw[jointIndex] += delta;
    return continuousRaw[jointIndex];
}

// 读取每个关节校准 offset；校准失败时回退到手动 offset。
static int32_t getEncoderOffset(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) return 0;
    if (g_jointCalibResult[jointIndex].success) {
        return g_jointCalibResult[jointIndex].offset;
    }
    return g_encoderOffsetManual[jointIndex];
}

// 编码器计数转角度（deg）。
static float convertEncoderCountToDeg(int32_t encoderCount)
{
    return (float)encoderCount * 360.0f / (float)kEncoderModulo;
}

// 根据总线号获取对应 ServoBusManager 实例。
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

// Solver 任务：执行传感数据映射 + 双环解算 + 舵机下发。
// 数据流：
//   目标角度 -> sharedData.targetAngles[]（UpperCommTask 写入）
//   磁编角度 -> sharedData.canRxQueue（CanCommTask 写入）
//   舵机反馈 -> ServoBusManager.readPosition()
//   控制输出 -> ServoBusManager.writePosition()
void taskSolver(void *parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    const uint8_t bus0_ids[] = {1};
    const uint8_t bus0_count = (uint8_t)(sizeof(bus0_ids) / sizeof(bus0_ids[0]));

    float localTargets[ENCODER_TOTAL_NUM] = {0.0f};
    float magAngles[ENCODER_TOTAL_NUM] = {0.0f};
    float servoAngles[ENCODER_TOTAL_NUM] = {0.0f};
    int16_t outPulses[ENCODER_TOTAL_NUM] = {0};

    RemoteSensorData_t sensorData;
    MappedAngleData_t mappedData;
    int32_t prevOrientedRaw[ENCODER_TOTAL_NUM] = {0};
    int32_t continuousRaw[ENCODER_TOTAL_NUM] = {0};
    bool unwrapInitialized[ENCODER_TOTAL_NUM] = {false};

#if SOLVER_DIAG_LOG_ENABLE
    uint32_t lastDiagLogMs = 0;
#endif

    while (1)
    {
        // 1) 读取舵机反馈（用于内环）。
        int bus0SuccessCount = servoBus0.syncReadPositions(bus0_ids, bus0_count);
        (void)bus0SuccessCount;

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

        ServoAngleData_t servoData;
        memset(&servoData, 0, sizeof(servoData));
        servoData.timestamp = millis();

        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            uint8_t bus = jointMap[i].busIndex;
            uint8_t id = jointMap[i].servoID;
            ServoBusManager *pBus = getBusByIndex(bus);

            if (pBus && pBus->isOnline(id))
            {
                int32_t absPos = pBus->getAbsolutePosition(id);
                servoAngles[i] = (float)absPos * 360.0f / 4096.0f;
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
        xQueueOverwrite(sharedData->servoAngleQueue, &servoData);

        if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE && sensorData.isValid)
        {
            memset(&mappedData, 0, sizeof(mappedData));
            mappedData.timestamp = sensorData.timestamp;
            mappedData.isValid = true;

            for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
            {
                if (sensorData.errorFlags[i]) {
                    mappedData.validFlags[i] = 0;
                    unwrapInitialized[i] = false;
                    continue;
                }

                const int32_t orientedRaw = orientEncoderRaw(sensorData.encoderValues[i], g_encoderDirection[i]);
                const int32_t continuous = unwrapEncoderToContinuous((uint8_t)i,
                                                                     orientedRaw,
                                                                     prevOrientedRaw,
                                                                     continuousRaw,
                                                                     unwrapInitialized);
                const int32_t mappedCount = continuous - getEncoderOffset((uint8_t)i);

                mappedData.angleValues[i] = clampToInt16(mappedCount);
                mappedData.validFlags[i] = 1;
                magAngles[i] = convertEncoderCountToDeg(mappedCount);
            }

            xQueueOverwrite(sharedData->mappedAngleQueue, &mappedData);
        }

        if (xSemaphoreTake(sharedData->targetAnglesMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            memcpy(localTargets, sharedData->targetAngles, sizeof(localTargets));
            xSemaphoreGive(sharedData->targetAnglesMutex);
        }

        angleSolver.compute(localTargets, magAngles, servoAngles, outPulses);

        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            uint8_t bus = jointMap[i].busIndex;
            uint8_t id = jointMap[i].servoID;
            ServoBusManager *pBus = getBusByIndex(bus);

            if (pBus)
            {
                int16_t targetPos = constrain(outPulses[i], -30719, 30719);
                pBus->setTarget(id, targetPos, 1000, 50);
            }
        }

        servoBus0.syncWriteAll();
        servoBus1.syncWriteAll();
        servoBus2.syncWriteAll();
        servoBus3.syncWriteAll();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
