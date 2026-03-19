#include "AngleSolver.h"

#include <string.h>

#include "pid.h"
#include "TaskSharedData.h"
#include "CalibrationTask.h"

#ifndef SOLVER_DIAG_LOG_ENABLE
#define SOLVER_DIAG_LOG_ENABLE 0
#endif

extern ServoBusManager servoBus0;
extern ServoBusManager servoBus1;
extern ServoBusManager servoBus2;
extern ServoBusManager servoBus3;
extern AngleSolver angleSolver;
extern JointMapItem jointMap[ENCODER_TOTAL_NUM];

// 构造：清零零位/比例/方向与PID状态
AngleSolver::AngleSolver() : _initialized(false)
{
    memset(_zeroOffsets, 0, sizeof(_zeroOffsets));
    memset(_gearRatios, 0, sizeof(_gearRatios));
    memset(_directions, 0, sizeof(_directions));
    memset(_pids, 0, sizeof(_pids));
}

// 初始化关节零位、减速比与方向
void AngleSolver::init(int16_t* zeroOffsets, float* gearRatios, int8_t* directions)
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        _zeroOffsets[i] = zeroOffsets[i];
        _gearRatios[i] = gearRatios[i];
        _directions[i] = directions[i];
    }
    _initialized = true;
}

// 初始化双环PID参数（外环角度、内环位置）
void AngleSolver::setPIDParams(float pidParams[][PID_PARAMETER_NUM])
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        PID_Init(&_pids[i][0], PID_POSITION, pidParams[0]);
        PID_Init(&_pids[i][1], PID_POSITION, pidParams[1]);
    }
}

// 计算双环PID并输出舵机脉冲
bool AngleSolver::compute(float* targetDegs, float* magActualDegs,
                          const int32_t* absolutePosition, int16_t* outServoPulses)
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        // 外环：角度环计算
        f_PID_Calculate(&_pids[i][0], targetDegs[i], magActualDegs[i]);

        // 内环：外环输出 + 当前位置作为目标
        float loop2Target = _pids[i][0].Output + (float)absolutePosition[i];
        float loop2Actual = (float)absolutePosition[i];
        f_PID_Calculate(&_pids[i][1], loop2Target, loop2Actual);

        // 合成舵机脉冲输出（内环输出叠加绝对位置）
        outServoPulses[i] = (int16_t)_pids[i][1].Output+absolutePosition[i];
    }
    return true;
}

// 获取指定关节/环路的PID输出
float AngleSolver::getPidOutput(uint8_t jointIndex, uint8_t loopIndex) const
{
    if (jointIndex >= JOINT_COUNT || loopIndex > 1) {
        return 0.0f;
    }
    return _pids[jointIndex][loopIndex].Output;
}

// 编码器计数与调试参数
static const int32_t kEncoderModulo = 16384;
static const int32_t kEncoderHalfTurn = kEncoderModulo / 2;
// 10 degrees in encoder counts (rounded).
static const int32_t kEncoderMarginCounts = (kEncoderModulo * 10 + 180) / 360;
static const uint8_t kClosedLoopJointStart = 4; // 测试关节 4~7（bus1）
static const uint8_t kClosedLoopJointCount = 4;
static const uint8_t kDebugJointIndices[] = {4, 5, 6, 7};
static const uint8_t kDebugJointCount = (uint8_t)(sizeof(kDebugJointIndices) / sizeof(kDebugJointIndices[0]));
static const uint8_t kTestActiveBusIndex = 1;   // 当前仅 bus1
static const uint8_t kTestActiveServoIdMin = 1; // bus1 调试 ID 范围
static const uint8_t kTestActiveServoIdMax = 4; // bus1 调试 ID 范围
static const float kDebugTargetMinDeg = 40.f;
static const float kDebugTargetMaxDeg = 40.0f;
static const uint32_t kDebugTargetHoldMs = 2500;
static const uint32_t kDebugTargetMoveMs = 5000;
static const bool kDebugUseSineTarget = true;
static const uint32_t kCanBusOfflineTimeoutMs = 300; // CAN 离线阈值（去抖）

// 0x7FFF 被上位机协议保留为“断连”标记，避免映射值冲突
#define kDebugUseTestTargets kDebugUseSineTarget

static int16_t clampMappedCountForProtocol(int32_t value)
{
    if (value > 32766) return 32766;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

// 按方向统一编码器朝向（保持计数递增方向一致）
static int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction)
{
    int32_t oriented = (int32_t)rawValue & (kEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kEncoderModulo - oriented) & (kEncoderModulo - 1);
    }
    return oriented;
}

// Wrap encoder delta to +/- half turn to keep startup consistent.
static int32_t wrapEncoderDelta(int32_t delta)
{
    while (delta > kEncoderHalfTurn) delta -= kEncoderModulo;
    while (delta < -kEncoderHalfTurn) delta += kEncoderModulo;
    return delta;
}

// 获取编码器零偏：优先自动校准，其次手动校准
static int32_t getEncoderOffset(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) return 0;
    if (g_jointCalibResult[jointIndex].success) {
        return g_jointCalibResult[jointIndex].offset;
    }
    return g_encoderOffsetManual[jointIndex];
}

// 编码器计数转角度
static float convertEncoderCountToDeg(int32_t encoderCount)
{
    return (float)encoderCount * 360.0f / (float)kEncoderModulo;
}

// 急停判定：总线/传感器/单关节任一异常即停
static bool shouldEmergencyStop(bool canBusOnline,
                                const RemoteSensorData_t& sensorData,
                                const MappedAngleData_t& mappedData,
                                uint8_t jointIndex)
{
    if (!canBusOnline || !sensorData.isValid) {
        return true;
    }

    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return true;
    }

    if (!g_jointCalibResult[jointIndex].success) {
        return true;
    }

    if (sensorData.errorFlags[jointIndex] != 0) {
        return true;
    }

    if (mappedData.validFlags[jointIndex] == 0) {
        return true;
    }

    return false;
}

// 生成调试用正弦目标（含最大/最小值保持）
#define DEBUG_TARGET_VALUE(jointIndex, minDeg, maxDeg, holdMs, moveMs, nowMs) \
    (((jointIndex) == 4) ? computeSineTarget((minDeg), (maxDeg), (holdMs), (moveMs), (nowMs)) : \
     (((jointIndex) == 5 || (jointIndex) == 6 || (jointIndex) == 7) ? 20.0f : \
      computeSineTarget((minDeg), (maxDeg), (holdMs), (moveMs), (nowMs))))

static float computeSineTarget(float minDeg,
                               float maxDeg,
                               uint32_t holdMs,
                               uint32_t moveMs,
                               uint32_t nowMs)
{
    if (maxDeg < minDeg) {
        float tmp = minDeg;
        minDeg = maxDeg;
        maxDeg = tmp;
    }

    const float range = maxDeg - minDeg;
    if (range == 0.0f) return minDeg;

    if (moveMs == 0) {
        const uint32_t cycle = holdMs * 2;
        if (cycle == 0) return minDeg;
        const uint32_t t = nowMs % cycle;
        return (t < holdMs) ? minDeg : maxDeg;
    }

    const uint32_t cycle = holdMs * 2 + moveMs * 2;
    if (cycle == 0) return minDeg;

    uint32_t t = nowMs % cycle;
    if (t < holdMs) return minDeg;
    t -= holdMs;

    if (t < moveMs) {
        const float s = (float)t / (float)moveMs;
        const float w = 0.5f - 0.5f * cosf(3.1415926f * s);
        return minDeg + range * w;
    }

    t -= moveMs;
    if (t < holdMs) return maxDeg;
    t -= holdMs;

    const float s = (float)t / (float)moveMs;
    const float w = 0.5f - 0.5f * cosf(3.1415926f * s);
    return maxDeg - range * w;
}

// 根据索引获取舵机总线对象
static ServoBusManager* getBusByIndex(uint8_t busIndex)
{
    switch (busIndex)
    {
    case 0: return &servoBus0;
    case 1: return &servoBus1;
    case 2: return &servoBus2;
    case 3: return &servoBus3;
    default: return NULL;
    }
}

void taskSolver(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    const uint8_t testBusIds[] = {1, 2, 3, 4};
    const uint8_t testBusCount = (uint8_t)(sizeof(testBusIds) / sizeof(testBusIds[0]));

    float localTargets[ENCODER_TOTAL_NUM] = {0.0f};
    float magAngles[ENCODER_TOTAL_NUM] = {0.0f};
    float servoAngles[ENCODER_TOTAL_NUM] = {0.0f};
    int32_t absolutePosition[ENCODER_TOTAL_NUM] = {0};
    int16_t outPulses[ENCODER_TOTAL_NUM] = {0};

    RemoteSensorData_t sensorData;
    MappedAngleData_t mappedData;

#if SOLVER_DIAG_LOG_ENABLE
    uint32_t lastDiagLogMs = 0;
#endif

    while (1)
    {
        // 同步读取舵机位置（调试总线）
        bool busWritePending[NUM_BUSES] = {false};
        ServoBusManager* testBus = getBusByIndex(kTestActiveBusIndex);
        int testBusSuccessCount = testBus ? testBus->syncReadPositions(testBusIds, testBusCount) : 0;
        (void)testBusSuccessCount;

#if SOLVER_DIAG_LOG_ENABLE
        uint32_t nowMs = millis();
        if (nowMs - lastDiagLogMs >= 500) {
            lastDiagLogMs = nowMs;
            const uint8_t diagId = kTestActiveServoIdMin;
            const int diagOnline = (testBus && testBus->isOnline(diagId)) ? 1 : 0;
            const long diagAbs = testBus ? (long)testBus->getAbsolutePosition(diagId) : 0;
            Serial.printf("[Solver] bus%d read=%d online=%d abs=%ld\r\n",
                          kTestActiveBusIndex,
                          testBusSuccessCount,
                          diagOnline,
                          diagAbs);
        }
#endif

        // 采集舵机角度与在线状态
        ServoAngleData_t servoData;
        ServoTelemetryData_t telemetryData;
        memset(&servoData, 0, sizeof(servoData));
        servoData.timestamp = millis();
        memset(&telemetryData, 0, sizeof(telemetryData));
        telemetryData.timestamp = servoData.timestamp;

        // 采集闭环舵机位置（绝对位置）
        for (int i = 0; i < kClosedLoopJointCount; i++)
        {
            uint8_t jointIndex = kClosedLoopJointStart + i;
            uint8_t bus = jointMap[jointIndex].busIndex;
            uint8_t id = jointMap[jointIndex].servoID;
            ServoBusManager* pBus = getBusByIndex(bus);

            if (pBus && pBus->isOnline(id))
            {
                int32_t absPos = pBus->getAbsolutePosition(id);
                servoAngles[jointIndex] = (float)absPos * 360.0f / 4096.0f;
                absolutePosition[jointIndex] = absPos;
                servoData.servoAngles[jointIndex] = absPos;
                servoData.onlineStatus[jointIndex] = 1;
            }
            else
            {
                servoAngles[jointIndex] = 0.0f;
                absolutePosition[jointIndex] = 0;
                servoData.servoAngles[jointIndex] = 0;
                servoData.onlineStatus[jointIndex] = 0;
            }
        }
        xQueueOverwrite(sharedData->servoAngleQueue, &servoData);

        // 采集舵机速度/负载/电压/温度
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            uint8_t bus = jointMap[i].busIndex;
            uint8_t id = jointMap[i].servoID;
            ServoBusManager* pBus = getBusByIndex(bus);
            if (pBus && pBus->isOnline(id)) {
                const ServoFeedback& fb = pBus->getFeedback(id);
                telemetryData.speed[i] = fb.speed;
                telemetryData.load[i] = fb.load;
                telemetryData.voltage[i] = fb.voltage;
                telemetryData.temperature[i] = fb.temperature;
                telemetryData.onlineStatus[i] = 1;
            } else {
                telemetryData.speed[i] = 0;
                telemetryData.load[i] = 0;
                telemetryData.voltage[i] = 0;
                telemetryData.temperature[i] = 0;
                telemetryData.onlineStatus[i] = 0;
            }
        }
        if (sharedData->servoTelemetryQueue) {
            xQueueOverwrite(sharedData->servoTelemetryQueue, &telemetryData);
        }

        // 判断 CAN 总线在线（基于时间戳）
        bool canBusOnline = false;
        if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE && sensorData.isValid) {
            const uint32_t nowMs = millis();
            canBusOnline = (nowMs - sensorData.timestamp) <= kCanBusOfflineTimeoutMs;
        }

        memset(&mappedData, 0, sizeof(mappedData));
        mappedData.timestamp = millis();
        mappedData.isValid = canBusOnline;

        if (canBusOnline)
        {
            // CAN 在线：按编码器有效标志映射角度
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
            {
                if (sensorData.errorFlags[i] != 0) {
                    mappedData.validFlags[i] = 0;
                    magAngles[i] = 0.0f;
                    continue;
                }
                const int32_t orientedRaw = orientEncoderRaw(sensorData.encoderValues[i], g_encoderDirection[i]);
                int32_t mappedCount = orientedRaw;
                if (g_jointCalibResult[i].success) {
                    const int32_t offset = getEncoderOffset((uint8_t)i);
                    const int32_t angleScope = g_jointCalibConfig[i].angleScope;
                    const int32_t bottomReserved = g_jointCalibConfig[i].bottomReserved;
                    int32_t delta = orientedRaw - offset + bottomReserved;
                    delta %= kEncoderModulo;
                    if (delta < 0) delta += kEncoderModulo;
                    if (delta > angleScope + kEncoderMarginCounts) {
                        delta -= kEncoderModulo;
                    }
                    mappedCount = delta - bottomReserved;
                }

                mappedData.angleValues[i] = clampMappedCountForProtocol(mappedCount);
                mappedData.validFlags[i] = 1;
                magAngles[i] = convertEncoderCountToDeg(mappedCount);
            }
        }
        else
        {
            // CAN offline: mark invalid.
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
                mappedData.validFlags[i] = 0;
            }
        }

        xQueueOverwrite(sharedData->mappedAngleQueue, &mappedData);

        // 读取目标角度（上位机/控制器）
        if (xSemaphoreTake(sharedData->targetAnglesMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            memcpy(localTargets, sharedData->targetAngles, sizeof(localTargets));
            xSemaphoreGive(sharedData->targetAnglesMutex);
        }
        const bool controlEnabled = (sharedData->control_enabled != 0);

        // 调试：生成正弦目标
        if (controlEnabled && kDebugUseTestTargets) {
            for (uint8_t di = 0; di < kDebugJointCount; di++) {
                uint8_t jointIndex = kDebugJointIndices[di];
                if (jointIndex >= ENCODER_TOTAL_NUM) {
                    continue;
                }
                localTargets[jointIndex] = DEBUG_TARGET_VALUE(jointIndex,
                    kDebugTargetMinDeg,
                    kDebugTargetMaxDeg,
                    kDebugTargetHoldMs,
                    kDebugTargetMoveMs,
                    millis());
            }
        }

        // 双环 PID 计算
        angleSolver.compute(localTargets, magAngles, absolutePosition, outPulses);

        // 调试数据上报
        if (sharedData->jointDebugQueue)
        {
            for (uint8_t di = 0; di < kDebugJointCount; di++)
            {
                uint8_t jointIndex = kDebugJointIndices[di];
                if (jointIndex < kClosedLoopJointStart ||
                    jointIndex >= (kClosedLoopJointStart + kClosedLoopJointCount))
                {
                    continue;
                }

                JointDebugData_t debugData;
                memset(&debugData, 0, sizeof(debugData));
                debugData.jointIndex = jointIndex;
                debugData.timestamp = millis();
                debugData.valid = (mappedData.validFlags[jointIndex] != 0) ? 1 : 0;
                debugData.targetDeg = localTargets[jointIndex];
                debugData.magActualDeg = magAngles[jointIndex];
                debugData.loop1Output = angleSolver.getPidOutput(jointIndex, 0);
                debugData.loop2Actual = (float)absolutePosition[jointIndex];
                debugData.loop2Output = angleSolver.getPidOutput(jointIndex, 1);

                if (xQueueSend(sharedData->jointDebugQueue, &debugData, 0) != pdTRUE) {
                    JointDebugData_t drop;
                    xQueueReceive(sharedData->jointDebugQueue, &drop, 0);
                    xQueueSend(sharedData->jointDebugQueue, &debugData, 0);
                }
            }
        }

        // 发送舵机目标（含急停）
        for (int i = 0; i < kClosedLoopJointCount; i++)
        {
            uint8_t jointIndex = kClosedLoopJointStart + i;
            uint8_t bus = jointMap[jointIndex].busIndex;
            uint8_t id = jointMap[jointIndex].servoID;
            ServoBusManager* pBus = getBusByIndex(bus);

            if (!controlEnabled || shouldEmergencyStop(canBusOnline, sensorData, mappedData, jointIndex)) {
                if (pBus && pBus->isOnline(id)) {
                    int16_t holdPos = constrain(absolutePosition[jointIndex], -30719, 30719);
                    pBus->setTarget(id, holdPos, 1000, 50);
                    if (bus < NUM_BUSES) {
                        busWritePending[bus] = true;
                    }
                }
                continue;
            }

            // 调试：生成正弦目标
            if (controlEnabled && kDebugUseTestTargets) {
                // 调试：仅对指定 bus/id 下发
                if (pBus &&
                    bus == kTestActiveBusIndex &&
                    id >= kTestActiveServoIdMin &&
                    id <= kTestActiveServoIdMax)
                {
                    int16_t targetPos = constrain(outPulses[jointIndex], -30719, 30719);
                    pBus->setTarget(id, targetPos, 1000, 50);
                    if (bus < NUM_BUSES) {
                        busWritePending[bus] = true;
                    }
                }
            } else if (pBus) {
                int16_t targetPos = constrain(outPulses[jointIndex], -30719, 30719);
                pBus->setTarget(id, targetPos, 1000, 50);
                if (bus < NUM_BUSES) {
                    busWritePending[bus] = true;
                }
            }
        }

        // 触发总线写入（如启用同步写）
        for (uint8_t bus = 0; bus < NUM_BUSES; bus++)
        {
            if (busWritePending[bus]) {
                ServoBusManager* pBus = getBusByIndex(bus);
                if (pBus) {
                     pBus->syncWriteAll();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

