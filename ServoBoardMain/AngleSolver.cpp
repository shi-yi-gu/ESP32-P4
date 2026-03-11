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

AngleSolver::AngleSolver() : _initialized(false)
{
    memset(_zeroOffsets, 0, sizeof(_zeroOffsets));
    memset(_gearRatios, 0, sizeof(_gearRatios));
    memset(_directions, 0, sizeof(_directions));
    memset(_pids, 0, sizeof(_pids));
}

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

void AngleSolver::setPIDParams(float pidParams[][PID_PARAMETER_NUM])
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        PID_Init(&_pids[i][0], PID_POSITION, pidParams[0]);
        PID_Init(&_pids[i][1], PID_POSITION, pidParams[1]);
    }
}

bool AngleSolver::compute(float* targetDegs, float* magActualDegs,
                          const int32_t* absolutePosition, int16_t* outServoPulses)
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        f_PID_Calculate(&_pids[i][0], targetDegs[i], magActualDegs[i]);

        float loop2Target = _pids[i][0].Output + (float)absolutePosition[i];
        float loop2Actual = (float)absolutePosition[i];
        f_PID_Calculate(&_pids[i][1], loop2Target, loop2Actual);

        outServoPulses[i] = (int16_t)_pids[i][1].Output+absolutePosition[i];
    }
    return true;
}

float AngleSolver::getPidOutput(uint8_t jointIndex, uint8_t loopIndex) const
{
    if (jointIndex >= JOINT_COUNT || loopIndex > 1) {
        return 0.0f;
    }
    return _pids[jointIndex][loopIndex].Output;
}

static const int32_t kEncoderModulo = 16384;
static const int32_t kEncoderHalfTurn = kEncoderModulo / 2;
static const uint8_t kClosedLoopJointStart = 4; // test joints 4~7 (bus1)
static const uint8_t kClosedLoopJointCount = 4;
static const uint8_t kDebugJointIndex = 5;       // stream debug info for joint-5
static const uint8_t kTestActiveBusIndex = 1;   // currently only bus1
static const uint8_t kTestActiveServoIdMin = 1; // debug range on bus1
static const uint8_t kTestActiveServoIdMax = 4; // debug range on bus1
static const float kDebugTargetMinDeg = 3.f;
static const float kDebugTargetMaxDeg = 30.0f;
static const uint32_t kDebugTargetHoldMs = 2500;
static const uint32_t kDebugTargetMoveMs = 5000;
static const bool kDebugUseSineTarget = true;
static const uint32_t kCanBusOfflineTimeoutMs = 300; // bus-level offline threshold (debounced for jitter)

// 0x7FFF is reserved by upper protocol as "DISCONNECT" sentinel.
// Clamp valid mapped counts away from this value to avoid false disconnect display.
static int16_t clampMappedCountForProtocol(int32_t value)
{
    if (value > 32766) return 32766;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

static int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction)
{
    int32_t oriented = (int32_t)rawValue & (kEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kEncoderModulo - oriented) & (kEncoderModulo - 1);
    }
    return oriented;
}

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

static int32_t getEncoderOffset(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) return 0;
    if (g_jointCalibResult[jointIndex].success) {
        return g_jointCalibResult[jointIndex].offset;
    }
    return g_encoderOffsetManual[jointIndex];
}

static float convertEncoderCountToDeg(int32_t encoderCount)
{
    return (float)encoderCount * 360.0f / (float)kEncoderModulo;
}

static bool shouldEmergencyStop(bool canBusOnline,
                                const RemoteSensorData_t& sensorData,
                                const MappedAngleData_t& mappedData,
                                uint8_t jointIndex)
{
    if (kDebugUseSineTarget) {
        return false;
    }

    if (!canBusOnline || !sensorData.isValid) {
        return true;
    }

    if (jointIndex >= ENCODER_TOTAL_NUM) {
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
    int32_t prevOrientedRaw[ENCODER_TOTAL_NUM] = {0};
    int32_t continuousRaw[ENCODER_TOTAL_NUM] = {0};
    bool unwrapInitialized[ENCODER_TOTAL_NUM] = {false};

#if SOLVER_DIAG_LOG_ENABLE
    uint32_t lastDiagLogMs = 0;
#endif

    while (1)
    {
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

        ServoAngleData_t servoData;
        memset(&servoData, 0, sizeof(servoData));
        servoData.timestamp = millis();

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
            // Gate on CAN bus availability and per-encoder validity flags.
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
            {
                if (sensorData.errorFlags[i] != 0) {
                    mappedData.validFlags[i] = 0;
                    unwrapInitialized[i] = false;
                    magAngles[i] = 0.0f;
                    continue;
                }
                const int32_t orientedRaw = orientEncoderRaw(sensorData.encoderValues[i], g_encoderDirection[i]);
                const int32_t continuous = unwrapEncoderToContinuous((uint8_t)i,
                                                                     orientedRaw,
                                                                     prevOrientedRaw,
                                                                     continuousRaw,
                                                                     unwrapInitialized);
                const int32_t mappedCount = continuous - getEncoderOffset((uint8_t)i);

                mappedData.angleValues[i] = clampMappedCountForProtocol(mappedCount);
                mappedData.validFlags[i] = 1;
                magAngles[i] = convertEncoderCountToDeg(mappedCount);
            }
        }
        else
        {
            // Bus offline: mark all joints invalid and reset unwrapping state.
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
                mappedData.validFlags[i] = 0;
                unwrapInitialized[i] = false;
            }
        }

        xQueueOverwrite(sharedData->mappedAngleQueue, &mappedData);

        if (xSemaphoreTake(sharedData->targetAnglesMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            memcpy(localTargets, sharedData->targetAngles, sizeof(localTargets));
            xSemaphoreGive(sharedData->targetAnglesMutex);
        }

        if (kDebugUseSineTarget && kDebugJointIndex < ENCODER_TOTAL_NUM) {
            localTargets[kDebugJointIndex] = computeSineTarget(
                kDebugTargetMinDeg,
                kDebugTargetMaxDeg,
                kDebugTargetHoldMs,
                kDebugTargetMoveMs,
                millis());
        }

        angleSolver.compute(localTargets, magAngles, absolutePosition, outPulses);

        if (sharedData->jointDebugQueue &&
            kDebugJointIndex >= kClosedLoopJointStart &&
            kDebugJointIndex < (kClosedLoopJointStart + kClosedLoopJointCount))
        {
            JointDebugData_t debugData;
            memset(&debugData, 0, sizeof(debugData));
            debugData.timestamp = millis();
            debugData.valid = (mappedData.validFlags[kDebugJointIndex] != 0) ? 1 : 0;
            debugData.targetDeg = localTargets[kDebugJointIndex];
            debugData.magActualDeg = magAngles[kDebugJointIndex];
            debugData.loop1Output = angleSolver.getPidOutput(kDebugJointIndex, 0);
            debugData.loop2Actual = (float)absolutePosition[kDebugJointIndex];
            debugData.loop2Output = angleSolver.getPidOutput(kDebugJointIndex, 1);
            xQueueOverwrite(sharedData->jointDebugQueue, &debugData);
        }

        for (int i = 0; i < kClosedLoopJointCount; i++)
        {
            uint8_t jointIndex = kClosedLoopJointStart + i;
            uint8_t bus = jointMap[jointIndex].busIndex;
            uint8_t id = jointMap[jointIndex].servoID;
            ServoBusManager* pBus = getBusByIndex(bus);

            if (shouldEmergencyStop(canBusOnline, sensorData, mappedData, jointIndex)) {
                if (pBus && pBus->isOnline(id)) {
                    int16_t holdPos = constrain(absolutePosition[jointIndex], -30719, 30719);
                    pBus->setTarget(id, holdPos, 1000, 50);
                    if (bus < NUM_BUSES) {
                        busWritePending[bus] = true;
                    }
                }
                continue;
            }

            if (kDebugUseSineTarget) {
                // Test stage: send to active bus IDs for debug.
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

