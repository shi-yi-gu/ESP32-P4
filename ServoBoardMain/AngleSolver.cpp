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
                          float* servoActualDegs, int16_t* outServoPulses)
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        targetDegs[i]=45;
        f_PID_Calculate(&_pids[i][0], targetDegs[i], magActualDegs[i]);

        float loop2Target = _pids[i][0].Output + servoActualDegs[i];
        float loop2Actual = servoActualDegs[i];
        f_PID_Calculate(&_pids[i][1], loop2Target, loop2Actual);

        outServoPulses[i] = (int16_t)_pids[i][1].Output;
    }
    return true;
}

static const int32_t kEncoderModulo = 16384;
static const int32_t kEncoderHalfTurn = kEncoderModulo / 2;
static const uint8_t kClosedLoopJointCount = 4; // test only joints 0~3
static const uint8_t kTestActiveBusIndex = 0;   // currently only bus0
static const uint8_t kTestActiveServoIdMin = 1; // debug range on bus0
static const uint8_t kTestActiveServoIdMax = 4; // debug range on bus0
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

    const uint8_t bus0Ids[] = {1, 2, 3, 4};
    const uint8_t bus0Count = (uint8_t)(sizeof(bus0Ids) / sizeof(bus0Ids[0]));

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
        int bus0SuccessCount = servoBus0.syncReadPositions(bus0Ids, bus0Count);
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

        for (int i = 0; i < kClosedLoopJointCount; i++)
        {
            uint8_t bus = jointMap[i].busIndex;
            uint8_t id = jointMap[i].servoID;
            ServoBusManager* pBus = getBusByIndex(bus);

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
            // Project rule: per-encoder faults are handled by lower controller.
            // Here we only gate on CAN bus availability, not sensorData.errorFlags[i].
            for (int i = 0; i < kClosedLoopJointCount; i++)
            {
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
            // Bus offline: mark all test joints invalid and reset unwrapping state.
            for (int i = 0; i < kClosedLoopJointCount; i++) {
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

        angleSolver.compute(localTargets, magAngles, servoAngles, outPulses);

        for (int i = 0; i < kClosedLoopJointCount; i++)
        {
            uint8_t bus = jointMap[i].busIndex;
            uint8_t id = jointMap[i].servoID;
            ServoBusManager* pBus = getBusByIndex(bus);

            // Test stage: send to bus0 IDs 1~4 for debug.
            if (pBus &&
                bus == kTestActiveBusIndex &&
                id >= kTestActiveServoIdMin &&
                id <= kTestActiveServoIdMax)
            {
                int16_t targetPos = constrain(outPulses[i], -30719, 30719);
                pBus->setTarget(id, targetPos, 1000, 50);
            }
        }

        servoBus0.syncWriteAll();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
