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
extern MotorMapItem joint16SecondaryMotor;
extern MotorMapItem motorMap[SERVO_TOTAL_NUM];

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

        const float loop2Target = _pids[i][0].Output + (float)absolutePosition[i];
        const float loop2Actual = (float)absolutePosition[i];
        f_PID_Calculate(&_pids[i][1], loop2Target, loop2Actual);

        outServoPulses[i] = (int16_t)_pids[i][1].Output + absolutePosition[i];
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
static const int32_t kEncoderMarginCounts = (kEncoderModulo * 10 + 180) / 360;

static const uint8_t kClosedLoopJointIndices[] = {4, 5, 6, 7, 16};
static const uint8_t kClosedLoopJointCount =
    (uint8_t)(sizeof(kClosedLoopJointIndices) / sizeof(kClosedLoopJointIndices[0]));

static const uint8_t kDebugJointIndices[] = {4, 5, 6, 7};
static const uint8_t kDebugJointCount =
    (uint8_t)(sizeof(kDebugJointIndices) / sizeof(kDebugJointIndices[0]));

// joint16 双舵机拮抗参数：
// - 副反馈投影到主坐标: S_proj = -S + secondaryOffset
// - 副舵机目标:          T_sub  = -T_main + secondaryOffset + tensionBias
// - 一致性判故障: abs(P - S_proj) > threshold 持续 faultCycles 周期
static const uint8_t kJoint16Index = 16;
static const int32_t kJoint16SecondaryOffset = 0;
static const int32_t kJoint16TensionBias = 0;
static const int32_t kJoint16DiffThresholdCounts = 180;
static const uint8_t kJoint16DiffFaultCycles = 3;

// Firmware-only debug mode:
// - ignores host START gate for closed-loop joints,
// - enforces JOINT mode,
// - drives J04~J07 using built-in target generator.
static const bool kLocalDebugJointControl = true;
static const float kDebugTargetMinDeg = 10.0f;
static const float kDebugTargetMaxDeg = 40.0f;
static const uint32_t kDebugTargetHoldMs = 2500;
static const uint32_t kDebugTargetMoveMs = 5000;
static const bool kDebugUseSineTarget = true;
static const float kDebugFixedTargetDeg = 20.0f;

static const uint32_t kCanBusOfflineTimeoutMs = 300;
static const uint16_t kEncoderDisconnectRaw = 0xFFFF;

static int16_t clampMappedCountForProtocol(int32_t value)
{
    if (value > 32766) return 32766;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

static int16_t clampServoPos(int32_t value)
{
    if (value > 30719) return 30719;
    if (value < -30719) return -30719;
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

static int32_t wrapEncoderDelta(int32_t delta)
{
    while (delta > kEncoderHalfTurn) delta -= kEncoderModulo;
    while (delta < -kEncoderHalfTurn) delta += kEncoderModulo;
    return delta;
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
        const float tmp = minDeg;
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

static int findMotorChannel(uint8_t bus, uint8_t id)
{
    for (int i = 0; i < SERVO_TOTAL_NUM; i++)
    {
        if (motorMap[i].busIndex == bus && motorMap[i].servoID == id) {
            return i;
        }
    }
    return -1;
}

static bool addReadTarget(uint8_t busIds[NUM_BUSES][MAX_SERVOS_PER_BUS],
                          uint8_t busCounts[NUM_BUSES],
                          uint8_t bus,
                          uint8_t id)
{
    if (bus >= NUM_BUSES) {
        return false;
    }
    for (uint8_t i = 0; i < busCounts[bus]; i++)
    {
        if (busIds[bus][i] == id) {
            return true;
        }
    }
    if (busCounts[bus] >= MAX_SERVOS_PER_BUS) {
        return false;
    }
    busIds[bus][busCounts[bus]++] = id;
    return true;
}

static bool isDebugJoint(uint8_t jointIndex)
{
    for (uint8_t i = 0; i < kDebugJointCount; i++)
    {
        if (kDebugJointIndices[i] == jointIndex) {
            return true;
        }
    }
    return false;
}

void taskSolver(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    float localTargets[ENCODER_TOTAL_NUM] = {0.0f};
    float magAngles[ENCODER_TOTAL_NUM] = {0.0f};
    int32_t absolutePosition[ENCODER_TOTAL_NUM] = {0};
    int16_t outPulses[ENCODER_TOTAL_NUM] = {0};

    RemoteSensorData_t sensorData;
    MappedAngleData_t mappedData;
    uint8_t joint16DiffFaultCounter = 0;

#if SOLVER_DIAG_LOG_ENABLE
    uint32_t lastDiagLogMs = 0;
#endif

    while (1)
    {
        bool busWritePending[NUM_BUSES] = {false};
        uint8_t readIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
        uint8_t readCounts[NUM_BUSES] = {0};
        int16_t jointCmdPos[ENCODER_TOTAL_NUM] = {0};
        uint8_t jointCmdValid[ENCODER_TOTAL_NUM] = {0};

        // 每周期读取全部舵机通道，保证 22 通道上报完整可见。
        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
        {
            addReadTarget(readIds, readCounts, motorMap[ch].busIndex, motorMap[ch].servoID);
        }

        for (uint8_t bus = 0; bus < NUM_BUSES; bus++)
        {
            if (readCounts[bus] == 0) {
                continue;
            }
            ServoBusManager* pBus = getBusByIndex(bus);
            if (pBus) {
                pBus->syncReadPositions(readIds[bus], readCounts[bus]);
            }
        }

        ServoAngleData_t servoData;
        ServoTelemetryData_t telemetryData;
        memset(&servoData, 0, sizeof(servoData));
        memset(&telemetryData, 0, sizeof(telemetryData));
        servoData.timestamp = millis();
        telemetryData.timestamp = servoData.timestamp;

        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
        {
            const uint8_t bus = motorMap[ch].busIndex;
            const uint8_t id = motorMap[ch].servoID;
            ServoBusManager* pBus = getBusByIndex(bus);
            if (pBus && pBus->isOnline(id))
            {
                const int32_t absPos = pBus->getAbsolutePosition(id);
                const ServoFeedback& fb = pBus->getFeedback(id);
                servoData.servoAngles[ch] = absPos;
                servoData.onlineStatus[ch] = 1;
                telemetryData.speed[ch] = fb.speed;
                telemetryData.load[ch] = fb.load;
                telemetryData.voltage[ch] = fb.voltage;
                telemetryData.temperature[ch] = fb.temperature;
                telemetryData.onlineStatus[ch] = 1;
            }
        }

        if (sharedData->servoAngleQueue) {
            xQueueOverwrite(sharedData->servoAngleQueue, &servoData);
        }
        if (sharedData->servoTelemetryQueue) {
            xQueueOverwrite(sharedData->servoTelemetryQueue, &telemetryData);
        }

        // 从舵机反馈构建“关节侧”绝对位置（闭环用）。
        memset(absolutePosition, 0, sizeof(absolutePosition));
        for (uint8_t i = 0; i < kClosedLoopJointCount; i++)
        {
            const uint8_t jointIndex = kClosedLoopJointIndices[i];
            const uint8_t bus = jointMap[jointIndex].busIndex;
            const uint8_t id = jointMap[jointIndex].servoID;
            const int ch = findMotorChannel(bus, id);
            if (ch >= 0 && servoData.onlineStatus[ch]) {
                absolutePosition[jointIndex] = servoData.servoAngles[ch];
            }
        }

        bool joint16DualFault = false;
        {
            const uint8_t pBus = jointMap[kJoint16Index].busIndex;
            const uint8_t pId = jointMap[kJoint16Index].servoID;
            const uint8_t sBus = joint16SecondaryMotor.busIndex;
            const uint8_t sId = joint16SecondaryMotor.servoID;
            const int pCh = findMotorChannel(pBus, pId);
            const int sCh = findMotorChannel(sBus, sId);
            const bool pOnline = (pCh >= 0) && (servoData.onlineStatus[pCh] != 0);
            const bool sOnline = (sCh >= 0) && (servoData.onlineStatus[sCh] != 0);

            if (pOnline && sOnline)
            {
                const int32_t p = servoData.servoAngles[pCh];
                const int32_t s = servoData.servoAngles[sCh];
                const int32_t sProj = -s + kJoint16SecondaryOffset;
                const int32_t fused = (p + sProj) / 2;
                const int32_t diff = (p >= sProj) ? (p - sProj) : (sProj - p);

                // 双反馈融合后的关节16位置用于闭环。
                absolutePosition[kJoint16Index] = fused;

                // 连续超差判故障，抑制单周期抖动误触发。
                if (diff > kJoint16DiffThresholdCounts) {
                    if (joint16DiffFaultCounter < 255) {
                        joint16DiffFaultCounter++;
                    }
                } else {
                    joint16DiffFaultCounter = 0;
                }
                joint16DualFault = (joint16DiffFaultCounter >= kJoint16DiffFaultCycles);
            }
            else
            {
                joint16DiffFaultCounter = 0;
                joint16DualFault = true;
            }
        }
        sharedData->joint16_dual_feedback_fault = joint16DualFault ? 1 : 0;

#if SOLVER_DIAG_LOG_ENABLE
        const uint32_t nowMs = millis();
        if (nowMs - lastDiagLogMs >= 500) {
            lastDiagLogMs = nowMs;
            const int pCh = findMotorChannel(jointMap[kJoint16Index].busIndex, jointMap[kJoint16Index].servoID);
            const int sCh = findMotorChannel(joint16SecondaryMotor.busIndex, joint16SecondaryMotor.servoID);
            const long pAbs = (pCh >= 0) ? (long)servoData.servoAngles[pCh] : 0;
            const long sAbs = (sCh >= 0) ? (long)servoData.servoAngles[sCh] : 0;
            Serial.printf("[Solver] j16 p=%ld s=%ld fault=%d\r\n", pAbs, sAbs, joint16DualFault ? 1 : 0);
        }
#endif

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
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
            {
                if (sensorData.encoderValues[i] == kEncoderDisconnectRaw) {
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

        if (sharedData->mappedAngleQueue) {
            xQueueOverwrite(sharedData->mappedAngleQueue, &mappedData);
        }

        if (xSemaphoreTake(sharedData->targetAnglesMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            memcpy(localTargets, sharedData->targetAngles, sizeof(localTargets));
            xSemaphoreGive(sharedData->targetAnglesMutex);
        }
        const bool hostControlEnabled = (sharedData->control_enabled != 0);
        const bool controlEnabled = hostControlEnabled || kLocalDebugJointControl;

        if (controlEnabled && kLocalDebugJointControl) {
            for (uint8_t di = 0; di < kDebugJointCount; di++) {
                const uint8_t jointIndex = kDebugJointIndices[di];
                localTargets[jointIndex] = kDebugUseSineTarget
                    ? computeSineTarget(
                        kDebugTargetMinDeg,
                        kDebugTargetMaxDeg,
                        kDebugTargetHoldMs,
                        kDebugTargetMoveMs,
                        millis())
                    : kDebugFixedTargetDeg;
            }
        }

        angleSolver.compute(localTargets, magAngles, absolutePosition, outPulses);

        if (kLocalDebugJointControl && sharedData->control_mode != CONTROL_MODE_JOINT) {
            sharedData->control_mode = CONTROL_MODE_JOINT;
        }

        if (sharedData->control_mode == CONTROL_MODE_DIRECT_MOTOR)
        {
            if (!controlEnabled || joint16DualFault) {
                if (joint16DualFault) {
                    // 触发双反馈故障后，需显式 start/reset 才恢复。
                    sharedData->control_enabled = 0;
                }
            } else {
                for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
                {
                    const uint8_t bus = motorMap[ch].busIndex;
                    const uint8_t id = motorMap[ch].servoID;
                    ServoBusManager* pBus = getBusByIndex(bus);
                    if (!pBus) {
                        continue;
                    }
                    const int16_t targetPos = clampServoPos(sharedData->motorTargetRaw[ch]);
                    pBus->setTarget(id, targetPos, 1000, 50);
                    busWritePending[bus] = true;
                }
            }
        }
        else
        {
            for (uint8_t i = 0; i < kClosedLoopJointCount; i++)
            {
                const uint8_t jointIndex = kClosedLoopJointIndices[i];
                const uint8_t bus = jointMap[jointIndex].busIndex;
                const uint8_t id = jointMap[jointIndex].servoID;
                ServoBusManager* pBus = getBusByIndex(bus);

                bool emergency =
                    (!controlEnabled) ||
                    joint16DualFault ||
                    shouldEmergencyStop(canBusOnline, sensorData, mappedData, jointIndex);

                // In pure local debug mode, only J04~J07 are actively driven.
                if (kLocalDebugJointControl && !isDebugJoint(jointIndex)) {
                    emergency = true;
                }

                if (!pBus) {
                    continue;
                }

                if (emergency)
                {
                    if (pBus->isOnline(id)) {
                        const int16_t holdPos = clampServoPos(absolutePosition[jointIndex]);
                        pBus->setTarget(id, holdPos, 1000, 50);
                        busWritePending[bus] = true;
                        jointCmdPos[jointIndex] = holdPos;
                        jointCmdValid[jointIndex] = 1;
                    }

                    if (jointIndex == kJoint16Index) {
                        ServoBusManager* pSecBus = getBusByIndex(joint16SecondaryMotor.busIndex);
                        if (pSecBus && pSecBus->isOnline(joint16SecondaryMotor.servoID)) {
                            const int secCh = findMotorChannel(joint16SecondaryMotor.busIndex, joint16SecondaryMotor.servoID);
                            const int16_t holdSec = (secCh >= 0) ? clampServoPos(servoData.servoAngles[secCh]) : 0;
                            pSecBus->setTarget(joint16SecondaryMotor.servoID, holdSec, 1000, 50);
                            busWritePending[joint16SecondaryMotor.busIndex] = true;
                        }
                    }
                    continue;
                }

                const int16_t targetPos = clampServoPos(outPulses[jointIndex]);
                pBus->setTarget(id, targetPos, 1000, 50);
                busWritePending[bus] = true;
                jointCmdPos[jointIndex] = targetPos;
                jointCmdValid[jointIndex] = 1;

                if (jointIndex == kJoint16Index) {
                    ServoBusManager* pSecBus = getBusByIndex(joint16SecondaryMotor.busIndex);
                    if (pSecBus) {
                        // joint16 拮抗下发：主收绳时副放绳（反向+偏置）。
                        const int32_t secTarget = -((int32_t)targetPos) + kJoint16SecondaryOffset + kJoint16TensionBias;
                        pSecBus->setTarget(joint16SecondaryMotor.servoID, clampServoPos(secTarget), 1000, 50);
                        busWritePending[joint16SecondaryMotor.busIndex] = true;
                    }
                }
            }
        }

        if (sharedData->jointDebugQueue)
        {
            for (uint8_t di = 0; di < kDebugJointCount; di++)
            {
                const uint8_t jointIndex = kDebugJointIndices[di];
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
                debugData.cmdTargetPos = jointCmdPos[jointIndex];
                debugData.cmdValid = jointCmdValid[jointIndex];

                if (xQueueSend(sharedData->jointDebugQueue, &debugData, 0) != pdTRUE) {
                    JointDebugData_t drop;
                    xQueueReceive(sharedData->jointDebugQueue, &drop, 0);
                    xQueueSend(sharedData->jointDebugQueue, &debugData, 0);
                }
            }
        }

        for (uint8_t bus = 0; bus < NUM_BUSES; bus++)
        {
            if (!busWritePending[bus]) {
                continue;
            }
            ServoBusManager* pBus = getBusByIndex(bus);
            if (pBus) {
                pBus->syncWriteAll();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
