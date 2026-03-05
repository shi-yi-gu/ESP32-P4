#include "CalibrationTask.h"

#include <string.h>

#include "AngleSolver.h"
#include "ServoBusManager.h"

extern ServoBusManager servoBus0;
extern ServoBusManager servoBus1;
extern ServoBusManager servoBus2;
extern ServoBusManager servoBus3;
extern JointMapItem jointMap[ENCODER_TOTAL_NUM];
extern volatile uint8_t g_calibrationUIStatus;

JointCalibrationConfig g_jointCalibConfig[ENCODER_TOTAL_NUM];
JointCalibrationResult g_jointCalibResult[ENCODER_TOTAL_NUM];

static const int32_t kEncoderModulo = 16384;
static const int32_t kEncoderHalfTurn = kEncoderModulo / 2;

static ServoBusManager* getBusByIndex(uint8_t busIndex) {
    switch (busIndex) {
        case 0: return &servoBus0;
        case 1: return &servoBus1;
        case 2: return &servoBus2;
        case 3: return &servoBus3;
        default: return NULL;
    }
}

static int16_t clampToServoPos(int32_t value) {
    if (value > 30719) return 30719;
    if (value < -30719) return -30719;
    return (int16_t)value;
}

static int32_t wrapEncoderDelta(int32_t target, int32_t actual) {
    int32_t delta = target - actual;
    while (delta > kEncoderHalfTurn) delta -= kEncoderModulo;
    while (delta < -kEncoderHalfTurn) delta += kEncoderModulo;
    return delta;
}

static bool readLatestEncoderRaw(TaskSharedData_t* sharedData, uint8_t jointIndex, int32_t* outEncoder) {
    if (!sharedData || !outEncoder || jointIndex >= ENCODER_TOTAL_NUM) return false;

    RemoteSensorData_t sensorData;
    if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) != pdTRUE) return false;
    if (!sensorData.isValid) return false;

    *outEncoder = (int32_t)sensorData.encoderValues[jointIndex];
    return true;
}

static bool moveJointToReserved(TaskSharedData_t* sharedData,
                                ServoBusManager* pBus,
                                uint8_t servoId,
                                uint8_t jointIndex,
                                const JointCalibrationConfig& cfg,
                                int32_t offset) {
    if (!sharedData || !pBus) return false;

    const int32_t targetEncoder = offset + cfg.angleReserved;
    const uint8_t ids[] = {servoId};
    const uint32_t beginMs = millis();

    while (millis() - beginMs < 3000) {
        int32_t currentEncoder = 0;
        if (!readLatestEncoderRaw(sharedData, jointIndex, &currentEncoder)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        int32_t encoderErr = wrapEncoderDelta(targetEncoder, currentEncoder);
        if (abs((int)encoderErr) <= 6) {
            return true;
        }

        if (pBus->syncReadPositions(ids, 1) <= 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        const int32_t currentServoPos = pBus->getAbsolutePosition(servoId);
        int32_t step = encoderErr / 8;
        if (step == 0) step = (encoderErr > 0) ? 1 : -1;

        int32_t stepLimit = (int32_t)abs((int)cfg.tightenStep) * 3;
        if (stepLimit < 1) stepLimit = 1;
        if (step > stepLimit) step = stepLimit;
        if (step < -stepLimit) step = -stepLimit;

        const int16_t targetPos = clampToServoPos(currentServoPos + step);
        pBus->setTarget(servoId, targetPos, cfg.tightenSpeed, cfg.tightenAcc);
        pBus->syncWriteAll();
        vTaskDelay(pdMS_TO_TICKS(cfg.settleMs));
    }

    return false;
}

void initDefaultCalibrationConfig(JointCalibrationConfig* cfg, uint8_t count) {
    if (!cfg) return;
    for (uint8_t i = 0; i < count; i++) {
        cfg[i].angleScope = 4000;
        cfg[i].bottomReserved = 120;
        cfg[i].topReserved = 120;
        cfg[i].angleReserved = 240;

        cfg[i].loadThreshold = 180;
        cfg[i].tightenStep = 10;
        cfg[i].tightenSpeed = 150;
        cfg[i].tightenAcc = 10;

        cfg[i].settleMs = 20;
        cfg[i].maxSearchMs = 5000;
    }
}

bool runSingleJointCalibration(TaskSharedData_t* sharedData,
                               uint8_t jointIndex,
                               const JointCalibrationConfig& cfg,
                               JointCalibrationResult* out) {
    if (!sharedData || !out || jointIndex >= ENCODER_TOTAL_NUM) return false;

    out->success = false;
    out->encoderMax = 0;
    out->offset = 0;
    out->angleMax = 0;

    const uint8_t bus = jointMap[jointIndex].busIndex;
    const uint8_t servoId = jointMap[jointIndex].servoID;
    ServoBusManager* pBus = getBusByIndex(bus);
    if (!pBus) return false;

    const uint8_t ids[] = {servoId};
    pBus->syncReadPositions(ids, 1);
    int16_t commandPos = clampToServoPos(pBus->getAbsolutePosition(servoId));

    bool seenBelowThreshold = false;
    const uint32_t searchBeginMs = millis();

    while (millis() - searchBeginMs < cfg.maxSearchMs) {
        int32_t tightenStep = (int32_t)cfg.tightenStep;
        if (tightenStep == 0) tightenStep = 1;
        commandPos = clampToServoPos((int32_t)commandPos + tightenStep);
        pBus->setTarget(servoId, commandPos, cfg.tightenSpeed, cfg.tightenAcc);
        pBus->syncWriteAll();
        vTaskDelay(pdMS_TO_TICKS(cfg.settleMs));

        const int16_t load = pBus->readLoad(servoId);
        if (load < cfg.loadThreshold) {
            seenBelowThreshold = true;
        }

        if (seenBelowThreshold && load >= cfg.loadThreshold) {
            int32_t encoderMax = 0;
            if (!readLatestEncoderRaw(sharedData, jointIndex, &encoderMax)) {
                return false;
            }

            out->encoderMax = encoderMax;
            out->offset = encoderMax - cfg.angleScope + cfg.bottomReserved;
            out->angleMax = cfg.angleScope - cfg.bottomReserved - cfg.topReserved;
            out->success = true;

            // Move the calibrated joint away from limit to reduce coupling impact.
            moveJointToReserved(sharedData, pBus, servoId, jointIndex, cfg, out->offset);
            return true;
        }
    }

    return false;
}

void taskServoCalibration(void* parameter) {
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    initDefaultCalibrationConfig(g_jointCalibConfig, ENCODER_TOTAL_NUM);
    memset(g_jointCalibResult, 0, sizeof(g_jointCalibResult));

    g_calibrationUIStatus = CALIB_STATUS_RUNNING;
    bool allSuccess = true;

    // Flowchart loop: i = 0 ... motorCount-1
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++) {
        JointCalibrationResult result;
        const bool ok = runSingleJointCalibration(sharedData, i, g_jointCalibConfig[i], &result);
        g_jointCalibResult[i] = result;
        if (!ok) allSuccess = false;

        vTaskDelay(pdMS_TO_TICKS(30));
    }

    g_calibrationUIStatus = allSuccess ? CALIB_STATUS_SUCCESS : CALIB_STATUS_FAILED;
    vTaskDelete(NULL);
}
