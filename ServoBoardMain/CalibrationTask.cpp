#include "CalibrationTask.h"

#include <string.h>

#include "AngleSolver.h"
#include "JointCalibrationProfile.h"
#include "ServoBusManager.h"

extern ServoBusManager servoBus0;
extern ServoBusManager servoBus1;
extern ServoBusManager servoBus2;
extern ServoBusManager servoBus3;
extern JointMapItem jointMap[ENCODER_TOTAL_NUM];
extern volatile uint8_t g_calibrationUIStatus;

// 全局配置与结果表：
// - g_jointCalibConfig: 每个关节的校准参数（启动时从 profile 装载）
// - g_jointCalibResult: 每个关节的校准输出（供 Solver 读取）
JointCalibrationConfig g_jointCalibConfig[ENCODER_TOTAL_NUM];
JointCalibrationResult g_jointCalibResult[ENCODER_TOTAL_NUM];

// 磁编安装方向表：1=正向，-1=反向。
// 这里用于将不同安装方向的原始磁编值统一到同一数学方向。
int8_t g_encoderDirection[ENCODER_TOTAL_NUM] = {
    1,-1,-1, 1, 
    1,-1,-1, 1, 
    1,-1,-1, 1, 
    1,-1,-1, 1, 
    1, 1,-1,-1, 1
};

// 手动 offset 兜底（某关节未完成自动校准时使用）。
int32_t g_encoderOffsetManual[ENCODER_TOTAL_NUM] = {0};

// AS5600/14bit 磁编一圈计数。
static const int32_t kEncoderModulo = 16384;
static const int32_t kEncoderHalfTurn = kEncoderModulo / 2;

// 将原始磁编值按安装方向统一到“正向增大”的坐标系。
static int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction) {
    int32_t oriented = (int32_t)rawValue & (kEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kEncoderModulo - oriented) & (kEncoderModulo - 1);
    }
    return oriented;
}

// 根据 busIndex 选择对应总线实例。
static ServoBusManager* getBusByIndex(uint8_t busIndex) {
    switch (busIndex) {
        case 0: return &servoBus0;
        case 1: return &servoBus1;
        case 2: return &servoBus2;
        case 3: return &servoBus3;
        default: return NULL;
    }
}

// 舵机目标值限幅，防止超出驱动可接受范围。
static int16_t clampToServoPos(int32_t value) {
    if (value > 30719) return 30719;
    if (value < -30719) return -30719;
    return (int16_t)value;
}

// 计算 target-actual 的最短环形差值，结果落在 [-8192, 8192] 附近。
static int32_t wrapEncoderDelta(int32_t target, int32_t actual) {
    int32_t delta = target - actual;
    while (delta > kEncoderHalfTurn) delta -= kEncoderModulo;
    while (delta < -kEncoderHalfTurn) delta += kEncoderModulo;
    return delta;
}

// 从 canRxQueue 读取最新磁编值（只读不出队），并完成方向统一。
static bool readLatestEncoderRaw(TaskSharedData_t* sharedData, uint8_t jointIndex, int32_t* outEncoder) {
    if (!sharedData || !outEncoder || jointIndex >= ENCODER_TOTAL_NUM) return false;

    RemoteSensorData_t sensorData;
    if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) != pdTRUE) return false;
    if (!sensorData.isValid) return false;
    if (sensorData.errorFlags[jointIndex]) return false;

    *outEncoder = orientEncoderRaw(sensorData.encoderValues[jointIndex], g_encoderDirection[jointIndex]);
    return true;
}

// 校准后把关节从极限点移到预留角，减少后续关节校准时的机械耦合影响。
// 这里使用一个轻量闭环：磁编误差 -> 舵机小步增量 -> 等待稳定 -> 重复。
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
            // 误差进入容差窗口，认为停靠完成。
            return true;
        }

        if (pBus->syncReadPositions(ids, 1) <= 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        const int32_t currentServoPos = pBus->getAbsolutePosition(servoId);
        int32_t step = encoderErr / 8;
        if (step == 0) step = (encoderErr > 0) ? 1 : -1;

        // 限制每轮步进，避免大误差时一次命令过大。
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
    // 从独立参数表加载每个关节的校准配置。
    loadJointCalibrationProfile(cfg, count);
}

// 单关节校准流程：
// 1) 获取关节对应舵机并从当前位置开始；
// 2) 按 tightenStep 持续收紧，读取舵机负载；
// 3) 观察到负载“先低于阈值，再回升到阈值以上”视为触底；
// 4) 读取触底时磁编值 encoderMax，计算 offset/angleMax；
// 5) 将关节回拉到预留角，避免影响后续关节。
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

    // 从当前绝对位置开始增量收紧，避免突跳。
    const uint8_t ids[] = {servoId};
    pBus->syncReadPositions(ids, 1);
    int16_t commandPos = clampToServoPos(pBus->getAbsolutePosition(servoId));

    bool seenBelowThreshold = false;
    const uint32_t searchBeginMs = millis();

    while (millis() - searchBeginMs < cfg.maxSearchMs) {
        // 发送一步收紧命令。
        int32_t tightenStep = (int32_t)cfg.tightenStep;
        if (tightenStep == 0) tightenStep = 1;
        commandPos = clampToServoPos((int32_t)commandPos + tightenStep);
        pBus->setTarget(servoId, commandPos, cfg.tightenSpeed, cfg.tightenAcc);
        pBus->syncWriteAll();
        vTaskDelay(pdMS_TO_TICKS(cfg.settleMs));

        const int16_t load = pBus->readLoad(servoId);
        if (load < cfg.loadThreshold) {
            // 先看到“低于阈值”，说明仍在自由段或刚接触。
            seenBelowThreshold = true;
        }

        if (seenBelowThreshold && load >= cfg.loadThreshold) {
            // 低->高跨越出现，判定到达机械边界附近。
            int32_t encoderMax = 0;
            if (!readLatestEncoderRaw(sharedData, jointIndex, &encoderMax)) {
                return false;
            }

            out->encoderMax = encoderMax;

            // offset 定义：把“可用角度下边界”对齐到 0。
            // 下边界 = encoderMax - angleScope + bottomReserved
            out->offset = encoderMax - cfg.angleScope + cfg.bottomReserved;

            // 有效角度范围 = 总行程 - 上下预留。
            out->angleMax = cfg.angleScope - cfg.bottomReserved - cfg.topReserved;
            out->success = true;

            // 校准后将该关节移到预留位置，降低耦合影响。
            moveJointToReserved(sharedData, pBus, servoId, jointIndex, cfg, out->offset);
            return true;
        }
    }

    // 超时仍未触发阈值跨越，判定失败。
    return false;
}

void taskServoCalibration(void* parameter) {
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    // 初始化配置与结果缓存。
    initDefaultCalibrationConfig(g_jointCalibConfig, ENCODER_TOTAL_NUM);
    memset(g_jointCalibResult, 0, sizeof(g_jointCalibResult));

    g_calibrationUIStatus = CALIB_STATUS_RUNNING;
    bool allSuccess = true;

    // 按索引顺序逐个关节校准。
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++) {
        JointCalibrationResult result;
        const bool ok = runSingleJointCalibration(sharedData, i, g_jointCalibConfig[i], &result);
        g_jointCalibResult[i] = result;
        if (!ok) allSuccess = false;

        // 给总线和机构一点缓冲时间，降低串扰。
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    // 全局状态上报：有一个失败即整体失败。
    g_calibrationUIStatus = allSuccess ? CALIB_STATUS_SUCCESS : CALIB_STATUS_FAILED;
    vTaskDelete(NULL);
}
