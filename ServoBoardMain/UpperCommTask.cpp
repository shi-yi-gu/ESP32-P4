#include "UpperCommTask.h"
#include "CanCommTask.h"
#include "TaskSharedData.h"
#include "CalibrationTask.h"

#include <math.h>
#include <string.h>

// UpperCommTask 模块职责：
// 1) 负责与上位机的串口收发；
// 2) 将下位机状态打包成统一帧格式上报；
// 3) 解析下行命令并同步到共享控制状态（目标角、控制使能、校准缓存）。

extern volatile uint8_t g_calibrationUIStatus;

// 上行帧协议：[0xFE][LEN][TYPE][PAYLOAD][0xFF]
#define PROTOCOL_HEADER 0xFE
#define PROTOCOL_TAIL 0xFF
#define PACKET_TYPE_SENSOR 0x01
#define PACKET_TYPE_CALIB_ACK 0x02
#define PACKET_TYPE_SERVO_ANGLE 0x03
#define PACKET_TYPE_SERVO_TELEM 0x05
#define PACKET_TYPE_JOINT1_DEBUG 0x04
// 0x7FFF 在协议中保留为“断连”哨兵，避免与正常角度值冲突。
#define PROTOCOL_DISCONNECT_SENTINEL ((int16_t)0x7FFF)

// 下行命令字节定义（与 desktop/protocol.py 保持一致）。
#define CMD_CALIBRATE 0xCA
#define CMD_ANGLE_CTRL 0xCB
#define CMD_START 0xCC
#define CMD_STOP 0xCD
#define CMD_RESET 0xCE
#define CMD_CALIB_DATA 0xCF
#define CMD_MOTOR_POS 0xD0

// 不同命令帧长度（命令字节 + payload）定义，用于串口流切包。
static const size_t kFloatPayloadBytes = ENCODER_TOTAL_NUM * sizeof(float);
static const size_t kMotorPosPayloadBytes = ENCODER_TOTAL_NUM * sizeof(uint16_t);
static const size_t kCmdAngleFrameBytes = 1 + kFloatPayloadBytes;
static const size_t kCmdCalibDataFrameBytes = 1 + kFloatPayloadBytes;
static const size_t kCmdMotorPosFrameBytes = 1 + kMotorPosPayloadBytes;
// 接收缓冲用于处理分片和粘包，容量足够覆盖多帧命令。
static const size_t kSerialRxBufferSize = 512;

// 以大端格式写入 float（用于上行 debug 包字段编码）。
static void appendFloatBigEndian(uint8_t* buffer, size_t* idx, float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    buffer[(*idx)++] = (uint8_t)((bits >> 24) & 0xFF);
    buffer[(*idx)++] = (uint8_t)((bits >> 16) & 0xFF);
    buffer[(*idx)++] = (uint8_t)((bits >> 8) & 0xFF);
    buffer[(*idx)++] = (uint8_t)(bits & 0xFF);
}

static void sendDataPacket(ServoStatus_t* pServo,
                           MappedAngleData_t* pMapped,
                           ServoAngleData_t* pServoAngle,
                           ServoTelemetryData_t* pTelemetry,
                           JointDebugData_t* pJointDebug)
{
    // 预留入参，当前版本暂未使用该结构体。
    (void)pServo;

    uint8_t buffer[256];
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00; // length placeholder

    // 发送优先级：
    // 1) 校准应答（一次性上报）
    // 2) 映射角度
    // 3) 舵机角度
    // 4) 遥测
    // 5) 单关节调试
    if (g_calibrationUIStatus != 0)
    {
        buffer[idx++] = PACKET_TYPE_CALIB_ACK;
        buffer[idx++] = g_calibrationUIStatus;
    }
    else if (pMapped)
    {
        buffer[idx++] = PACKET_TYPE_SENSOR;
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            int16_t val = PROTOCOL_DISCONNECT_SENTINEL;
            const bool channelValid = pMapped->isValid && (pMapped->validFlags[i] != 0);
            if (channelValid) {
                val = pMapped->angleValues[i];
                // 保护：避免把断连哨兵当作正常值上报。
                if (val == PROTOCOL_DISCONNECT_SENTINEL) {
                    val = (int16_t)0x7FFE;
                }
            }
            buffer[idx++] = (uint8_t)(((uint16_t)val >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)((uint16_t)val & 0xFF);
        }
    }
    else if (pServoAngle)
    {
        buffer[idx++] = PACKET_TYPE_SERVO_ANGLE;
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            int32_t angle = pServoAngle->servoAngles[i];
            buffer[idx++] = (angle >> 24) & 0xFF;
            buffer[idx++] = (angle >> 16) & 0xFF;
            buffer[idx++] = (angle >> 8) & 0xFF;
            buffer[idx++] = angle & 0xFF;
        }

        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            buffer[idx++] = pServoAngle->onlineStatus[i];
        }
    }
    else if (pTelemetry)
    {
        buffer[idx++] = PACKET_TYPE_SERVO_TELEM;
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            int16_t speed = pTelemetry->speed[i];
            int16_t load = pTelemetry->load[i];
            buffer[idx++] = (uint8_t)(((uint16_t)speed >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)((uint16_t)speed & 0xFF);
            buffer[idx++] = (uint8_t)(((uint16_t)load >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)((uint16_t)load & 0xFF);
            buffer[idx++] = pTelemetry->voltage[i];
            buffer[idx++] = pTelemetry->temperature[i];
            buffer[idx++] = pTelemetry->onlineStatus[i];
        }
    }
    else if (pJointDebug)
    {
        buffer[idx++] = PACKET_TYPE_JOINT1_DEBUG;
        buffer[idx++] = pJointDebug->jointIndex;
        buffer[idx++] = pJointDebug->valid;
        appendFloatBigEndian(buffer, &idx, pJointDebug->targetDeg);
        appendFloatBigEndian(buffer, &idx, pJointDebug->magActualDeg);
        appendFloatBigEndian(buffer, &idx, pJointDebug->loop1Output);
        appendFloatBigEndian(buffer, &idx, pJointDebug->loop2Actual);
        appendFloatBigEndian(buffer, &idx, pJointDebug->loop2Output);
    }
    else
    {
        return;
    }

    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2); // LEN = TYPE + PAYLOAD + TAIL

    Serial.write(buffer, idx);

    // 校准状态包是一次性语义，发出后立即清零。
    if (g_calibrationUIStatus != 0)
    {
        g_calibrationUIStatus = 0;
    }
}

// 从小端字节流解析 float（下行命令 payload 使用小端）。
static float decodeFloatLittleEndian(const uint8_t* data)
{
    uint32_t bits = 0;
    bits |= (uint32_t)data[0];
    bits |= ((uint32_t)data[1] << 8);
    bits |= ((uint32_t)data[2] << 16);
    bits |= ((uint32_t)data[3] << 24);

    float out = 0.0f;
    memcpy(&out, &bits, sizeof(out));
    return out;
}

// 将 payload 解析为 float 数组；长度不足或空指针时返回 false。
static bool parseFloatArrayLittleEndian(const uint8_t* payload, size_t payloadLen, float* outValues, uint8_t count)
{
    if (!payload || !outValues) {
        return false;
    }

    const size_t required = (size_t)count * sizeof(float);
    if (payloadLen < required) {
        return false;
    }

    for (uint8_t i = 0; i < count; i++)
    {
        outValues[i] = decodeFloatLittleEndian(payload + (size_t)i * sizeof(float));
    }
    return true;
}

// 将目标角写入共享数据（带互斥保护）。
static void applyTargetAngles(TaskSharedData_t* sharedData, const float* angles, uint8_t count)
{
    if (count > ENCODER_TOTAL_NUM) count = ENCODER_TOTAL_NUM;
    if (xSemaphoreTake(sharedData->targetAnglesMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (int i = 0; i < count; i++)
        {
            sharedData->targetAngles[i] = angles[i];
        }
        xSemaphoreGive(sharedData->targetAnglesMutex);
    }
}

// 重置目标角为全零，常用于 reset/legacy 控制路径。
static void clearTargetAngles(TaskSharedData_t* sharedData)
{
    float zeroAngles[ENCODER_TOTAL_NUM] = {0.0f};
    applyTargetAngles(sharedData, zeroAngles, ENCODER_TOTAL_NUM);
}

// 缓存上位机下发的校准零点（当前阶段仅缓存，不直接改求解链路）。
static void cacheCalibZeroRaw(TaskSharedData_t* sharedData, const float* values, uint8_t count)
{
    if (!sharedData || !values) {
        return;
    }

    if (count > ENCODER_TOTAL_NUM) {
        count = ENCODER_TOTAL_NUM;
    }

    for (uint8_t i = 0; i < count; i++)
    {
        float v = values[i];
        // 保护：非法浮点值按 0 处理，避免污染缓存。
        if (!isfinite(v)) {
            v = 0.0f;
        }
        sharedData->calib_zero_raw_cache[i] = (int32_t)lroundf(v);
    }
    for (uint8_t i = count; i < ENCODER_TOTAL_NUM; i++)
    {
        sharedData->calib_zero_raw_cache[i] = 0;
    }
    sharedData->calib_zero_raw_valid = 1;
}

// 返回命令总长度（含命令字节）。未知命令返回 0。
static size_t getCommandFrameLength(uint8_t cmd)
{
    switch (cmd)
    {
        case CMD_CALIBRATE:
        case CMD_START:
        case CMD_STOP:
        case CMD_RESET:
            return 1;
        case CMD_ANGLE_CTRL:
            return kCmdAngleFrameBytes;
        case CMD_CALIB_DATA:
            return kCmdCalibDataFrameBytes;
        case CMD_MOTOR_POS:
            return kCmdMotorPosFrameBytes;
        default:
            return 0;
    }
}

// 兼容旧版单字节命令：
// 'c' -> 校准状态回到 IDLE；'b' -> 清空目标角。
static void handleLegacySingleByteCommand(TaskSharedData_t* sharedData, uint8_t cmd)
{
    if (cmd == (uint8_t)'c') {
        g_calibrationUIStatus = CALIB_STATUS_IDLE;
        return;
    }

    if (cmd == (uint8_t)'b') {
        clearTargetAngles(sharedData);
    }
}

// 处理已切好的完整命令帧。
static void handleParsedCommand(TaskSharedData_t* sharedData, const uint8_t* frame, size_t frameLen)
{
    if (!sharedData || !frame || frameLen == 0) {
        return;
    }

    const uint8_t cmd = frame[0];

    if (cmd == CMD_CALIBRATE)
    {
        // 当前测试阶段不执行自动标定，仅复位 UI 状态。
        g_calibrationUIStatus = CALIB_STATUS_IDLE;
        return;
    }

    if (cmd == CMD_START)
    {
        // 打开控制使能，Solver 才会从目标角驱动输出。
        sharedData->control_enabled = 1;
        return;
    }

    if (cmd == CMD_STOP)
    {
        // 关闭控制使能，Solver 进入保持当前位置逻辑。
        sharedData->control_enabled = 0;
        return;
    }

    if (cmd == CMD_RESET)
    {
        // reset 语义：清目标 + 停控。
        clearTargetAngles(sharedData);
        sharedData->control_enabled = 0;
        return;
    }

    if (cmd == CMD_ANGLE_CTRL)
    {
        float parsedAngles[ENCODER_TOTAL_NUM] = {0.0f};
        if (parseFloatArrayLittleEndian(frame + 1, frameLen - 1, parsedAngles, ENCODER_TOTAL_NUM)) {
            applyTargetAngles(sharedData, parsedAngles, ENCODER_TOTAL_NUM);
        }
        return;
    }

    if (cmd == CMD_CALIB_DATA)
    {
        float zeroRaw[ENCODER_TOTAL_NUM] = {0.0f};
        if (parseFloatArrayLittleEndian(frame + 1, frameLen - 1, zeroRaw, ENCODER_TOTAL_NUM)) {
            cacheCalibZeroRaw(sharedData, zeroRaw, ENCODER_TOTAL_NUM);
        }
        return;
    }

    // CMD_MOTOR_POS：目前仅支持帧级兼容，业务逻辑暂不生效。
}

void taskUpperComm(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    MappedAngleData_t mappedData;
    uint8_t rxBuffer[kSerialRxBufferSize];
    size_t rxLen = 0;

    Serial.println("<<<SYS_READY>>>");

    for (;;)
    {
        // 1) 收集串口字节流到本地缓冲，处理突发输入。
        while (Serial.available())
        {
            int byteVal = Serial.read();
            if (byteVal < 0) {
                break;
            }

            if (rxLen < sizeof(rxBuffer)) {
                rxBuffer[rxLen++] = (uint8_t)byteVal;
            } else {
                // 缓冲满时丢弃最旧字节，保证解析继续推进。
                memmove(rxBuffer, rxBuffer + 1, sizeof(rxBuffer) - 1);
                rxBuffer[sizeof(rxBuffer) - 1] = (uint8_t)byteVal;
                rxLen = sizeof(rxBuffer);
            }
        }

        // 2) 从缓冲中按“命令 + 固定长度 payload”切帧并处理。
        size_t parseOffset = 0;
        while (parseOffset < rxLen)
        {
            const uint8_t cmd = rxBuffer[parseOffset];

            if (cmd == (uint8_t)'c' || cmd == (uint8_t)'b')
            {
                handleLegacySingleByteCommand(sharedData, cmd);
                parseOffset += 1;
                continue;
            }

            const size_t frameLen = getCommandFrameLength(cmd);
            if (frameLen == 0)
            {
                // 未知命令：丢弃该字节继续同步。
                parseOffset += 1;
                continue;
            }

            if ((parseOffset + frameLen) > rxLen)
            {
                // 帧未收齐：等待下一轮串口补齐。
                break;
            }

            handleParsedCommand(sharedData, rxBuffer + parseOffset, frameLen);
            parseOffset += frameLen;
        }

        if (parseOffset > 0)
        {
            const size_t remain = rxLen - parseOffset;
            if (remain > 0) {
                memmove(rxBuffer, rxBuffer + parseOffset, remain);
            }
            rxLen = remain;
        }

        // 3) 上行状态发送（按优先级依次尝试）。
        bool sentPacket = false;
        if (xQueueReceive(sharedData->mappedAngleQueue, &mappedData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, &mappedData, NULL, NULL, NULL);
            sentPacket = true;
        }

        JointDebugData_t jointDebugData;
        while (xQueueReceive(sharedData->jointDebugQueue, &jointDebugData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, &jointDebugData);
            sentPacket = true;
        }

        ServoTelemetryData_t telemetryData;
        if (xQueueReceive(sharedData->servoTelemetryQueue, &telemetryData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, &telemetryData, NULL);
            sentPacket = true;
        }

        if (!sentPacket)
        {
            ServoAngleData_t servoAngleData;
            if (xQueueReceive(sharedData->servoAngleQueue, &servoAngleData, 0) == pdTRUE)
            {
                sendDataPacket(NULL, NULL, &servoAngleData, NULL, NULL);
            }
            else if (g_calibrationUIStatus != 0)
            {
                sendDataPacket(NULL, NULL, NULL, NULL, NULL);
            }
        }

        // 固定任务节拍，降低串口与队列轮询占用。
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
