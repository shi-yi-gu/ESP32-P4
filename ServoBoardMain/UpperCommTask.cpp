#include "UpperCommTask.h"
#include "CanCommTask.h"
#include "TaskSharedData.h"
#include "CalibrationTask.h"

#include <math.h>
#include <string.h>

extern volatile uint8_t g_calibrationUIStatus;

// 上行帧格式（下位机 -> 上位机）: [0xFE][LEN][TYPE][PAYLOAD][0xFF]
#define PROTOCOL_HEADER 0xFE
#define PROTOCOL_TAIL 0xFF
#define PACKET_TYPE_SENSOR 0x01
#define PACKET_TYPE_CALIB_ACK 0x02
#define PACKET_TYPE_SERVO_ANGLE 0x03
#define PACKET_TYPE_JOINT1_DEBUG 0x04
#define PACKET_TYPE_SERVO_TELEM 0x05

#define PROTOCOL_DISCONNECT_SENTINEL ((int16_t)0x7FFF)

// 下行指令（上位机 -> 下位机）
#define CMD_CALIBRATE 0xCA
#define CMD_ANGLE_CTRL 0xCB
#define CMD_START 0xCC
#define CMD_STOP 0xCD
#define CMD_RESET 0xCE
#define CMD_CALIB_DATA 0xCF
#define CMD_MOTOR_POS 0xD0

static const size_t kFloatPayloadBytes = ENCODER_TOTAL_NUM * sizeof(float);
static const size_t kMotorPosPayloadBytes = SERVO_TOTAL_NUM * sizeof(uint16_t);
static const size_t kCmdAngleFrameBytes = 1 + kFloatPayloadBytes;
static const size_t kCmdCalibDataFrameBytes = 1 + kFloatPayloadBytes;
static const size_t kCmdMotorPosFrameBytes = 1 + kMotorPosPayloadBytes;
static const size_t kSerialRxBufferSize = 512;

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
    (void)pServo;

    uint8_t buffer[256];
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00; // LEN 占位，尾部统一回填

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
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            const int32_t angle = pServoAngle->servoAngles[i];
            buffer[idx++] = (uint8_t)((angle >> 24) & 0xFF);
            buffer[idx++] = (uint8_t)((angle >> 16) & 0xFF);
            buffer[idx++] = (uint8_t)((angle >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(angle & 0xFF);
        }
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            buffer[idx++] = pServoAngle->onlineStatus[i];
        }
    }
    else if (pTelemetry)
    {
        buffer[idx++] = PACKET_TYPE_SERVO_TELEM;
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            const int16_t speed = pTelemetry->speed[i];
            const int16_t load = pTelemetry->load[i];
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

    if (g_calibrationUIStatus != 0) {
        g_calibrationUIStatus = 0;
    }
}

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

static bool parseInt16ArrayBigEndian(const uint8_t* payload, size_t payloadLen, int32_t* outValues, uint8_t count)
{
    if (!payload || !outValues) {
        return false;
    }
    const size_t required = (size_t)count * sizeof(uint16_t);
    if (payloadLen < required) {
        return false;
    }
    for (uint8_t i = 0; i < count; i++)
    {
        const uint16_t raw = ((uint16_t)payload[(size_t)i * 2] << 8) | payload[(size_t)i * 2 + 1];
        outValues[i] = (int16_t)raw;
    }
    return true;
}

static void applyTargetAngles(TaskSharedData_t* sharedData, const float* angles, uint8_t count)
{
    if (count > ENCODER_TOTAL_NUM) count = ENCODER_TOTAL_NUM;
    if (xSemaphoreTake(sharedData->targetAnglesMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (uint8_t i = 0; i < count; i++)
        {
            sharedData->targetAngles[i] = angles[i];
        }
        xSemaphoreGive(sharedData->targetAnglesMutex);
    }
}

static void clearTargetAngles(TaskSharedData_t* sharedData)
{
    float zeroAngles[ENCODER_TOTAL_NUM] = {0.0f};
    applyTargetAngles(sharedData, zeroAngles, ENCODER_TOTAL_NUM);
}

static void applyMotorTargets(TaskSharedData_t* sharedData, const int32_t* targets, uint8_t count)
{
    if (!sharedData || !targets) {
        return;
    }
    if (count > SERVO_TOTAL_NUM) {
        count = SERVO_TOTAL_NUM;
    }
    for (uint8_t i = 0; i < count; i++)
    {
        sharedData->motorTargetRaw[i] = targets[i];
    }
}

static void clearMotorTargets(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; i++)
    {
        sharedData->motorTargetRaw[i] = 0;
    }
}

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

static void handleParsedCommand(TaskSharedData_t* sharedData, const uint8_t* frame, size_t frameLen)
{
    if (!sharedData || !frame || frameLen == 0) {
        return;
    }

    const uint8_t cmd = frame[0];

    if (cmd == CMD_CALIBRATE)
    {
        g_calibrationUIStatus = CALIB_STATUS_IDLE;
        return;
    }

    if (cmd == CMD_START)
    {
        // 启动后不强制切模式，只恢复使能与故障计数。
        sharedData->control_enabled = 1;
        sharedData->joint16_dual_feedback_fault = 0;
        return;
    }

    if (cmd == CMD_STOP)
    {
        sharedData->control_enabled = 0;
        return;
    }

    if (cmd == CMD_RESET)
    {
        // RESET: 回到角控模式，并清空角控/直控目标缓存。
        clearTargetAngles(sharedData);
        clearMotorTargets(sharedData);
        sharedData->control_enabled = 0;
        sharedData->control_mode = CONTROL_MODE_JOINT;
        sharedData->joint16_dual_feedback_fault = 0;
        return;
    }

    if (cmd == CMD_ANGLE_CTRL)
    {
        // 角控命令：严格按 21 路 float 小端解析。
        float parsedAngles[ENCODER_TOTAL_NUM] = {0.0f};
        if (parseFloatArrayLittleEndian(frame + 1, frameLen - 1, parsedAngles, ENCODER_TOTAL_NUM)) {
            applyTargetAngles(sharedData, parsedAngles, ENCODER_TOTAL_NUM);
            sharedData->control_mode = CONTROL_MODE_JOINT;
        }
        return;
    }

    if (cmd == CMD_CALIB_DATA)
    {
        // 标定零点数据：按 21 路写入缓存。
        float zeroRaw[ENCODER_TOTAL_NUM] = {0.0f};
        if (parseFloatArrayLittleEndian(frame + 1, frameLen - 1, zeroRaw, ENCODER_TOTAL_NUM)) {
            cacheCalibZeroRaw(sharedData, zeroRaw, ENCODER_TOTAL_NUM);
        }
        return;
    }

    if (cmd == CMD_MOTOR_POS)
    {
        // 直控命令：严格按 22 路 uint16 大端解析并切到直控模式。
        int32_t motorTargets[SERVO_TOTAL_NUM] = {0};
        if (parseInt16ArrayBigEndian(frame + 1, frameLen - 1, motorTargets, SERVO_TOTAL_NUM)) {
            applyMotorTargets(sharedData, motorTargets, SERVO_TOTAL_NUM);
            sharedData->control_mode = CONTROL_MODE_DIRECT_MOTOR;
        }
    }
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
        while (Serial.available())
        {
            const int byteVal = Serial.read();
            if (byteVal < 0) {
                break;
            }

            if (rxLen < sizeof(rxBuffer)) {
                rxBuffer[rxLen++] = (uint8_t)byteVal;
            } else {
                memmove(rxBuffer, rxBuffer + 1, sizeof(rxBuffer) - 1);
                rxBuffer[sizeof(rxBuffer) - 1] = (uint8_t)byteVal;
                rxLen = sizeof(rxBuffer);
            }
        }

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
                parseOffset += 1;
                continue;
            }

            if ((parseOffset + frameLen) > rxLen)
            {
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

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
