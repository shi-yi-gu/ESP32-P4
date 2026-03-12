#include "UpperCommTask.h"
#include "CanCommTask.h"
#include "TaskSharedData.h"
#include "CalibrationTask.h"

#include <string.h>

extern volatile uint8_t g_calibrationUIStatus;

#define PROTOCOL_HEADER 0xFE
#define PROTOCOL_TAIL 0xFF
#define PACKET_TYPE_SENSOR 0x01
#define PACKET_TYPE_CALIB_ACK 0x02
#define PACKET_TYPE_SERVO_ANGLE 0x03
#define PACKET_TYPE_SERVO_TELEM 0x05
#define PACKET_TYPE_JOINT1_DEBUG 0x04
#define PROTOCOL_DISCONNECT_SENTINEL ((int16_t)0x7FFF)

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
    buffer[idx++] = 0x00; // length placeholder

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
                // Guard: avoid sending disconnect sentinel as valid data.
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
    buffer[1] = (uint8_t)(idx - 2); // TYPE + PAYLOAD + TAIL

    Serial.write(buffer, idx);

    // one-shot status
    if (g_calibrationUIStatus != 0)
    {
        g_calibrationUIStatus = 0;
    }
}

static void applyTargetAngles(TaskSharedData_t* sharedData, float* angles, uint8_t count)
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

void taskUpperComm(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    MappedAngleData_t mappedData;

    Serial.println("<<<SYS_READY>>>");

    for (;)
    {
        if (Serial.available())
        {
            uint8_t rxByte = Serial.read();

            // Calibration is disabled in current test mode.
            if (rxByte == 'c' || rxByte == 0xCA)
            {
                g_calibrationUIStatus = CALIB_STATUS_IDLE;
            }

            if (rxByte == 'b' || rxByte == 0xCB)
            {
                float parsedAngles[ENCODER_TOTAL_NUM] = {0.0f};
                // TODO: parse 21 float values from serial payload.
                applyTargetAngles(sharedData, parsedAngles, ENCODER_TOTAL_NUM);
            }
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
