#include "UpperCommTask.h"
#include "CanCommTask.h"
#include "TaskSharedData.h"
#include "CalibrationTask.h"

extern volatile uint8_t g_calibrationUIStatus;

#define PROTOCOL_HEADER 0xFE
#define PROTOCOL_TAIL 0xFF
#define PACKET_TYPE_SENSOR 0x01
#define PACKET_TYPE_CALIB_ACK 0x02
#define PACKET_TYPE_SERVO_ANGLE 0x03

static void sendDataPacket(ServoStatus_t* pServo, MappedAngleData_t* pMapped, ServoAngleData_t* pServoAngle)
{
    (void)pServo;

    uint8_t buffer[128];
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
            int16_t val = pMapped->validFlags[i] ? pMapped->angleValues[i] : (int16_t)0x7FFF;
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

    for (;;)
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

        if (xQueueReceive(sharedData->mappedAngleQueue, &mappedData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, &mappedData, NULL);
        }
        else
        {
            ServoAngleData_t servoAngleData;
            if (xQueueReceive(sharedData->servoAngleQueue, &servoAngleData, 0) == pdTRUE)
            {
                sendDataPacket(NULL, NULL, &servoAngleData);
            }
            else if (g_calibrationUIStatus != 0)
            {
                sendDataPacket(NULL, NULL, NULL);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
