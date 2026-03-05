#include "UpperCommTask.h"
#include "CanCommTask.h"
#include "TaskSharedData.h"

extern volatile uint8_t g_calibrationUIStatus;

// --- 协议定义 ---
#define PROTOCOL_HEADER 0xFE
#define PROTOCOL_TAIL 0xFF
#define PACKET_TYPE_SENSOR 0x01
#define PACKET_TYPE_CALIB_ACK 0x02
#define PACKET_TYPE_SERVO_ANGLE 0x03

// 内部辅助：发送数据包
void sendDataPacket(ServoStatus_t *pServo, MappedAngleData_t *pMapped, ServoAngleData_t *pServoAngle)
{
    (void)pServo;

    uint8_t buffer[128];
    size_t idx = 0;

    // 1. 帧头
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00; // 长度占位

    // 2. 负载
    if (g_calibrationUIStatus != 0)
    {
        buffer[idx++] = PACKET_TYPE_CALIB_ACK;
        buffer[idx++] = g_calibrationUIStatus;
    }
    else if (pMapped)
    {
        // 发送映射后角度计数（与 PID 输入同源）
        buffer[idx++] = PACKET_TYPE_SENSOR;
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            // 0x7FFF 作为无效值哨兵
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

    // 3. 帧尾
    buffer[idx++] = PROTOCOL_TAIL;

    // 4. LEN = TYPE + PAYLOAD + TAIL
    buffer[1] = (uint8_t)(idx - 2);

    // 5. 发送
    Serial.write(buffer, idx);

    // 发送后清除一次性状态
    if (g_calibrationUIStatus != 0)
    {
        g_calibrationUIStatus = 0;
    }
}

// 安全写入目标角度到共享数据
void applyTargetAngles(TaskSharedData_t* sharedData, float* angles, uint8_t count)
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

void taskUpperComm(void *parameter)
{
    TaskSharedData_t *sharedData = (TaskSharedData_t *)parameter;
    MappedAngleData_t mappedData;

    Serial.println("<<<SYS_READY>>>");

    for (;;)
    {
        // [Part 1] 接收来自 PC 的指令
        if (Serial.available())
        {
            uint8_t rxByte = Serial.read();

            // 'c' 或 0xCA 触发校准
            if (rxByte == 'c' || rxByte == 0xCA)
            {
                RemoteCommand_t cmd;
                cmd.cmdID = 0x200;
                cmd.len = 1;
                cmd.payload[0] = 0xCA;

                if (xQueueSend(sharedData->canTxQueue, &cmd, 0) == pdTRUE)
                {
                    g_calibrationUIStatus = 1;
                }
            }

            // 'b' 或 0xCB 设置目标角度（当前占位解析）
            if (rxByte == 'b' || rxByte == 0xCB)
            {
                float parsedAngles[ENCODER_TOTAL_NUM] = {0.0f};
                // TODO: 按你的协议从串口载荷解析 21 个 float
                applyTargetAngles(sharedData, parsedAngles, ENCODER_TOTAL_NUM);
            }
        }

        // [Part 2] 发送状态与数据
        // 优先发送映射角度，保证显示与控制口径一致
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
