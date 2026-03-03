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
void sendDataPacket(ServoStatus_t *pServo, RemoteSensorData_t *pSensor, ServoAngleData_t *pServoAngle)
{
    uint8_t buffer[128]; // 增大缓冲区以容纳舵机角度数据
    size_t idx = 0;

    // 1. 帧头
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00; // 长度占位

    // 2. 负载内容
    if (g_calibrationUIStatus != 0)
    {
        // [高优先级] 校准状态反馈
        buffer[idx++] = PACKET_TYPE_CALIB_ACK;
        buffer[idx++] = g_calibrationUIStatus;
        // g_calibrationUIStatus = 0; // 发送后清除状态，避免重复
    }
    else if (pSensor)
    {
        // [常规] 传感器数据
        buffer[idx++] = PACKET_TYPE_SENSOR;
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            uint16_t val = pSensor->encoderValues[i];
            buffer[idx++] = (val >> 8) & 0xFF;
            buffer[idx++] = val & 0xFF;
        }
    }
    else if (pServoAngle)
    {
        // [新增] 舵机角度数据
        buffer[idx++] = PACKET_TYPE_SERVO_ANGLE;
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            int32_t angle = pServoAngle->servoAngles[i];
            buffer[idx++] = (angle >> 24) & 0xFF;
            buffer[idx++] = (angle >> 16) & 0xFF;
            buffer[idx++] = (angle >> 8) & 0xFF;
            buffer[idx++] = angle & 0xFF;
        }
        // 发送在线状态
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            buffer[idx++] = pServoAngle->onlineStatus[i];
        }
    }
    else
    {
        return; // 无数据
    }

    // 3. 填充长度与帧尾
    // buffer[1] = idx - 2;
    buffer[idx++] = PROTOCOL_TAIL;

    // 4. 正确计算 LEN: LEN = TYPE + PAYLOAD + TAIL
    // 注意：idx 已包含 TAIL，所以 LEN = idx - 2 (减去 FE 和 LEN 自身)
    buffer[1] = (uint8_t)(idx - 2);

    // 4. 写串口
    Serial.write(buffer, idx);

    // 【关键】仅在此处清除状态（防止重复发送）
    if (g_calibrationUIStatus != 0)
    {
        g_calibrationUIStatus = 0;
    }
}

// ============================================================
// 【新增】安全写入目标角度到共享数据
// ============================================================
void applyTargetAngles(TaskSharedData_t* sharedData, float* angles, uint8_t count) {
    if (count > ENCODER_TOTAL_NUM) count = ENCODER_TOTAL_NUM;
    if (xSemaphoreTake(sharedData->targetAnglesMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < count; i++) {
            sharedData->targetAngles[i] = angles[i];
        }
        xSemaphoreGive(sharedData->targetAnglesMutex);
    }
}
// --- 主任务函数 ---
void taskUpperComm(void *parameter)
{
    TaskSharedData_t *sharedData = (TaskSharedData_t *)parameter;
    RemoteSensorData_t sensorData;

    Serial.println("<<<SYS_READY>>>"); // 启动标志

    for (;;)
    {
        // ====================================================
        // [Part 1] 接收：处理来自 PC 的指令 (RX)
        // ====================================================
        if (Serial.available())
        {
            uint8_t rxByte = Serial.read();

            // 简单指令解析: 'c' 或 0xCA 触发校准
            if (rxByte == 'c' || rxByte == 0xCA)
            {
                RemoteCommand_t cmd;
                cmd.cmdID = 0x200; // 目标: S3
                cmd.len = 1;
                cmd.payload[0] = 0xCA; // 内容: Calibrate

                // 发送给 CAN 任务
                if (xQueueSend(sharedData->canTxQueue, &cmd, 0) == pdTRUE)
                {
                    g_calibrationUIStatus = 1; // 设置本地状态为 PENDING
                }
            }
            // 简单指令解析: 'b' 或 0xCB 设置目标角度
            if (rxByte == 'b' || rxByte == 0xCB)
            {
                float parsedAngles[ENCODER_TOTAL_NUM];
                // ... 原有的解析逻辑，将字节流转为 float 数组 ...
                for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
                {
                    //  memcpy(&parsedAngles[i], &payload[i * 4], sizeof(float));
                }

                // 【新增】将目标角度写入共享数据，供 taskSolver 读取
                applyTargetAngles(sharedData, parsedAngles, ENCODER_TOTAL_NUM);
            }

            // 可以在此扩展其他指令，例如 's' 停止等
        }

        // ====================================================
        // [Part 2] 发送：上传状态与数据 (TX)
        // ====================================================

        // 尝试从队列获取最新的传感器数据
        // 使用 Receive 移除队列中旧数据，保证实时性
        if (xQueueReceive(sharedData->canRxQueue, &sensorData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, &sensorData, NULL);
        }
        // 尝试从队列获取最新的舵机角度数据
        else {
            ServoAngleData_t servoAngleData;
            if (xQueueReceive(sharedData->servoAngleQueue, &servoAngleData, 0) == pdTRUE)
            {
                sendDataPacket(NULL, NULL, &servoAngleData);
            }
            // 如果没有传感器数据，但有校准状态变化 (Success/Fail)，也需要立即发送
            else if (g_calibrationUIStatus != 0)
            {
                sendDataPacket(NULL, NULL, NULL);
            }
        }

        // 任务调度延时
        // 既要保证 input 响应快，又要避免 output 发送太快堵塞串口
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
