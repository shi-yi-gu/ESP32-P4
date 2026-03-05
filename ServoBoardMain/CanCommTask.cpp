#include "CanCommTask.h"
#include "driver/twai.h"

// CAN 驱动初始化（只安装一次）。
static void setupTwai() {
    static bool installed = false;
    if (installed) return;

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)TWAI_TX_PIN, 
        (gpio_num_t)TWAI_RX_PIN, 
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // 增大 RX 队列，降低任务繁忙时丢包概率。
    g_config.rx_queue_len = 64; 

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        installed = true;
        Serial.println("[CAN] Driver Installed OK");
    } else {
        // Serial.println("[CAN] Driver Install FAILED");
    }
}

void taskCanComm(void *parameter) {
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    setupTwai();

    // 接收缓存：累计多帧编码器数据，凑齐后再整体覆盖队列。
    static RemoteSensorData_t rxBuffer;
    memset(&rxBuffer, 0, sizeof(rxBuffer));

    twai_message_t rxMsg;
    uint32_t lastRxTime = 0;
    RemoteCommand_t txCmd;

    for (;;) {
        // ==========================================
        // 1. 接收 CAN 帧（非阻塞轮询）
        // ==========================================
        while (twai_receive(&rxMsg, 0) == ESP_OK) {
            lastRxTime = millis();

            // ------------------------------------------
            // 解析编码器数据帧 (0x100 ~ 0x105)
            // 格式: 每帧 8 字节，包含 4 路编码器数据；
            //      每路 2 字节，高字节在前（Big-Endian）。
            // ------------------------------------------
            if (rxMsg.identifier >= CAN_ID_ENC_BASE && rxMsg.identifier <= CAN_ID_ENC_LAST) {
                int frameIdx = rxMsg.identifier - CAN_ID_ENC_BASE; // 0~5
                int baseIdx = frameIdx * 4; // 每帧 4 路编码器

                for (int i = 0; i < 4; i++) {
                    int realIdx = baseIdx + i;
                    if (realIdx < ENCODER_TOTAL_NUM) {
                        // 与 HalTWAI::sendEncoderData() 打包格式保持一致。
                        // 发送端: buf[i*2] = (val >> 8); buf[i*2+1] = (val & 0xFF);
                        uint16_t val = ((uint16_t)rxMsg.data[i * 2] << 8) | rxMsg.data[i * 2 + 1];

                        rxBuffer.encoderValues[realIdx] = val;

                        // 简单无效值标记（0xFFFF 或 0x3FFF 通常表示异常）。
                        rxBuffer.errorFlags[realIdx] = (val == 0xFFFF || val == 0x3FFF) ? 1 : 0;
                    }
                }

                // 收到最后一帧后，将完整一组编码器数据写入队列。
                if (rxMsg.identifier == CAN_ID_ENC_LAST) {
                    rxBuffer.timestamp = millis();
                    rxBuffer.isValid = true;
                    xQueueOverwrite(sharedData->canRxQueue, &rxBuffer);
                }
            }
            // ------------------------------------------
            // 解析错误状态帧 (0x1F0)
            // 格式: 前 4 字节为 errorBitmap（Little-Endian）；
            //      后续字节为逐关节 errorFlags。
            // ------------------------------------------
            else if (rxMsg.identifier == CAN_ID_ERR_STATUS) {
                // 解析 errorBitmap（4 字节，Little-Endian）。
                uint32_t bitmap = 0;
                for (int i = 0; i < 4 && i < rxMsg.data_length_code; i++) {
                    bitmap |= ((uint32_t)rxMsg.data[i] << (i * 8));
                }
                rxBuffer.errorBitmap = bitmap;

                // 解析 errorFlags（剩余字节）。
                for (int i = 4; i < rxMsg.data_length_code && (i - 4) < ENCODER_TOTAL_NUM; i++) {
                    rxBuffer.errorFlags[i - 4] = rxMsg.data[i];
                }
            }
        }

        // ==========================================
        // 2. 超时检测（可选）
        // ==========================================
        if (millis() - lastRxTime > 500 && rxBuffer.isValid) {
            // 可选：超时后将缓存标记无效。
            // rxBuffer.isValid = false;
            // Serial.println("[CAN] RX Timeout");
        }

        // [TX] 发送指令（例如校准触发命令）。
        if (xQueueReceive(sharedData->canTxQueue, &txCmd, 0) == pdTRUE) {
            twai_message_t txMsg;
            txMsg.identifier = txCmd.cmdID;
            txMsg.extd = 0;
            txMsg.data_length_code = txCmd.len;
            memcpy(txMsg.data, txCmd.payload, txCmd.len);
            twai_transmit(&txMsg, pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(5)); // 5ms 周期
    }
}
