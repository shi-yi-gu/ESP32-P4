#include "CanCommTask.h"
#include "driver/twai.h"

#include <string.h>

// CanCommTask 模块职责：
// 1) 接收 CAN/TWAI 总线数据并聚合为 RemoteSensorData_t 快照；
// 2) 将快照发布到 sharedData->canRxQueue；
// 3) 从 sharedData->canTxQueue 取控制命令并转发到 CAN 总线。

// 仅把 0xFFFF 视为无效原始值。
// 0x3FFF 是 14-bit 编码器的合法上限，不能误判为断连。
static inline bool isInvalidEncoderRaw(uint16_t value)
{
    return value == 0xFFFF;
}

// TWAI 驱动幂等初始化：只在首次进入任务时安装并启动一次。
static void setupTwai()
{
    static bool installed = false;
    if (installed) return;

    // 当前链路固定 1Mbps，接收过滤器放开，交由上层按 CAN ID 分拣。
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)TWAI_TX_PIN,
        (gpio_num_t)TWAI_RX_PIN,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // 提高 RX 队列深度，降低突发帧丢失概率。
    g_config.rx_queue_len = 64;

    // 安装失败时保持现有行为：不额外重试，不改任务主循环结构。
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        installed = true;
        Serial.println("[CAN] Driver Installed OK");
    }
}

void taskCanComm(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    setupTwai();

    // 持久化接收快照缓存：累计多帧后一次性发布到 canRxQueue。
    static RemoteSensorData_t rxBuffer;
    memset(&rxBuffer, 0, sizeof(rxBuffer));

    twai_message_t rxMsg;
    uint32_t lastRxTime = 0;
    RemoteCommand_t txCmd;

    for (;;)
    {
        // 1) RX 循环（非阻塞）：每轮尽量清空 TWAI 接收队列，降低积压。
        while (twai_receive(&rxMsg, 0) == ESP_OK)
        {
            lastRxTime = millis();

            // 编码器帧：0x100 ~ CAN_ID_ENC_LAST，每帧携带 4 路编码器（每路 2 字节）。
            if (rxMsg.identifier >= CAN_ID_ENC_BASE && rxMsg.identifier <= CAN_ID_ENC_LAST)
            {
                int frameIdx = rxMsg.identifier - CAN_ID_ENC_BASE;
                int baseIdx = frameIdx * 4;

                for (int i = 0; i < 4; i++)
                {
                    int realIdx = baseIdx + i;
                    // 越界保护：最后一帧可能含填充位，超过 ENCODER_TOTAL_NUM 的通道直接忽略。
                    if (realIdx < ENCODER_TOTAL_NUM)
                    {
                        // 协议按高字节在前组包。
                        uint16_t val = ((uint16_t)rxMsg.data[i * 2] << 8) | rxMsg.data[i * 2 + 1];
                        rxBuffer.encoderValues[realIdx] = val;
                        rxBuffer.errorFlags[realIdx] = isInvalidEncoderRaw(val) ? 1 : 0;
                    }
                }

                // 以最后一帧到达作为“完整快照”触发点，避免消费者读到半包数据。
                if (rxMsg.identifier == CAN_ID_ENC_LAST)
                {
                    rxBuffer.timestamp = millis();
                    rxBuffer.isValid = true;
                    xQueueOverwrite(sharedData->canRxQueue, &rxBuffer);
                }
            }
            // 错误状态帧：0x1F0。
            else if (rxMsg.identifier == CAN_ID_ERR_STATUS)
            {
                uint32_t bitmap = 0;
                // DLC 防护：位图最多读取前 4 字节。
                for (int i = 0; i < 4 && i < rxMsg.data_length_code; i++)
                {
                    bitmap |= ((uint32_t)rxMsg.data[i] << (i * 8));
                }
                rxBuffer.errorBitmap = bitmap;

                // 余下字节按通道错误标志写入，且受 ENCODER_TOTAL_NUM 边界保护。
                for (int i = 4; i < rxMsg.data_length_code && (i - 4) < ENCODER_TOTAL_NUM; i++)
                {
                    rxBuffer.errorFlags[i - 4] = rxMsg.data[i];
                }
            }
        }

        // 2) 超时钩子（当前仅占位）：短时无帧不强制置 invalid，避免瞬时抖动引发状态闪断。
        if (millis() - lastRxTime > 500 && rxBuffer.isValid)
        {
            // 保持现有行为：暂不主动拉低有效位。
            // rxBuffer.isValid = false;
        }

        // 3) TX 队列转发：从共享队列取命令并原样封装为 TWAI 帧发送。
        if (xQueueReceive(sharedData->canTxQueue, &txCmd, 0) == pdTRUE)
        {
            twai_message_t txMsg;
            txMsg.identifier = txCmd.cmdID;
            txMsg.extd = 0;
            txMsg.data_length_code = txCmd.len;
            memcpy(txMsg.data, txCmd.payload, txCmd.len);
            twai_transmit(&txMsg, pdMS_TO_TICKS(10));
        }

        // 固定任务节拍，平衡实时性与 CPU 占用。
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
