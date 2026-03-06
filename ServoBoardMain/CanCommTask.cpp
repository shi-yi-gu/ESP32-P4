#include "CanCommTask.h"
#include "driver/twai.h"

#include <string.h>

// Treat only 0xFFFF as invalid raw value.
// 0x3FFF is a legal max value for 14-bit encoders and should not trigger disconnect.
static inline bool isInvalidEncoderRaw(uint16_t value)
{
    return value == 0xFFFF;
}

static void setupTwai()
{
    static bool installed = false;
    if (installed) return;

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)TWAI_TX_PIN,
        (gpio_num_t)TWAI_RX_PIN,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    g_config.rx_queue_len = 64;

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

    static RemoteSensorData_t rxBuffer;
    memset(&rxBuffer, 0, sizeof(rxBuffer));

    twai_message_t rxMsg;
    uint32_t lastRxTime = 0;
    RemoteCommand_t txCmd;

    for (;;)
    {
        // 1) RX loop (non-blocking)
        while (twai_receive(&rxMsg, 0) == ESP_OK)
        {
            lastRxTime = millis();

            // Encoder frames: 0x100 ~ 0x105, each frame carries 4 channels.
            if (rxMsg.identifier >= CAN_ID_ENC_BASE && rxMsg.identifier <= CAN_ID_ENC_LAST)
            {
                int frameIdx = rxMsg.identifier - CAN_ID_ENC_BASE;
                int baseIdx = frameIdx * 4;

                for (int i = 0; i < 4; i++)
                {
                    int realIdx = baseIdx + i;
                    if (realIdx < ENCODER_TOTAL_NUM)
                    {
                        uint16_t val = ((uint16_t)rxMsg.data[i * 2] << 8) | rxMsg.data[i * 2 + 1];
                        rxBuffer.encoderValues[realIdx] = val;
                        rxBuffer.errorFlags[realIdx] = isInvalidEncoderRaw(val) ? 1 : 0;
                    }
                }

                // Publish a complete snapshot when the last encoder frame arrives.
                if (rxMsg.identifier == CAN_ID_ENC_LAST)
                {
                    rxBuffer.timestamp = millis();
                    rxBuffer.isValid = true;
                    xQueueOverwrite(sharedData->canRxQueue, &rxBuffer);
                }
            }
            // Error status frame: 0x1F0
            else if (rxMsg.identifier == CAN_ID_ERR_STATUS)
            {
                uint32_t bitmap = 0;
                for (int i = 0; i < 4 && i < rxMsg.data_length_code; i++)
                {
                    bitmap |= ((uint32_t)rxMsg.data[i] << (i * 8));
                }
                rxBuffer.errorBitmap = bitmap;

                for (int i = 4; i < rxMsg.data_length_code && (i - 4) < ENCODER_TOTAL_NUM; i++)
                {
                    rxBuffer.errorFlags[i - 4] = rxMsg.data[i];
                }
            }
        }

        // 2) Optional timeout hook
        if (millis() - lastRxTime > 500 && rxBuffer.isValid)
        {
            // Keep current behavior: do not forcibly invalidate on transient timeout.
            // rxBuffer.isValid = false;
        }

        // 3) TX queue
        if (xQueueReceive(sharedData->canTxQueue, &txCmd, 0) == pdTRUE)
        {
            twai_message_t txMsg;
            txMsg.identifier = txCmd.cmdID;
            txMsg.extd = 0;
            txMsg.data_length_code = txCmd.len;
            memcpy(txMsg.data, txCmd.payload, txCmd.len);
            twai_transmit(&txMsg, pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
