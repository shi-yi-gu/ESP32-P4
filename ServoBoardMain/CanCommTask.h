#ifndef CAN_COMM_TASK_H
#define CAN_COMM_TASK_H

#include <Arduino.h>
#include "TaskSharedData.h"

// CAN 任务栈大小在 TaskSharedData.h 中统一定义。

// CAN ID 定义：与下位机 HalTWAI 发送端保持一致。
#define CAN_ID_ENC_BASE   0x100
#define CAN_ID_ENC_LAST   (CAN_ID_ENC_BASE + (ENCODER_TOTAL_NUM + 3) / 4 - 1) // 21 路编码器对应 0x100~0x105
#define CAN_ID_ERR_STATUS 0x1F0

void taskCanComm(void *parameter);

#endif
