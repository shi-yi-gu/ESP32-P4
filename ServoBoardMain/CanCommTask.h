#ifndef CAN_COMM_TASK_H
#define CAN_COMM_TASK_H

#include <Arduino.h>
#include "TaskSharedData.h"

// CAN 任务栈大小在 TaskSharedData.h 中统一定义。
// CAN ID 定义与下位机 HalTWAI 发送侧保持一致：
// 1) 编码器数据按 4 路/帧连续映射；
// 2) 关节索引与数组下标一一对应，不可随意换序；
// 3) 错误状态使用独立帧 ID（CAN_ID_ERR_STATUS）。
#define CAN_ID_ENC_BASE   0x100
#define CAN_ID_ENC_LAST   (CAN_ID_ENC_BASE + (ENCODER_TOTAL_NUM + 3) / 4 - 1) // 21 路编码器对应 0x100~0x105
#define CAN_ID_ERR_STATUS 0x1F0

// CAN 通信任务入口。
// parameter 应传入 TaskSharedData_t*，用于访问 canRxQueue/canTxQueue 等共享资源。
// 任务常驻运行，不返回。
void taskCanComm(void *parameter);

#endif
