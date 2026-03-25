#ifndef CAN_COMM_TASK_H
#define CAN_COMM_TASK_H

#include <Arduino.h>

#include "TaskSharedData.h"

// Encoder payload frames: 4 channels per frame.
#define CAN_ID_ENC_BASE 0x100
#define CAN_ID_ENC_LAST (CAN_ID_ENC_BASE + (ENCODER_TOTAL_NUM + 3) / 4 - 1)

// Error-detail protocol (new): 3 frames, 7 channels per frame.
#define CAN_ID_ERROR_DETAIL_BASE 0x1F0
#define CAN_ID_ERROR_DETAIL_LAST 0x1F2
#define CAN_ERROR_DETAIL_FRAME_COUNT 3
#define CAN_ERROR_DETAIL_CODES_PER_FRAME 7
#define CAN_ERROR_DETAIL_DLC 8
#define CAN_ERROR_REASSEMBLY_TIMEOUT_MS 100
#define CAN_ERROR_TABLE_CLEAR_TIMEOUT_MS 300

void taskCanComm(void* parameter);

#endif
