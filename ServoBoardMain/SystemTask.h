#ifndef SYSTEM_TASK_H
#define SYSTEM_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>      // 【新增】

// 应用层头文件
#include "TaskSharedData.h"
#include "UpperCommTask.h"
#include "CanCommTask.h"
#include "AngleSolver.h"          // 【新增】引入 AngleSolver（含 taskSolver 声明）
#include "ServoBusManager.h"      // 【新增】引入舵机总线管理器

// 关节数量定义
// #define ENCODER_TOTAL_NUM 21 // 已在 TaskSharedData.h 中定义

// 全局状态标志位（跨任务共享）
extern volatile uint8_t g_calibrationUIStatus;

// 系统初始化函数（替代 setup() 中的逻辑）
void System_Init();

// 主循环函数（替代 loop()）
void System_Loop();

#endif // SYSTEM_TASK_H
