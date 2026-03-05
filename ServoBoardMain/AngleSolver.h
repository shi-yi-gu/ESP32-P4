#ifndef ANGLE_SOLVER_H
#define ANGLE_SOLVER_H

#include <stdint.h>
#include <math.h>

// 关键点：pid.h 为 C 头文件，需通过 extern "C" 防止 C++ 名字改编。
#ifdef __cplusplus
extern "C" {
#endif
#include "pid.h"
#ifdef __cplusplus
}
#endif

#include "TaskSharedData.h"       // 任务共享数据定义（含 ENCODER_TOTAL_NUM）。
#include "ServoBusManager.h"      // 舵机总线管理器声明。

// 关节数量（当前手部共 21 轴）。
#define JOINT_COUNT 21
// STS3215 角度换算系数：4096 step = 360 deg。
#define STS_STEPS_PER_DEG 11.3777f

// 关节映射结构：将关节索引(0~20)映射到物理总线和舵机 ID。
struct JointMapItem {
    uint8_t busIndex;   // 总线编号 0~3
    uint8_t servoID;    // 舵机 ID
};

class AngleSolver {
public:
    AngleSolver();

    /**
     * @brief 初始化全部关节参数（按关节索引一一对应）。
     * @param zeroOffsets 21 个关节的零位脉冲偏置。
     * @param gearRatios 21 个关节的减速比。
     * @param directions 21 个关节的方向符号（1 或 -1）。
     */
    void init(int16_t* zeroOffsets, float* gearRatios, int8_t* directions);

    // pidParams[0] 为外环 PID，pidParams[1] 为内环 PID。
    // 每组包含 Kp, Ki, Kd, Deadband, LimitIntegral, LimitOutput。
    void setPIDParams(float pidParams[][PID_PARAMETER_NUM]);
    
    /**
     * @brief 批量执行 21 轴角度解算（双环 PID）。
     *
     * @param targetDegs 输入：目标角度（来自规划层/上位机）。
     * @param magActualDegs 输入：磁编反馈角度（来自 CAN）。
     * @param servoActualDegs 输入：舵机反馈角度（来自舵机总线）。
     * @param outServoPulses 输出：舵机目标控制量（脉冲）。
     * @return true 计算完成。
     */
    bool compute(float* targetDegs, float* magActualDegs, float* servoActualDegs, int16_t* outServoPulses);

    // 复位全部 PID 内部状态。
    void resetAll();

private:
    int16_t _zeroOffsets[JOINT_COUNT];
    float   _gearRatios[JOINT_COUNT];
    int8_t  _directions[JOINT_COUNT];

    // PID 实例数组: [关节ID 0~20][环ID 0=外环, 1=内环]。
    PID_Info_TypeDef _pids[JOINT_COUNT][2];

    bool _initialized;
};

// 全局实例（定义于 SystemTask.cpp）。
extern AngleSolver  angleSolver;
extern JointMapItem jointMap[ENCODER_TOTAL_NUM];

// Solver 任务入口声明。
void taskSolver(void* parameter);

#endif
