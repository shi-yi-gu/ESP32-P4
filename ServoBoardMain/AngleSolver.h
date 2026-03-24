#ifndef ANGLE_SOLVER_H
#define ANGLE_SOLVER_H

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "pid.h"
#ifdef __cplusplus
}
#endif

#include "ServoBusManager.h"
#include "TaskSharedData.h"

#define JOINT_COUNT ENCODER_TOTAL_NUM
#define STS_STEPS_PER_DEG 11.3777f

struct JointMapItem {
    uint8_t busIndex;
    uint8_t servoID;
};

struct MotorMapItem {
    uint8_t busIndex;
    uint8_t servoID;
};

class AngleSolver {
public:
    AngleSolver();

    void init(int16_t* zeroOffsets, float* gearRatios, int8_t* directions);
    void setPIDParams(float pidParams[][PID_PARAMETER_NUM]);

    bool compute(float* targetDegs,
                 float* magActualDegs,
                 const int32_t* absolutePosition,
                 int16_t* outServoPulses);

    float getPidOutput(uint8_t jointIndex, uint8_t loopIndex) const;
    void resetAll();

private:
    int16_t _zeroOffsets[JOINT_COUNT];
    float _gearRatios[JOINT_COUNT];
    int8_t _directions[JOINT_COUNT];
    PID_Info_TypeDef _pids[JOINT_COUNT][2];
    bool _initialized;
};

extern AngleSolver angleSolver;

// Primary mapping: joint index (0..20) -> primary servo.
extern JointMapItem jointMap[ENCODER_TOTAL_NUM];
// Secondary mapping for joint16 dual-servo coupling.
extern MotorMapItem joint16SecondaryMotor;
// Motor channel mapping: 0..21 -> servo bus/id.
extern MotorMapItem motorMap[SERVO_TOTAL_NUM];

void taskSolver(void* parameter);

#endif
