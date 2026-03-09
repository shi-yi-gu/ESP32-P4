#include "SystemTask.h"
#include "ServoBusManager.h"
#include "AngleSolver.h"
#include "CalibrationTask.h"

#include <string.h>

volatile uint8_t g_calibrationUIStatus = 0;

TaskSharedData_t sharedData;

TaskHandle_t taskUpperCommHandle = NULL;
TaskHandle_t taskCanCommHandle = NULL;
TaskHandle_t taskSolverHandle = NULL;

ServoBusManager servoBus0;
ServoBusManager servoBus1;
ServoBusManager servoBus2;
ServoBusManager servoBus3;

AngleSolver angleSolver;

// Joint index (0~20) -> (busIndex, servoID)
JointMapItem jointMap[ENCODER_TOTAL_NUM] = {
    // Bus 0
    {0, 1}, {0, 2}, {0, 3}, {0, 4},
    // Bus 1
    {1, 1}, {1, 2}, {1, 3}, {1, 4},
    // Bus 2
    {2, 1}, {2, 2}, {2, 3}, {2, 4},
    // Bus 3
    {3, 1}, {3, 2}, {3, 3}, {3, 4}, {3, 5},
    // Remaining 4 joints
    {0, 5}, {1, 5}, {2, 5}, {3, 6}
};

void System_Init() {
    Serial.begin(921600);
    while (!Serial) {
        delay(10);
    }
    delay(1000);

    sharedData.cmdQueue = xQueueCreate(5, sizeof(ServoCommand_t));
    sharedData.statusQueue = xQueueCreate(3, sizeof(ServoStatus_t));
    sharedData.canRxQueue = xQueueCreate(1, sizeof(RemoteSensorData_t));
    sharedData.canTxQueue = xQueueCreate(5, sizeof(RemoteCommand_t));
    sharedData.servoAngleQueue = xQueueCreate(1, sizeof(ServoAngleData_t));
    sharedData.mappedAngleQueue = xQueueCreate(1, sizeof(MappedAngleData_t));
    sharedData.jointDebugQueue = xQueueCreate(1, sizeof(JointDebugData_t));

    if (!sharedData.cmdQueue || !sharedData.statusQueue ||
        !sharedData.canRxQueue || !sharedData.canTxQueue ||
        !sharedData.servoAngleQueue || !sharedData.mappedAngleQueue ||
        !sharedData.jointDebugQueue) {
        while (1) {}
    }

    sharedData.targetAnglesMutex = xSemaphoreCreateMutex();
    if (!sharedData.targetAnglesMutex) {
        while (1) {}
    }
    memset(sharedData.targetAngles, 0, sizeof(sharedData.targetAngles));

    // Keep all buses initialized to avoid changing existing wiring assumptions.
    servoBus0.begin(0, 21, 20, 1000000);
    servoBus1.begin(1, 23, 22, 1000000);
    servoBus2.begin(2, 27, 26, 1000000);
    servoBus3.begin(3, 33, 32, 1000000);

    int16_t zeros[ENCODER_TOTAL_NUM];
    float ratios[ENCODER_TOTAL_NUM];
    int8_t dirs[ENCODER_TOTAL_NUM];
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        zeros[i] = 2048;
        ratios[i] = 1.0f;
        dirs[i] = 1;
    }
    angleSolver.init(zeros, ratios, dirs);

    float pidConfigs[2][PID_PARAMETER_NUM] = {
        {15.0f, 0.2f, 0.0f, 0.0f, 100.0f, 300.0f},
        {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 30719.0f}
    };
    angleSolver.setPIDParams(pidConfigs);

    // Test mode: no auto calibration movement, use manual encoderMax for joints 0~3.
    initManualCalibrationForTest();

    xTaskCreate(
        taskUpperComm,
        "UpperComm",
        UPPER_COMM_TASK_STACK_SIZE,
        &sharedData,
        TASK_UPPER_COMM_PRIORITY,
        &taskUpperCommHandle
    );

    xTaskCreate(
        taskCanComm,
        "CanComm",
        CAN_COMM_TASK_STACK_SIZE,
        &sharedData,
        TASK_CAN_COMM_PRIORITY,
        &taskCanCommHandle
    );

    xTaskCreate(
        taskSolver,
        "Solver",
        SOLVER_TASK_STACK_SIZE,
        &sharedData,
        TASK_SOLVER_PRIORITY,
        &taskSolverHandle
    );

    Serial.println("FreeRTOS tasks created successfully.");
    Serial.println("System ready.");
}

void System_Loop() {
    vTaskDelay(pdMS_TO_TICKS(10));
}
