#include "SystemTask.h"
#include "ServoBusManager.h"  // 新增
#include "AngleSolver.h"      // 新增


// =============== 全局变量定义 ===============
volatile uint8_t g_calibrationUIStatus = 0;

// 共享数据实例
TaskSharedData_t sharedData;

// 任务句柄
TaskHandle_t taskUpperCommHandle = NULL;
TaskHandle_t taskCanCommHandle   = NULL;
TaskHandle_t taskSolverHandle    = NULL;  // 【新增】解算任务句柄

// 注意：原 ino 中的 ServoManager 相关变量已被注释，因未启用 ServoCtrlTask，
// 若后续需启用，请在该文件中取消注释并添加相应头文件。
/*
ServoManager servoManager0(BUS0_SERVO_NUM);
ServoManager servoManager1(BUS1_SERVO_NUM);
ServoManager* servoManagers[NUM_SERVO_BUSES] = {
    &servoManager0,
    &servoManager1
};
*/


// 【新增】4路舵机总线管理器实例
ServoBusManager servoBus0;
ServoBusManager servoBus1;
ServoBusManager servoBus2;
ServoBusManager servoBus3;

// 【新增】角度解算器实例
AngleSolver angleSolver;


// 【新增】关节映射表: 将 0-20 索引映射到 (BusIndex, ServoID)
// 请根据实际硬件连线修改
JointMapItem jointMap[ENCODER_TOTAL_NUM] = {
    // Bus 0 (4个关节)
    {0, 1}, {0, 2}, {0, 3}, {0, 4},
    // Bus 1 (4个关节)
    {1, 1}, {1, 2}, {1, 3}, {1, 4},
    // Bus 2 (4个关节)
    {2, 1}, {2, 2}, {2, 3}, {2, 4},
    // Bus 3 (5个关节)
    {3, 1}, {3, 2}, {3, 3}, {3, 4}, {3, 5},
    // 剩余4个关节（根据实际分配）
    {0, 5}, {1, 5}, {2, 5}, {3, 6}  // 示例分配，请根据实际调整
};


// =============== 系统初始化函数 ===============
void System_Init() {
    // 初始化串口
    Serial.begin(921600);
    while (!Serial) {
        delay(10);
    }
    delay(1000);


    
    // 1. 创建队列 (资源分配)
    sharedData.cmdQueue    = xQueueCreate(5, sizeof(ServoCommand_t));
    sharedData.statusQueue = xQueueCreate(3, sizeof(ServoStatus_t));
    sharedData.canRxQueue  = xQueueCreate(1, sizeof(RemoteSensorData_t));
    sharedData.canTxQueue  = xQueueCreate(5, sizeof(RemoteCommand_t));
    sharedData.servoAngleQueue = xQueueCreate(1, sizeof(ServoAngleData_t));
    sharedData.mappedAngleQueue = xQueueCreate(1, sizeof(MappedAngleData_t));

    
    if (!sharedData.cmdQueue || !sharedData.statusQueue ||
        !sharedData.canRxQueue || !sharedData.canTxQueue ||
        !sharedData.servoAngleQueue || !sharedData.mappedAngleQueue) {
        while (1);
    }

    // 【新增】创建目标角度互斥锁
    sharedData.targetAnglesMutex = xSemaphoreCreateMutex();
    if (!sharedData.targetAnglesMutex) {
        while (1);
    }

    // 【新增】初始化目标角度为 0（防止上电飞车）
    memset(sharedData.targetAngles, 0, sizeof(sharedData.targetAngles));


servoBus0.begin(0, 21, 20, 1000000);  // 总线0: RX=16, TX=17, 波特率1Mbps
servoBus1.begin(1, 23, 22, 1000000);  // 总线1: RX=18, TX=19
servoBus2.begin(2, 27, 26, 1000000);  // 总线2: RX=20, TX=21（根据实际修改）
servoBus3.begin(3, 33, 32, 1000000);  // 总线3: RX=22, TX=23（根据实际修改）

    // 【新增】初始化 AngleSolver
    int16_t zeros[ENCODER_TOTAL_NUM];
    float   ratios[ENCODER_TOTAL_NUM];
    int8_t  dirs[ENCODER_TOTAL_NUM];

    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        zeros[i]  = 2048;   // 默认中位
        ratios[i] = 1.0f;
        dirs[i]   = 1;
    }
    angleSolver.init(zeros, ratios, dirs);

    // 【新增】设置 PID 参数 (双环)
    float pidConfigs[2][PID_PARAMETER_NUM] = {
        {20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3000.0f},   // 外环(位置): Mag -> Correction
        { 5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 30719.0f}    // 内环(舵机): Correction -> Servo
    };
    angleSolver.setPIDParams(pidConfigs);


    // 创建上位机通信任务
    xTaskCreate(
        taskUpperComm,
        "UpperComm",
        UPPER_COMM_TASK_STACK_SIZE,
        &sharedData,
        TASK_UPPER_COMM_PRIORITY,
        &taskUpperCommHandle
    );

    // 4.3 【新增】CAN 通信任务
    xTaskCreate(
        taskCanComm,
        "CanComm",
        CAN_COMM_TASK_STACK_SIZE,
        &sharedData,
        TASK_CAN_COMM_PRIORITY,
        &taskCanCommHandle
    );

    // 【新增】创建解算任务
    xTaskCreate(
        taskSolver, 
        "Solver",
         SOLVER_TASK_STACK_SIZE, 
         &sharedData,
        TASK_SOLVER_PRIORITY, 
        &taskSolverHandle);

    Serial.println("✅ FreeRTOS tasks created successfully.");
    Serial.println("System ready. Commands: s(停止), r(恢复)");
}

// =============== 主循环函数 ===============
void System_Loop() {
    static uint32_t lastPrintTime = 0;

    // 每 200ms 打印一次（可选，建议注释以提升实时性）
    // if (millis() - lastPrintTime > 200) {
    //     lastPrintTime = millis();
    //     printCanMonitor(); // 若需此函数，请将其移到 UpperCommTask 或单独文件
    // }

    // 必须保留延时，防止触发看门狗！
    vTaskDelay(pdMS_TO_TICKS(10));
}
