set(APP_SRCS
    "app_main.cpp"
    "../ServoBoardMain/SystemTask.cpp"
    "../ServoBoardMain/UpperCommTask.cpp"
    "../ServoBoardMain/CanCommTask.cpp"
    "../ServoBoardMain/AngleSolver.cpp"
    "../ServoBoardMain/CalibrationTask.cpp"
    "../ServoBoardMain/JointCalibrationProfile.cpp"
    "../ServoBoardMain/ServoBusManager.cpp"
    "../ServoBoardMain/pid.c"
    "../libraries/FTServo_Arduino/src/HLSCL.cpp"
    "../libraries/FTServo_Arduino/src/SCS.cpp"
    "../libraries/FTServo_Arduino/src/SCSCL.cpp"
    "../libraries/FTServo_Arduino/src/SCSerial.cpp"
    "../libraries/FTServo_Arduino/src/SMS_STS.cpp"
)

idf_component_register(
    SRCS ${APP_SRCS}
    INCLUDE_DIRS
        "."
        "../ServoBoardMain"
        "../libraries/FTServo_Arduino/src"
)
