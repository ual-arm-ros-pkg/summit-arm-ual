cmake_minimum_required(VERSION 3.8)
project(robotnik_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    # Messages:
    msg/Encoders.msg
    msg/InputsOutputs.msg
    msg/PTZ.msg
    msg/Data.msg
    msg/Interfaces.msg
    msg/Axis.msg
    msg/AlarmSensor.msg
    msg/Alarms.msg
    msg/MotorStatus.msg
    msg/MotorsStatus.msg
    msg/State.msg
    msg/ArmStatus.msg
    msg/BatteryStatus.msg
    msg/BatteryStatusStamped.msg
    msg/OdomCalibrationStatus.msg
    msg/OdomCalibrationStatusStamped.msg
    msg/OdomManualCalibrationStatus.msg
    msg/OdomManualCalibrationStatusStamped.msg
    msg/MotorHeadingOffset.msg
    msg/MotorsStatusDifferential.msg
    msg/InverterStatus.msg
    msg/RobotnikMotorsStatus.msg
    msg/ElevatorAction.msg
    msg/ElevatorStatus.msg
    msg/PantiltStatus.msg
    msg/PantiltStatusStamped.msg
    msg/CartesianEulerPose.msg
    msg/AlarmMonitor.msg
    msg/AlarmsMonitor.msg
    msg/NamedInputsOutputs.msg
    msg/NamedInputOutput.msg
    msg/Register.msg
    msg/Registers.msg
    msg/StringArray.msg
    msg/LaserMode.msg
    msg/LaserStatus.msg
    msg/Logger.msg
    msg/SafetyModuleStatus.msg
    msg/StringStamped.msg
    msg/SubState.msg
    msg/QueryAlarm.msg
    msg/BatteryDockingStatus.msg
    msg/BatteryDockingStatusStamped.msg
    msg/BoolArray.msg
    msg/MotorPID.msg
    msg/MotorCurrent.msg
    msg/Pose2DArray.msg
    msg/Pose2DStamped.msg
    msg/ReturnMessage.msg
    msg/PresenceSensor.msg
    msg/PresenceSensorArray.msg
    msg/WatchdogStatus.msg
    msg/WatchdogStatusArray.msg
    msg/MotorReferenceValue.msg
    msg/MotorReferenceValueArray.msg
    msg/RecordStatus.msg
    # Services:
    srv/AckAlarm.srv
    srv/AxisRecord.srv
    srv/EnableDisable.srv
    srv/GetAlarms.srv
    srv/GetBool.srv
    srv/GetDigitalInput.srv
    srv/GetModbusRegister.srv
    srv/GetMode.srv
    srv/GetMotorsHeadingOffset.srv
    srv/GetPOI.srv
    srv/GetPTZ.srv
    srv/Home.srv
    srv/InsertTask.srv
    srv/LoggerQuery.srv
    srv/QueryAlarms.srv
    srv/Record.srv
    srv/ResetFromSubState.srv
    srv/SetAnalogOutput.srv
    srv/SetBuzzer.srv
    srv/SetByte.srv
    srv/SetCartesianEulerPose.srv
    srv/SetCurrent.srv
    srv/SetDigitalOutput.srv
    srv/SetElevator.srv
    srv/SetEncoderTurns.srv
    srv/SetFloatValue.srv
    srv/SetHeight.srv
    srv/SetInt16.srv
    srv/SetLaserMode.srv
    srv/SetModbusRegister.srv
    srv/SetModbusRegisterBit.srv
    srv/SetMode.srv
    srv/SetMotorMode.srv
    srv/SetMotorPID.srv
    srv/SetMotorStatus.srv
    srv/SetNamedDigitalOutput.srv
    srv/SetOdometry.srv
    srv/SetPTZ.srv
    srv/SetString.srv
    srv/SetTransform.srv
#    ADD_LINTER_TESTS
    # actions
    action/SetElevator.action
    # deps:
    DEPENDENCIES
        std_msgs
        geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)


if(BUILD_TESTING)
find_package(ament_cmake_xmllint REQUIRED)
ament_xmllint()
endif()

ament_package()
