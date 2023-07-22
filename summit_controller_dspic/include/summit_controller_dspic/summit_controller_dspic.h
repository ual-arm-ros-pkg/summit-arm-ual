/** \file summit_controller_dspic.h
 * \author Robotnik Automation S.L.L.
 * \version 2.0
 * \date    2012
 *
 * \brief class for the summit dspic controller
 *
 * (C) Robotnik Automation, SLL
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>  // cmd_vel
#include <nav_msgs/msg/odometry.hpp>  // odom
#include <rclcpp/rclcpp.hpp>
#include <robotnik_msgs/srv/get_mode.hpp>
#include <robotnik_msgs/srv/set_mode.hpp>
#include <robotnik_msgs/srv/set_odometry.hpp>
#include <vector>

#include "SerialDevice.h"

#define BUFFER_SIZE 1000

// ERROR FLAGS
#define DSPIC_ERROR_NONE 0
#define DSPIC_ERROR_OPENING 1
#define DSPIC_ERROR_SERIALCOMM 2
#define DSPIC_ERROR_TIMEOUT 3

//! Timeout for controlling the communication with the device (in seconds)
#define DSPIC_TIMEOUT_COMM 2.0

//#define DSPIC_DEFAULT_PORT 	                "/dev/ttyS0"
//#define DSPIC_DEFAULT_PORT                    "/dev/ttyS0"
#define DSPIC_DEFAULT_PARITY "none"  //"even" "odd""none"
#define DSPIC_DEFAULT_TRANSFERRATE 38400  // 9600
#define DSPIC_DEFAULT_DATA_SIZE 8

#define DSPIC_CMD_MODIFY_ODOMETRY 1
#define DSPIC_CMD_INIT_ODOMETRY 2
#define DSPIC_CMD_RESET_ODOMETRY 3
#define DSPIC_CMD_SET_D 4
#define DSPIC_CMD_SET_R 5
#define DSPIC_CMD_SET_MOTOR_REFERENCES 6
#define DSPIC_CMD_READ_ODOMETRY 7
#define DSPIC_CMD_RESET_CONTROLLER 8
#define DSPIC_CMD_READ_PARAMETERS 9

#define DSPIC_CMD_READ_ENCODER 12
#define DSPIC_CMD_READ_ENCODER_GYRO 13
#define DSPIC_CMD_CALIBRATE_OFFSET 14
#define DSPIC_CMD_SET_TRIMMER 15

#define DSPIC_CMD_DEFAULT 13

#define DSPIC_FWD_DIRECTION 0
#define DSPIC_BWD_DIRECTION 2
#define DSPIC_FWD_TRACTION 1
#define DSPIC_BWD_TRACTION 3

// Mechanical constants of the device controlled by DSPIC

#define DSPIC_PI 3.14159265358979323846
// nominal wheel diameter = 0.184 m
// empirical wheel diameter = 0.160 m (depends on wheel internal foam type)
// these are RPM of wheel, gearbox reduction calculated in controller.
//                 Reduction=1/(774.40/(motor gear=12)) =  (1/64.53)
//#define DSPIC_RPM2MPS = (DSPIC_DIAMETER_WHEEL * DSPIC_PI)/60.0
#define DSPIC_RPM2MPS 0.009212684  // 0.00902843
// Distance back axis to front axis car-type kinematics
#define DSPIC_D_WHEELS_M 0.372

// Default max speeds
#define MOTOR_DEF_MAX_SPEED 3.00  // m/s
#define MOTOR_DEF_MAX_ANGLE 0.5236  // rad DTOR(30)=0.5236
#define MOTOR_PERCENT_BWD 0.5  // % percentage speed backwards

// Commands
#define GET_ODOMETRY "G"
#define SET_ODOMETRY "S"
#define INIT_ODOMETRY "I"
#define SET_DWHEELS "d"
#define SET_RPM2MPS "r"
#define GET_PARAMETERS "P"
#define RESET_CONTROLLER "X,170"
#define GET_ENCODER "E"
#define GET_ENCODER_GYRO "Y"
#define CALIBRATE_OFFSET "O"
#define SET_MOTOR_REFERENCES "A"
#define SET_PID_GAINS "C"  // kp,ki,kd
#define CHANGE_TERMINAL_MODE "M"
#define SET_TRIMMER "T"

// Responses from device
#define ERROR_CRC 1
#define ERROR_CMD 2
#define ERROR_OVFL 3
#define ERROR_OVRUN 4
#define ERROR_FRAME 5
#define ERROR_OUTRANGE 6
#define ERROR_SPI 7
#define ERROR_CMDOV 8
#define ERROR_PARAMOV 9
#define OK_GET_ODOMETRY 10
#define OK_SET_ODOMETRY 11
#define OK_INIT_ODOMETRY 12
#define OK_SET_DWHEELS 13
#define OK_SET_RPM2MPS 14

#define OK_GET_PARAMETERS 16

#define OK_GET_ENCODER 19
#define OK_GET_ENCODER_GYRO 20
#define OK_CALIBRATE_OFFSET 21
#define OK_SET_TRIMMER 22
#define ERROR_GET_ODOMETRY -10
#define ERROR_SET_ODOMETRY -11
#define ERROR_INIT_ODOMETRY -12
#define ERROR_SET_DWHEELS -13
#define ERROR_SET_RPM2MPS -14

#define ERROR_GET_PARAMETERS -16

#define ERROR_GET_ENCODER -19
#define ERROR_GET_ENCODER_GYRO -20
#define ERROR_CALIBRATE_OFFSET -21
#define ERROR_SET_TRIMMER -22
#define BOARD_RESET 30

// S,x,y,th        SET_ODOMETRY
// I               INIT_ODOMETRY

#define DSPIC_SERIAL_DELAY \
    10000  //! us between serial transmisions to the DSPIC controller
#define DSPIC_MAX_ERRORS 3  // Max number of consecutive communications errors

#define SINGLE_ACKERMANN 1  //  just forward axes turn (standard car steering)
#define DUAL_ACKERMANN_INVERTED 2  //  forward and backward axes turn opposite
#define DUAL_ACKERMANN_GEARED 3  //  forward and backward axes turn simetric

#define SUMMIT_JOINTS 10

namespace summit_ctrl_dspic
{
//! struct which contains all the device data
typedef struct dsPic_data_
{
    //! Odometry
    double odometry_x;
    double odometry_y;
    double odometry_yaw;
    double vel_x;
    double vel_y;
    double vel_yaw;
    //! Distance between wheels
    double dDWheels;
    //! Parameter for calculating velocity
    double dRPMtoMPS;
    //! Counts of encoder
    int16_t iEncoder;
    //! Counts of encoder in previous period
    int16_t iEncoderAnt;
    //! Value of the gyroscope's orientation
    double dGyro;
    //! Value of the gyroscope's offset
    double dOffsetGyro;
    //! Reference turn angle
    double rad;
    //! Reference linear speed
    double mps;
    //! Enable / disable motors
    bool bMotorsEnabled;
    //! Battery voltage
    double dBatt;
    //! For each channel, trimming data
    int last_channel;
    int offset1[4];
    int center[4];
    int offset2[4];

} dsPic_data_t;

//! Defines standard states of a component
enum States
{
    INIT_STATE,
    STANDBY_STATE,
    READY_STATE,
    EMERGENCY_STATE,
    FAILURE_STATE,
    SHUTDOWN_STATE
};

//! Defines return values for methods and functions
enum ReturnValue
{
    OK    = 0,
    ERROR = -1,
};

// Source of the robot's angle
enum AngleMode
{
    GYRO,
    ODOMETRY
};

//! Class to operate with the ms20 magnets sensor
class summit_controller_dspic
{
   public:
    //! Buffer for receiving data from the device
    char RecBuf[BUFFER_SIZE];

    //! Contains the data structure associated with the DSPIC device
    dsPic_data_t dsPic_data;

   private:
    //! Internal component state
    int iState_;
    //! Mutex for controlling the changes and access to odomety values
    pthread_mutex_t mutex_odometry;
    //! Contains the data for the next programmed command
    dsPic_data_t dsPic_data_tmp;
    //! Contains serial device structure for port
    SerialDevice* serial;
    //! Status of DSPIC
    int iStatusDSPIC;
    //! Auxiliar variable for controling the timeout in the communication
    rclcpp::Time tNext;
    //! Auxiliar variable for controling the timeout in the communication
    rclcpp::Time tNow;
    //! Contains the last ocurred error
    int iErrorType;
    //! Contains the next programmed command
    int iCmd;
    //! Flag to decide if the odometry is calculated inside the component or
    //! read from the external dsPic device
    static const bool bLocalOdometry;
    //! Encoder counts per revolution
    static const double fCountsPerRev;
    //! Source for the robot's angle
    AngleMode amMode;
    //! Flag active when the offset has been calibrated successfully
    bool bOffsetCalibration;
    //! Time of the last dsPic reply
    rclcpp::Time tDsPicReply;
    //! Flag active when an offset calibration message has been sent to the
    //! dsPic
    bool bSentOffsetCalibration;
    //!	Flag to control the calculation of the offset
    bool bSuccessCalibration;
    // Control mode variable
    int active_kinematic_mode_;

   public:
    //! Public constructor
    summit_controller_dspic(std::string port);
    //! Public destructor
    ~summit_controller_dspic();
    //! Switches the state of the node into the desired state
    void SwitchToState(States new_state);
    //! Component State Machine
    void StateMachine();
    //! Open serial port
    int Open();
    //! Close serial port
    int Close();
    //! Gets the parameters for the odometry
    void GetParameters(double* d, double* rpm2rps);
    //! Gets the value of the encoders
    void GetEncoderValue(int16_t* enc);
    //! Gets the value of the encoder
    void GetEncoderGyroValue(int16_t* enc, double* gyro);
    //! Gets the power voltage data
    float GetVoltage();
    //! Returns the value of current references
    void GetMotorReferences(double* rad, double* mps);
    //! Set new odometry values
    void ModifyOdometry(double x, double y, double yaw);
    //! Sends init odometry msg
    void InitOdometry();
    //! Resets internal odometry
    void ResetOdometry();
    //! Sets the distance between wheels msg
    void SetDWheels(double distance);
    //! Sets the value for converting from RPM to MPS
    void SetRPMtoMPS(double value);
    //! Sends read odometry msg and reads the value
    void ReadOdometry();
    //! Sends Reset Controller message
    void ResetController();
    //! Sends read parameters msg
    void ReadParameters();
    //! Sends read encoder msg
    void ReadEncoder();
    //! Calculates new values for the offset
    void CalibrateOffsetGyro();
    //! Reads the value of the encoder and the gyroscope
    void ReadEncoderGyro();
    //! True if the gyroscope's offset has been calibrated
    bool IsGyroCalibrated();
    //! Returns the value of the gyro and its offset
    void GetGyroValues(double* angle, double* offset);
    //! Set desired values
    void SetMotorReferences(double rad, double mps);
    //! Set trimmer values
    void SetTrimmer(int channel, int offset1, int center, int offset2);
    //! Switches motors on/off
    void ToggleMotorPower(unsigned char val);
    //! Callback - command references
    void cmdVelCallback(
        const geometry_msgs::msg::Twist::ConstSharedPtr& cmd_vel);
    // void commandCallback(const
    // ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);

    //! Set odometry service function
    bool set_odometry(
        const robotnik_msgs::srv::SetOdometry_Request& req,
        robotnik_msgs::srv::SetOdometry_Response&      res);
    //! Set mode service function
    bool set_mode(
        const robotnik_msgs::srv::SetMode_Request& request,
        robotnik_msgs::srv::SetMode_Response&      response);

   private:
    //! Set new odometry values
    int SendModifyOdometry(double x, double y, double yaw);
    //! Sends init odometry msg
    int SendInitOdometry();
    //! Resets internal odometry
    int SendResetOdometry();
    //! Sets the distance between wheels msg
    int SendSetDWheels(double distance);
    //! Sets the value for converting from RPM to MPS
    int SendSetRPMtoMPS(double value);
    //! Sends read odometry msg and reads the value
    int SendReadOdometry();
    //! Sends Reset Controller message
    int SendResetController();
    //! Sends read parameters msg
    int SendReadParameters();
    //! Sends read encoder msg
    int SendReadEncoder();
    //! Sets the values for moving
    int SendSetMotorReferences(double rad, double mps);
    //! Set trimmer
    int SendSetTrimmer(int channel);
    //! Actions in the initial state
    void InitState();
    //! Actions in Ready state
    void ReadyState();
    //! Actions in Failure State
    void FailureState();
    //! Actions in standby state
    void StandbyState();
    //! Read RS-232 messages from the controller
    int ReadControllerMsgs();
    //! Calculates the CRC for the message
    unsigned char ComputeCRC(char* string, int size);
    //! Process received messages from the device
    int ProcessMsg(char* msg);
    //! Calculates accurate sampling period
    double GetSamplingPeriod();
    //! Do the calcs to update the odometry
    void UpdateOdometry();
    //! Sends the message for recalibration
    int SendCalibrateOffsetGyro();
    //! Sends read encoders and gyroscope msg
    int SendReadEncoderGyro();
};

}  // namespace summit_ctrl_dspic
