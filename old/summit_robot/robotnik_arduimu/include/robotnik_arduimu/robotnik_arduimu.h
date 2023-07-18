/** \file robotnik_arduimu_node.h
 * \author Robotnik Automation S.L.L.
 * \version 1.0
 * \date    2012
 *
 * \brief class for the Robotnik ArduImu (+ GPS board optional )
 *
 * (C) Robotnik Automation, SLL
 */

#include <geometry_msgs/Vector3Stamped.h>  // messages used to publish raw data - magnetometer
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>  // messages used to publish raw data - acc, gryo
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <stdint.h>
#include <string.h>
#include <tf/transform_broadcaster.h>

#include <vector>

#include "SerialDevice.h"

#define BUFFER_SIZE 1000

// ERROR FLAGS
#define ARDUIMU_ERROR_NONE 0
#define ARDUIMU_ERROR_OPENING 1
#define ARDUIMU_ERROR_SERIALCOMM 2
#define ARDUIMU_ERROR_TIMEOUT 3

//! Timeout for controlling the communication with the device (in seconds)
#define ARDUIMU_TIMEOUT_COMM 2.0

#define ARDUIMU_DEFAULT_PORT "/dev/ttyS0"
#define ARDUIMU_DEFAULT_PARITY "none"  //"even" "odd""none"
#define ARDUIMU_DEFAULT_TRANSFERRATE 115200  // 9600
#define ARDUIMU_DEFAULT_DATA_SIZE 8

#define ARDUIMU_PI 3.14159265358979323846

// Responses from device
#define ERROR_CRC 1
#define ARDUIMU_SERIAL_DELAY \
    10000  //! us between serial transmisions to the controller
#define ARDUIMU_MAX_ERRORS_INIT \
    1000  // Max number of consecutive communications errors in INIT state

namespace robotnik_arduimu_ns

{
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

//! Class to operate with the ms20 magnets sensor
class robotnik_arduimu_node
{
   public:
    //! Buffer for receiving data from the device
    char RecBuf[BUFFER_SIZE];

    //! Raw data imu
    int   temperature_;
    int   aX_, aY_, aZ_;  // Accelerations
    int   gX_, gY_, gZ_;  // Gyro speeds
    int   mX_, mY_, mZ_;  // Magnetometers
    float mH_;  // Magnetometer
    int   dgX_, dgY_, dgZ_;  // Calibration of gyros

    // Magnetometer calibration
    double x_offset_, y_offset_, z_offset_;
    double x_gain_, y_gain_, z_gain_;

    // Variables to manage a yaw heading value comming only from magnetometer
    double yaw_mag_;
    double yaw_mag_init_;

    //! Data GPS
    int latitude_, longitude_, altitude_;
    int cog_, sog_, fix_, sat_, tow_;

   private:
    //! Internal component state
    int iState_;
    //! Mutex for controlling the changes and access to raw data values
    pthread_mutex_t mutex_imu;
    //! Contains serial device structure for port
    SerialDevice* serial;
    //! Auxiliar variable for controling the timeout in the communication
    ros::Time tNext;
    //! Auxiliar variable for controling the timeout in the communication
    ros::Time tNow;
    //! Contains the last ocurred error
    int iErrorType;
    //! Contains the next programmed command
    int iCmd;
    //! Flag to mark that gyros have been calibrated
    bool bCalibrated_;
    //! Flag active when the offset has been calibrated successfully
    bool bOffsetCalibration;
    //! Time of the last ArduImu reply
    ros::Time tArduImuReply;
    //! Desired frequency of the loop
    double desired_freq_;

   public:
    //! Public constructor
    robotnik_arduimu_node(std::string port);
    //! Public destructor
    ~robotnik_arduimu_node();
    //! Switches the state of the node into the desired state
    void SwitchToState(States new_state);
    //! Component State Machine
    void StateMachine();
    //! Open serial port
    int Open();
    //! Close serial port
    int Close();
    //! Service Callback - Calibrate
    bool srvCallback_Calibrate(
        std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    //! Service Callback - Reset Magnetic Yaw
    bool srvCallback_MagYawReset(
        std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

   private:
    //! Actions in the initial state
    void InitState();
    //! Actions in Ready state
    void ReadyState();
    //! Actions in Failure State
    void FailureState();
    //! Read RS-232 messages from the controller
    int ReadControllerMsgs();
    //! Calculates the CRC for the message
    // unsigned char ComputeCRC(char *string, int size);
    //! Process received messages from the device
    int ProcessMsg(char* msg);
    //! Calculates accurate sampling period
    double GetSamplingPeriod();
};

}  // namespace robotnik_arduimu_ns
