/** \file robotnik_arduimu_node.cc
 * \author Robotnik Automation S.L.L.
 * \version 2.0
 * \date    2012
 *
 * \brief robotnik_arduimu_node class driver
 * Component to read raw data from ArduIMU
 * (C) 2013 Robotnik Automation, SLL
 */

#include "robotnik_arduimu/robotnik_arduimu.h"

#include <math.h>

#include <cstdlib>

using namespace robotnik_arduimu_ns;
std::string            port;
robotnik_arduimu_node* rbk_arduimu;

/*!	\fn robotnik_arduimu_node::robotnik_arduimu_node()
 * 	\brief Public constructor
 */
robotnik_arduimu_node::robotnik_arduimu_node(std::string port)
    : desired_freq_(125)
{
    // frame IMU
    // T:-944,AX:-5,AY:-12,AZ:-31,GX:2,GY:-1,GZ:0,MGX:-203,MGY:-307,MGZ:-348,MGH:117.47
    // frame GPS   LAT:0,LON:0,ALT:0,COG:0,SOG:0,FIX:0,SAT:0,TOW:0,

    // Initial values
    temperature_ = 0;
    aX_          = 0;
    aY_          = 0;
    aZ_          = 0;
    gX_          = 0;
    gY_          = 0;
    gZ_          = 0;
    mX_          = 0;
    mY_          = 0;
    mZ_          = 0;
    mH_          = 0.0;
    dgX_         = 0;
    dgY_         = 0;
    dgZ_         = 0;
    bCalibrated_ = false;

    yaw_mag_      = 0.0;
    yaw_mag_init_ = 0.0;

    // Create serial port
    serial = new SerialDevice(
        port.c_str(), ARDUIMU_DEFAULT_TRANSFERRATE, ARDUIMU_DEFAULT_PARITY,
        ARDUIMU_DEFAULT_DATA_SIZE);
};

/*!	\fn robotnik_arduimu_node::~robotnik_arduimu_node()
 * 	\brief Public destructor
 */
robotnik_arduimu_node::~robotnik_arduimu_node()
{
    // Close serial port
    if (serial != NULL) serial->ClosePort();

    // Delete serial port
    if (serial != NULL) delete serial;
}

/*!	\fn void robotnik_arduimu_node::SwitchToState(States new_state)
 * 	function that switches the state of the component into the desired state
 * 	\param new_state as an integer, the new state of the component
 */
void robotnik_arduimu_node::SwitchToState(States new_state)
{
    if (new_state == iState_) return;

    iState_ = new_state;
    switch (new_state)
    {
        case INIT_STATE:
            // ROS_INFO("robotnik_arduimu_node:: switching to INIT state");
            break;
        case STANDBY_STATE:
            // ROS_INFO("robotnik_arduimu_node:: switching to STANDBY state");
            break;
        case READY_STATE:
            // ROS_INFO("robotnik_arduimu_node:: switching to READY state");
            break;
        case EMERGENCY_STATE:
            // ROS_INFO("robotnik_arduimu_node:: switching to EMERGENCY state");
            break;
        case FAILURE_STATE:
            // ROS_INFO("robotnik_arduimu_node:: switching to FAILURE state");
            break;
        case SHUTDOWN_STATE:
            // ROS_INFO("robotnik_arduimu_node:: switching to SHUTDOWN state");
            break;
        default:
            ROS_ERROR("robotnik_arduimu_node:: Switching to UNKNOWN state");
            break;
    }
}

/*!	\fn void robotnik_arduimu_node::StateMachine()
 * 	function that manages internal states of the component
 */
void robotnik_arduimu_node::StateMachine()
{
    switch (iState_)
    {
        case INIT_STATE:
            InitState();
            break;
        case READY_STATE:
            ReadyState();
            break;
        case FAILURE_STATE:
            FailureState();
            break;
        default:
            break;
    }
}

/*!	\fn int robotnik_arduimu_node::Open(char *dev)
 * 	\brief Open serial port
 * 	\returns -1 Error
 * 	\returns 0 Ok
 */
int robotnik_arduimu_node::Open()
{
    // Setup serial device
    if (this->serial->OpenPort2() == SERIAL_ERROR)
    {
        ROS_ERROR("robotnik_arduimu_node::Open: Error Opening Serial Port");
        iErrorType = ARDUIMU_ERROR_OPENING;
        SwitchToState(FAILURE_STATE);
        return ERROR;
    }
    ROS_INFO(
        "robotnik_arduimu_node::Open: serial port opened at %s",
        serial->GetDevice());

    usleep(50000);
    ROS_INFO("Starting gyro calibration");
    SwitchToState(INIT_STATE);

    return OK;
}

/*!	\fn int robotnik_arduimu_node::Close()
 * 	\brief Closes serial port
 * 	\returns ERROR
 * 	\returns OK
 */
int robotnik_arduimu_node::Close()
{
    if (serial != NULL) serial->ClosePort();
    return OK;
}

/*!	\fn void robotnik_arduimu_node::InitState()
 * 	\brief Actions in the initial state
 * 	First call-> Sends start data logging message -> IF ERROR go to
 *FAILURE_STATE
 *	-> IF OK inits communication's timers
 * 	Next call -> Reads messages from the sensor and compares timers	-> IF no
 *responses go to FAILURE_STATE
 * 	-> IF OK go to READY_STATE
 */
void robotnik_arduimu_node::InitState()
{
    static int  count_errors_reading = 0;
    int         ret                  = 0;
    static int  iCyclesCalibration   = 0;
    static int  iCyclesStabilization = 0;
    static bool bCalibrating         = false;

    ret = ReadControllerMsgs();
    switch (ret)
    {
        case ERROR:  // Nothing received
            // ROS_ERROR("robotnik_arduimu_node::InitState: received ERROR");
            count_errors_reading++;
            break;
        case OK:
            // Inits ArduIMU reply timer
            tArduImuReply = ros::Time::now();

            // Allow future use of the InitState for calibration
            if ((!bCalibrated_) && (!bCalibrating))
            {
                iCyclesStabilization = 0;
                iCyclesCalibration   = 0;
                dgX_                 = 0;
                dgY_                 = 0;
                dgZ_                 = 0;
                bCalibrating         = true;
            }
            // Calibrating
            if (bCalibrating == true)
            {
                iCyclesStabilization++;
                if (iCyclesStabilization > 20)
                {
                    if (!bCalibrated_)
                    {
                        iCyclesCalibration++;  // Calibration of gyros
                        dgX_ += gX_;
                        dgY_ += gY_;
                        dgZ_ += gZ_;
                    }
                    if (iCyclesCalibration == 20)
                    {
                        dgX_ /= 20;
                        dgY_ /= 20;
                        dgZ_ /= 20;
                        bCalibrated_ = true;
                        bCalibrating = false;
                        ROS_INFO(
                            "Calibration done - gyro calibration daX:%d daY:%d "
                            "daZ:%d",
                            dgX_, dgY_, dgZ_);
                        SwitchToState(READY_STATE);
                    }
                }
            }
            break;
    }

    if (count_errors_reading >= ARDUIMU_MAX_ERRORS_INIT)
    {
        iErrorType = ARDUIMU_ERROR_SERIALCOMM;
        ROS_ERROR(
            "robotnik_arduimu_node::InitState: %d errors reading",
            count_errors_reading);
        SwitchToState(FAILURE_STATE);
        count_errors_reading = 0;
    }
}

/*! \fn void robotnik_arduimu_node::ReadyState()
 *  \brief Actions in Ready state
 *    First time -> Gets current time
 *    Next times -> Read from controller and checks the timeout between
 * responses
 *    -> IF TIMEOUT go to FAILURE_STATE
 *    -> IF OK resets communication's timer
 */
void robotnik_arduimu_node::ReadyState()
{
    static int count_errors_reading = 0;
    int        ret                  = 0;

    // Actions in ReadyState
    ret = ReadControllerMsgs();
    switch (ret)
    {
        case ERROR:  // Nothing received
            count_errors_reading++;
            break;
        default:
            // If no errors, reset counter
            count_errors_reading = 0;
            // Saves current time
            // Inits nodeguard reply timer
            tArduImuReply = ros::Time::now();
            // ROS_INFO("robotnik_arduimu_node::ReadyState: Ret = %d", ret);
            break;
    }

    // Checks the communication status
    ros::Time current_time = ros::Time::now();
    if ((current_time - tArduImuReply).toSec() > ARDUIMU_TIMEOUT_COMM)
    {
        ROS_ERROR(
            "robotnik_arduimu_node::ReadyState: Timeout in communication with "
            "the device");
        tArduImuReply = ros::Time::now();  // Resets the timer
    }
}

/*!	\fn void robotnik_arduimu_node::FailureState()
 * 	\brief Actions in Failure State
 */
void robotnik_arduimu_node::FailureState()
{
    int        timer           = 2500000;  // useconds
    static int recovery_cycles = 0;

    recovery_cycles++;

    if (recovery_cycles >= 100)
    {  // Try to recover every 'second'
        ROS_INFO("robotnik_arduimu_node::FailureState: Trying to recover..");
        switch (iErrorType)
        {
            case ARDUIMU_ERROR_OPENING:  // Try to recover
                ROS_ERROR(
                    "robotnik_arduimu_node::FailureState: Recovering from "
                    "failure state (ARDUIMU_ERROR_OPENING.)");
                this->Close();
                usleep(timer);
                this->Open();
                SwitchToState(INIT_STATE);
                break;
            case ARDUIMU_ERROR_TIMEOUT:
                ROS_ERROR(
                    "robotnik_arduimu_node::FailureState: Recovering from "
                    "failure state (ARDUIMU_ERROR_TIMEOUT.)");
                this->Close();
                usleep(timer);
                this->Open();
                SwitchToState(INIT_STATE);
                break;
            case ARDUIMU_ERROR_SERIALCOMM:
                ROS_ERROR(
                    "robotnik_arduimu_node::FailureState: Recovering from "
                    "failure state (ARDUIMU_ERROR_SERIALCOMM.)");
                this->Close();
                usleep(timer);
                this->Open();
                SwitchToState(INIT_STATE);
                break;
        }
        recovery_cycles = 0;
    }
}

/*!	\fn int robotnik_arduimu_node::ReadControllerMsgs()
 * 	Read RS-232 messages from the controller
 * 	Returns OK if messages readed
 * 	Return ERROR otherwise
 */
int robotnik_arduimu_node::ReadControllerMsgs()
{
    int read_bytes = 0;  // Number of received bytes
    int ret        = ERROR;
    // char cReadBuffer[100] = "\0";
    char cReadBuffer[200] = "\0";

    // Read controller messages
    if (serial->ReadPort(cReadBuffer, &read_bytes, 200) == ERROR)
    {
        ROS_ERROR(
            "robotnik_arduimu_node::ReadControllerMsgs: Error reading port");
        return ERROR;
    }
    // ROS_INFO("ReadControllerMsgs:  %s", cReadBuffer);
    while (read_bytes > 0)
    {
        // ROS_INFO("ReadControllerMsgs:  %s", cReadBuffer);
        ret = ProcessMsg(cReadBuffer);  // returns ERROR or Number of bytes !
        if (ret == ERROR)
            return ERROR;
        else
            ret = OK;
        if (serial->ReadPort(cReadBuffer, &read_bytes, 100) == ERROR)
        {
            // ROS_ERROR("robotnik_arduimu_node::ReadControllerMsgs: Error after
            // 1st port read"); return ERROR;
            ret = OK;
        }
    }

    return ret;
}

/*!	\fn unsigned char robotnik_arduimu_node::ComputeCRC(char *string, int size)
 * Calculates the CRC for the message
 * \return OK
 * \return ERROR
 */
/*
unsigned char robotnik_arduimu_node::ComputeCRC(char *string, int size){

    int j=0;
    unsigned char crc=0;

    while ( (string[j] != 0) && ( j< size ) ) {
        crc +=string[j];
        //ROS_INFO(" char[%d] -> %c = %d, crc = %d", j, string[j], string[j],
(int)crc); j++;
     }
    return crc;
}
*/

/*!	\fn int robotnik_arduimu_node::ProcessMsg(char *msg)
 * Process received messages from the device
 * \return ERROR if nothing for processing or incorrect token num
 * \return OK otherwise
 */
int robotnik_arduimu_node::ProcessMsg(char* msg)
{
    char* ptr;
    // char cReceivedTokens[20][16];
    char cReceivedTokens[15][16];
    char cReceivedSubTokens[2][10];
    int  i = 0, j = 0;
    int  MAX_TOKENS = 15;
    // int MAX_TOKENS = 25;
    // int TOKENS = 19;
    int TOKENS_IMU = 11;
    // int TOKENS_GPS = 8;

    // T:-964,AX:1739,AY:-62,AZ:7698,GX:4312,GY:-3573,GZ:-1459,MX:-307,MY:-2,MZ:319,MH:173.63,LT:0,LN:0,AT:0,CG:0,SG:0,FX:0,ST:0,TW:0,

    ptr = strtok(msg, ",");

    // Extract all strings separated by comas
    // All tokens sent by device MUST have length < 16 or
    // buffer ovfl will be generated
    while ((ptr != NULL) && (i < (MAX_TOKENS - 1)))
    {
        sprintf(cReceivedTokens[i], "%s", ptr);
        ptr = strtok(NULL, ",");
        i++;
    }
    i--;  // frame has a trailing ','

    if (i == 0)
    {
        ROS_ERROR("robotnik_arduimu_node::ProcessMsg: nothing for processing");
        return ERROR;
    }
    // if ((i!=TOKENS_IMU)&&(i!=TOKENS_GPS)) {
    if (i != TOKENS_IMU)
    {
        ROS_ERROR(
            "robotnik_arduimu_node::ProcessMsg: received unexpected number of "
            "tokens %d",
            i);
        return ERROR;
    }

    // Preprocessing of tokens
    for (j = 0; j < i; j++)
    {
        ptr = strtok(cReceivedTokens[j], ":");
        sprintf(cReceivedSubTokens[0], "%s", ptr);
        ptr = strtok(NULL, ":");
        sprintf(cReceivedSubTokens[1], "%s", ptr);
        // ROS_INFO("st1 %s   st2 %s", cReceivedSubTokens[0],
        // cReceivedSubTokens[1]); Processing of IMU tokens frame IMU
        // T:-944,AX:-5,AY:-12,AZ:-31,GX:2,GY:-1,GZ:0,MGX:-203,MGY:-307,MGZ:-348,MGH:117.47
        if (i == TOKENS_IMU)
        {
            if (!strcmp(cReceivedSubTokens[0], "T"))
                temperature_ = atoi(cReceivedSubTokens[1]);
            if (!strcmp(cReceivedSubTokens[0], "X"))
                aX_ = atoi(cReceivedSubTokens[1]);
            if (!strcmp(cReceivedSubTokens[0], "Y"))
                aY_ = atoi(cReceivedSubTokens[1]);
            if (!strcmp(cReceivedSubTokens[0], "Z"))
                aZ_ = atoi(cReceivedSubTokens[1]);
            if (!strcmp(cReceivedSubTokens[0], "A"))
                gX_ = atoi(cReceivedSubTokens[1]);
            if (!strcmp(cReceivedSubTokens[0], "B"))
                gY_ = atoi(cReceivedSubTokens[1]);
            if (!strcmp(cReceivedSubTokens[0], "C"))
                gZ_ = atoi(cReceivedSubTokens[1]);
            if (!strcmp(cReceivedSubTokens[0], "R"))
                mX_ = atoi(cReceivedSubTokens[1]);
            if (!strcmp(cReceivedSubTokens[0], "S"))
                mY_ = atoi(cReceivedSubTokens[1]);
            if (!strcmp(cReceivedSubTokens[0], "T"))
                mZ_ = atoi(cReceivedSubTokens[1]);
            // if(!strcmp(cReceivedSubTokens[0], "MH")) mH_ =
            // atof(cReceivedSubTokens[1]); // float TT contains measured time
            // in ms between succesive sends (about 5 - 6 ms)
            if (!strcmp(cReceivedSubTokens[0], "M"))
                mH_ = atoi(cReceivedSubTokens[1]);
        }
    }

    /*
    if (i==TOKENS_IMU) {
        if(!strcmp(cReceivedSubTokens[0], "T")) temperature_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "AX")) aX_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "AY")) aY_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "AZ")) aZ_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "GX")) gX_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "GY")) gY_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "GZ")) gZ_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "MX")) mX_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "MY")) mY_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "MZ")) mZ_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "MH")) mH_ =
    atof(cReceivedSubTokens[1]); // float
        }
    // Processing of GPS tokens
        // frame GPS   LAT:0,LON:0,ALT:0,COG:0,SOG:0,FIX:0,SAT:0,TOW:0,
    if (i==TOKENS_GPS) {
        if(!strcmp(cReceivedSubTokens[0], "LT")) latitude_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "LN"))
    longitude_ = atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0],
    "AT")) altitude_ = atoi(cReceivedSubTokens[1]);
        if(!strcmp(cReceivedSubTokens[0], "CG")) cog_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "SG")) sog_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "FX")) fix_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "ST")) sat_ =
    atoi(cReceivedSubTokens[1]); if(!strcmp(cReceivedSubTokens[0], "TW")) tow_ =
    atoi(cReceivedSubTokens[1]);
        }
            */

    return OK;
}

////
// CALLBACKS
// Service Calibrate
bool robotnik_arduimu_node::srvCallback_Calibrate(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    bCalibrated_ = false;
    SwitchToState(INIT_STATE);
    return true;
}

// Service Magnetic Yaw Reset
bool robotnik_arduimu_node::srvCallback_MagYawReset(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    // Set init to last value
    yaw_mag_init_ = yaw_mag_;
    yaw_mag_      = 0.0;
    return true;
}

////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS

/*!     \fn double robotnik_arduimu_node::GetSamplingPeriod()
 *      \brief Returns accurate measured sampling period
 *      \return sample period in seconds
 */
double robotnik_arduimu_node::GetSamplingPeriod()
{
    static bool      first = true;
    static ros::Time now, last;
    double           secs = 0.0;

    now = ros::Time::now();

    if (first)
    {
        first = false;
        last  = now;
        return -1.0;
    }

    secs = (now - last).toSec();
    last = now;
    return secs;
}

// MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robotnik_arduimu_node_node");
    // ROS_INFO("robotnik_arduimu_node for ROS %.2f", NODE_VERSION);
    // int time_remaining = -1;
    float mean_filter[25];
    float mean = 0.0;
    int   i;
    for (i = 0; i < 25; i++) mean_filter[i] = 0.0;

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    pn.param<std::string>("port", port, "/dev/ttyUSB_IMU");

    std::string base_frame_id;
    std::string imu_frame_id;
    pn.param<std::string>("base_frame_id", base_frame_id, "base_link");
    pn.param<std::string>("imu_frame_id", imu_frame_id, "imu");

    // Create node object
    rbk_arduimu = new robotnik_arduimu_node(port);

    double x_offset = 0.0, y_offset = 0.0, z_offset = 0.0;
    double x_gain = 1.0, y_gain = 1.0, z_gain = 1.0;
    pn.getParam("x_offset", x_offset);
    pn.getParam("y_offset", y_offset);
    pn.getParam("z_offset", z_offset);
    pn.getParam("x_gain", x_gain);
    pn.getParam("y_gain", y_gain);
    pn.getParam("z_gain", z_gain);

    // Services
    ros::ServiceServer srv_Calibrate;

    // Advertise services
    srv_Calibrate = n.advertiseService(
        "imu/Calibrate", &robotnik_arduimu_node::srvCallback_Calibrate,
        rbk_arduimu);

    // Services
    ros::ServiceServer srv_MYReset;

    // Advertise services
    srv_Calibrate = n.advertiseService(
        "imu/MYReset", &robotnik_arduimu_node::srvCallback_MagYawReset,
        rbk_arduimu);

    // Publishing
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 50);
    ros::Publisher mag_pub =
        n.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 50);
    ros::Publisher yaw_mag_pub =
        n.advertise<std_msgs::Float32>("imu/yaw_mag", 50);

    tf::TransformBroadcaster tf_broadcaster;

    if (rbk_arduimu->Open() == OK)
        ROS_INFO("Connected to robotnik_arduimu_node component");
    else
    {
        ROS_FATAL("Could not connect to robotnik_arduimu_node component");
        //	ROS_BREAK();
    }

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time    = ros::Time::now();

    ros::Rate r(125.0);  // same freq for read and publish
    while (n.ok())
    {
        current_time = ros::Time::now();

        // 1 State Machine
        rbk_arduimu->StateMachine();

        // 2 Publish raw data
        //! Message for publishing temperature
        // TODO

        //! Message for publishing acc, gyro
        sensor_msgs::Imu ImuMsg;
        ImuMsg.header.stamp    = current_time;
        ImuMsg.header.frame_id = imu_frame_id;

        // set the values
        // ArduImu MPU6000 programmed:
        // Gyro scale 250º/s

        // Accelerometer
        // Real scale empiric conversion factor = 1/780  measure to m/s²
        // Theoretical conversion factor = 1/417 (Accel scale 4g = 4096 LSB / g)
        geometry_msgs::Vector3 lin_acc;
        lin_acc.x = (float)(0.327 * (rbk_arduimu->aX_ / 255.0));  // m/s^2
        lin_acc.y = (float)(0.327 * (rbk_arduimu->aY_ / 255.0));
        lin_acc.z = (float)(0.327 * (rbk_arduimu->aZ_ / 255.0));
        ImuMsg.linear_acceleration = lin_acc;
        // Gyro
        // Theoretical conversion factor = 1/131  (+/-250º/s = 16 bit) (131.072)
        // Real scale empiric conversion factor = 1/8400 7287.0
        geometry_msgs::Vector3 ang_vel;  // in rad/s
        ang_vel.x = (float)(rbk_arduimu->gX_ - rbk_arduimu->dgX_) / 7287.0;
        ang_vel.y = (float)(rbk_arduimu->gY_ - rbk_arduimu->dgY_) / 7287.0;
        ang_vel.z = (float)(rbk_arduimu->gZ_ - rbk_arduimu->dgZ_) / 7287.0;

        ImuMsg.angular_velocity = ang_vel;

        // publish the imu raw message
        imu_pub.publish(ImuMsg);

        //! Message for publishing magnetometer
        geometry_msgs::Vector3Stamped MagMsg;
        MagMsg.header.stamp    = current_time;
        MagMsg.header.frame_id = imu_frame_id;

        // set the values
        // HMC5883L device - Full scale = +/-8 gauss
        // XYZ
        // XZY
        // YXZ fallo rotx90 rotz180
        // YZX fallo rotx90 rotz180
        // ZYX
        // ZXY fallo rotx90 rotz180
        float mxc =
            (float)((rbk_arduimu->mX_ * 8.0 / 32768.0) - x_offset) / x_gain;
        float myc =
            (float)((rbk_arduimu->mY_ * 8.0 / 32768.0) - y_offset) / y_gain;
        float mzc =
            (float)((rbk_arduimu->mZ_ * 8.0 / 32768.0) - z_offset) / z_gain;
        MagMsg.vector.x = mxc;
        MagMsg.vector.y = myc;
        MagMsg.vector.z = mzc;
        // MagMsg.vector.x = (float) rbk_arduimu->mX_ * 8.0 / 32768.0;
        // MagMsg.vector.y = (float) rbk_arduimu->mY_ * 8.0 / 32768.0;
        // MagMsg.vector.z = (float) rbk_arduimu->mZ_ * 8.0 / 32768.0;
        // ROS_INFO("mX_=%d mY_=%d mZ_=%d", rbk_arduimu->mX_, rbk_arduimu->mY_,
        // rbk_arduimu->mZ_ );
        // publish the magnetometer message
        mag_pub.publish(MagMsg);

        float             heading = -atan2(MagMsg.vector.y, MagMsg.vector.x);
        std_msgs::Float32 yaw_mag_msg;
        mean = heading;
        for (i = 24; i > 0; i--)
        {
            mean_filter[i] = mean_filter[i - 1];
            mean += mean_filter[i];
        }
        mean_filter[0] = heading;
        mean           = mean / 25.0;
        // rbk_arduimu->yaw_mag_ = mean - rbk_arduimu->yaw_mag_init_;
        rbk_arduimu->yaw_mag_ = heading - rbk_arduimu->yaw_mag_init_;
        if (rbk_arduimu->yaw_mag_ > 3.1415926535)
            rbk_arduimu->yaw_mag_ -= 2.0 * 3.1415926535;
        if (rbk_arduimu->yaw_mag_ < -3.1415926535)
            rbk_arduimu->yaw_mag_ += 2.0 * 3.1415926535;
        // ROS_INFO("heading=%5.2f   mean=%5.2f", heading, mean);
        yaw_mag_msg.data = rbk_arduimu->yaw_mag_;
        yaw_mag_pub.publish(yaw_mag_msg);
        // ROS_INFO("heading=%5.2f ", heading / 3.1415926535 * 180.0 );

        ros::spinOnce();
        r.sleep();
    }

    rbk_arduimu->Close();
}
// EOF
