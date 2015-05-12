/** \file summit_controller_dspic.cc
 * \author Robotnik Automation S.L.L.
 * \version 2.0
 * \date    2012
 *
 * \brief summit_controller_dspic class driver
 * Component to manage summit_controller_dspic servo controller
 * (C) 2012 Robotnik Automation, SLL
*/

#include "summit_controller_dspic/summit_controller_dspic.h"
#include <math.h>
#include <cstdlib>
#include <std_msgs/Float32.h>
#include <robotnik_msgs/set_odometry.h>          // service
#include <robotnik_msgs/set_mode.h>
#include <sensor_msgs/JointState.h>


using namespace summit_ctrl_dspic;

std::string port;
summit_controller_dspic * summit_controller;

const bool summit_controller_dspic::bLocalOdometry = true;
const double summit_controller_dspic::fCountsPerRev = 33039.36;   // counts per wheel revolution = enc * gearbox = 512 * 64.53 = 33039.36

/*!	\fn summit_controller_dspic::dsPic()
 * 	\brief Public constructor
*/
summit_controller_dspic::summit_controller_dspic( std::string port ) {

	// Odometry initial values
	dsPic_data.dRPMtoMPS = DSPIC_RPM2MPS;
	dsPic_data.dDWheels = DSPIC_D_WHEELS_M;
	dsPic_data.iEncoder = 0;
	dsPic_data.iEncoderAnt = 0;
	dsPic_data.dGyro = dsPic_data.dOffsetGyro = 0.0;
	dsPic_data.rad = 0.0;   // turn angle reference
	dsPic_data.mps = 0.0;   // linear speed reference
        dsPic_data.dBatt = 0.0;
      	dsPic_data.bMotorsEnabled = true;
        dsPic_data.odometry_x = 0.0;
        dsPic_data.odometry_y = 0.0;
        dsPic_data.odometry_yaw = 0.0;
	pthread_mutex_init(&mutex_odometry, NULL); //Initialization for odometry's mutex
	iErrorType = DSPIC_ERROR_NONE;
    active_kinematic_mode_ = DUAL_ACKERMANN_INVERTED; //DUAL_ACKERMANN_INVERTED; // SINGLE_ACKERMANN;  // Default kinematic mode

	// Could be read from parameter yaml file
	dsPic_data.offset1[DSPIC_FWD_DIRECTION] = 5;
	dsPic_data.center[DSPIC_FWD_DIRECTION] = 0;
	dsPic_data.offset2[DSPIC_FWD_DIRECTION] = 5;
		
	dsPic_data.offset1[DSPIC_FWD_TRACTION] = 0;
	dsPic_data.center[DSPIC_FWD_TRACTION] = 0;
	dsPic_data.offset2[DSPIC_FWD_TRACTION] = 0;

	dsPic_data.offset1[DSPIC_BWD_DIRECTION] = 5;
	dsPic_data.center[DSPIC_BWD_DIRECTION] = 0;
	dsPic_data.offset2[DSPIC_BWD_DIRECTION] = -5;

	dsPic_data.offset1[DSPIC_BWD_TRACTION] = 0;
	dsPic_data.center[DSPIC_BWD_TRACTION] = 0;
	dsPic_data.offset2[DSPIC_BWD_TRACTION] = 0;
	
	dsPic_data.last_channel = DSPIC_FWD_DIRECTION;  // Default channel to be accessed

    // Create serial port
	serial = new SerialDevice(port.c_str(), DSPIC_DEFAULT_TRANSFERRATE, DSPIC_DEFAULT_PARITY, DSPIC_DEFAULT_DATA_SIZE); //Creates serial device
	amMode = GYRO;
	bOffsetCalibration = bSentOffsetCalibration = false;
};

/*!	\fn summit_controller_dspic::~summit_controller_dspic()
 * 	\brief Public destructor
*/
summit_controller_dspic::~summit_controller_dspic(){

	// Close serial port
	if (serial!=NULL) serial->ClosePort();

     // Delete serial port
     if (serial!=NULL) delete serial;

     // Destroy odometry mutex
     pthread_mutex_destroy(& mutex_odometry );
}


/*!	\fn void summit_controller_dspic::SwitchToState(States new_state)
 * 	function that switches the state of the component into the desired state
 * 	\param new_state as an integer, the new state of the component
*/
void summit_controller_dspic::SwitchToState(States new_state){

	if(new_state == iState_)
		return;
		
	iState_ = new_state;
	switch (new_state) {
		case INIT_STATE:
			ROS_INFO("summit_controller_dspic:: switching to INIT state");
			break;
		case STANDBY_STATE:
			ROS_INFO("summit_controller_dspic:: switching to STANDBY state");
			break;
		case READY_STATE:
			ROS_INFO("summit_controller_dspic:: switching to READY state");
			break;
		case EMERGENCY_STATE:
			ROS_INFO("summit_controller_dspic:: switching to EMERGENCY state");
			break;
		case FAILURE_STATE:
			ROS_INFO("summit_controller_dspic:: switching to FAILURE state");
			break;
        	case SHUTDOWN_STATE:
			ROS_INFO("summit_controller_dspic:: switching to SHUTDOWN state");
			break;
		default:
			ROS_ERROR("summit_controller_dspic:: Switching to UNKNOWN state");
			break;
	}
}

/*!	\fn void summit_controller_dspic::StateMachine()
 * 	function that manages internal states of the component 
*/
void summit_controller_dspic::StateMachine(){

   switch (iState_){
        case INIT_STATE:
                InitState();
        break;
        case STANDBY_STATE:
                StandbyState();
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

/*!	\fn int summit_controller_dspic::Open(char *dev)
 * 	\brief Open serial port
 * 	\returns -1 Error
 * 	\returns 0 Ok
*/
int summit_controller_dspic::Open(){

	// Setup serial device
	if (this->serial->OpenPort2() == SERIAL_ERROR) {
      ROS_ERROR("summit_controller_dspic::Open: Error Opening Serial Port");
	  SwitchToState(FAILURE_STATE);
	  iErrorType = DSPIC_ERROR_OPENING;
	  return ERROR;
      }
	ROS_INFO("summit_controller_dspic::Open: serial port opened at %s", serial->GetDevice());

	// Send Set Trimmer
	SendSetTrimmer( DSPIC_FWD_DIRECTION );
	usleep(500000);
	int ret = ReadControllerMsgs();  
	if (ret!=OK) ROS_WARN("Couldn't configure trimmer for FWD DIRECTION");
	SendSetTrimmer( DSPIC_BWD_DIRECTION );
	usleep(500000);
	ret = ReadControllerMsgs();  
	if (ret!=OK) ROS_WARN("Couldn't configure trimmer for BWD DIRECTION");
	
	// Send 1st command to verify device is working
    SendCalibrateOffsetGyro();
	usleep(1200000);    
    SendReadEncoderGyro();
    usleep(50000);
	SwitchToState(INIT_STATE);

	return OK;
}

/*!	\fn int summit_controller_dspic::Close()
 * 	\brief Closes serial port
 * 	\returns ERROR
 * 	\returns OK
*/
int summit_controller_dspic::Close(){

	if (serial!=NULL) {
        	serial->ClosePort();
        	}
	return OK;
}

/*!	\fn void summit_controller_dspic::InitState()
 * 	\brief Actions in the initial state
 * 	First call-> Sends start data logging message -> IF ERROR go to FAILURE_STATE
 *	-> IF OK inits communication's timers
 * 	Next call -> Reads messages from the sensor and compares timers	-> IF no responses go to FAILURE_STATE
 * 	-> IF OK go to READY_STATE
*/
void summit_controller_dspic::InitState(){
	static int count_errors_reading = 0, count_errors_sending = 0;
	int ret = 0, ret2 = 0;

	ret = ReadControllerMsgs();
	switch(ret) {
		case ERROR:	// Nothing received 
			ROS_ERROR("summit_controller_dspic::InitState: received ERROR");
			count_errors_reading++;
		break;
		case ERROR_CRC:
			ROS_ERROR("summit_controller_dspic::InitState: Error CRC");
			count_errors_reading++;
		break;
		case ERROR_CMD:
			ROS_ERROR("summit_controller_dspic::InitState: Error CMD");
			count_errors_reading++;
		break;
		case ERROR_OVFL:
			ROS_ERROR("summit_controller_dspic::InitState: Error OVFL");
			count_errors_reading++;
		break;
		case ERROR_OVRUN:
			ROS_ERROR("summit_controller_dspic::InitState: Error OVRUN");
			count_errors_reading++;
		break;
		case ERROR_FRAME:
			ROS_ERROR("summit_controller_dspic::InitState: Error FRAME");
			count_errors_reading++;
		break;
		case ERROR_OUTRANGE:
			ROS_ERROR("summit_controller_dspic::InitState: Error OUTofRANGE");
			count_errors_reading++;
		break;
		case ERROR_SPI:
			ROS_ERROR("summit_controller_dspic::InitState: Error SPI");
			//count_errors_reading++;
		break;
		case ERROR_CMDOV: 
            ROS_ERROR("summit_controller_dspic::InitState: Error CMDOV - Command Overflow");
            count_errors_reading++;
        break;
        case ERROR_PARAMOV:
            ROS_ERROR("summit_controller_dspic::InitState: Error PARAMOV - Parameter Overflow");
            count_errors_reading++;
        break;
		case OK:
			dsPic_data.iEncoderAnt = dsPic_data.iEncoder;
			// Inits dsPic reply timer
			tDsPicReply = ros::Time::now();
			SwitchToState(READY_STATE);
		break;
	}

	ret2 = SendReadEncoderGyro();
	if(ret2 == ERROR)
		count_errors_sending++;
	else
		count_errors_sending = 0;

	if((count_errors_sending >= DSPIC_MAX_ERRORS) /*|| ((count_errors_reading >= DSPIC_MAX_ERRORS))*/){
		iErrorType = DSPIC_ERROR_SERIALCOMM;
		ROS_ERROR("summit_controller_dspic::InitState: %d errors sending, %d errors reading", count_errors_sending, count_errors_reading);
		SwitchToState(FAILURE_STATE);
		count_errors_sending = 0;
        }

}

/*! \fn void summit_controller_dspic::ReadyState()
 *  \brief Actions in Ready state
 *    First time -> Gets current time
 *    Next times -> Read from controller and checks the timeout between responses
 *    -> IF TIMEOUT go to FAILURE_STATE
 *    -> IF OK resets communication's timer
*/
void summit_controller_dspic::ReadyState(){

	static int count_errors_reading = 0, count_errors_sending = 0;
	int ret = 0, ret2 = 0;

	// Actions in ReadyState
	ret = ReadControllerMsgs();
	switch(ret) {
		case ERROR:	// Nothing received
			count_errors_reading++;
		break;
		case ERROR_CRC:
			ROS_ERROR("summit_controller_dspic::ReadyState: Error CRC");
			count_errors_reading++;
		break;
		case ERROR_CMD:
			ROS_ERROR("summit_controller_dspic::ReadyState: Error CMD");
			count_errors_reading++;
		break;
		case ERROR_OVFL:
			ROS_ERROR("summit_controller_dspic::ReadyState: Error OVFL");
			count_errors_reading++;
		break;
		case ERROR_OVRUN:
			ROS_ERROR("summit_controller_dspic::ReadyState: Error OVRUN");
			count_errors_reading++;
		break;
		case ERROR_FRAME:
			ROS_ERROR("summit_controller_dspic::ReadyState: Error FRAME");
			count_errors_reading++;
		break;
		case ERROR_OUTRANGE:
			ROS_ERROR("summit_controller_dspic::ReadyState: Error OUTofRANGE");
			count_errors_reading++;
		break;
		case ERROR_SPI:
			ROS_ERROR("summit_controller_dspic::ReadyState: Error SPI");
			//count_errors_reading++;
                break;
                case ERROR_CMDOV:
                        ROS_ERROR("summit_controller_dspic::ReadyState: Error CMDOV - Command Overflow");
                        count_errors_reading++;
                break;
                case ERROR_PARAMOV:
                        ROS_ERROR("summit_controller_dspic::ReadyState: Error PARAMOV - Parameter Overflow");
                        count_errors_reading++;
                break;

		default:
			// If no errors, reset counter
			count_errors_reading = 0;
			// Saves current time
			// Inits nodeguard reply timer
			tDsPicReply = ros::Time::now(); 
			ROS_INFO("summit_controller_dspic::ReadyState: Ret = %d", ret);
		break;
	}

    int ch= 0;
	switch (iCmd) {

		case DSPIC_CMD_MODIFY_ODOMETRY:
			ret2 = SendModifyOdometry(dsPic_data_tmp.odometry_x, dsPic_data_tmp.odometry_y, dsPic_data_tmp.odometry_yaw );
            	break;
		case DSPIC_CMD_INIT_ODOMETRY:
			ret2 = SendInitOdometry();
            	break;
		case DSPIC_CMD_RESET_ODOMETRY:
			ret2 = SendResetOdometry();
            break;
		case DSPIC_CMD_SET_D:
			ret2 = SendSetDWheels(dsPic_data_tmp.dDWheels);
            break;
		case DSPIC_CMD_SET_R:
			ret2 = SendSetRPMtoMPS(dsPic_data_tmp.dRPMtoMPS);
            break;
		case DSPIC_CMD_READ_ODOMETRY:
			ret2 = SendReadOdometry();
            break;
		case DSPIC_CMD_RESET_CONTROLLER:
			ret2 = SendResetController();
		    break;
		case DSPIC_CMD_READ_PARAMETERS:
			ret2 = SendReadParameters();
		    break;
		case DSPIC_CMD_READ_ENCODER:
			ret2 = SendReadEncoder();
            break;
		case DSPIC_CMD_READ_ENCODER_GYRO:
			ret2 = SendReadEncoderGyro();
	        break;
		case DSPIC_CMD_CALIBRATE_OFFSET:
			ret2 = SendCalibrateOffsetGyro();
			SwitchToState(STANDBY_STATE);	// Stays on standby until checks the new and right offset
         	break;
        case DSPIC_CMD_SET_MOTOR_REFERENCES:
            ret2 = SendSetMotorReferences(dsPic_data_tmp.rad, dsPic_data_tmp.mps);
            break;
        case DSPIC_CMD_SET_TRIMMER:
			ch = dsPic_data.last_channel;
			ret2 = SendSetTrimmer( ch );
			break;
	    default:
            // Default command according to mode
	        ret2 = SendReadEncoderGyro();
	        break;
	}

	if(ret2 == ERROR)
		count_errors_sending++;
	else
		count_errors_sending = 0;

	// Odometry local calculation
	if(bLocalOdometry){
		UpdateOdometry();
		iCmd = DSPIC_CMD_READ_ENCODER_GYRO; 	// Next command by default
	}else
		iCmd = DSPIC_CMD_READ_ODOMETRY; 	// Next command by default

	if(count_errors_sending >= DSPIC_MAX_ERRORS){
		iErrorType = DSPIC_ERROR_SERIALCOMM;
		ROS_ERROR("summit_controller_dspic::ReadyState: %d errors sending, %d errors reading", count_errors_sending, count_errors_reading);
		SwitchToState(FAILURE_STATE);
		count_errors_sending = 0;
		count_errors_reading = 0;
        }

	// Checks the communication status
	ros::Time current_time = ros::Time::now(); 
	if (((current_time - tDsPicReply).toSec() > DSPIC_TIMEOUT_COMM) && !bSentOffsetCalibration) {
	// if(diff > 150000 && !bSentOffsetCalibration ){ //
		ROS_ERROR("summit_controller_dspic::ReadyState: Timeout in communication with the device");
		tDsPicReply = ros::Time::now();	// Resets the timer
        }

}

/*!	\fn void summit_controller_dspic::StandbyState()
 * 	\brief Actions in standby state
 */
void summit_controller_dspic::StandbyState(){
	static int count_errors_reading = 0, count_errors_sending = 0;
	int ret = 0, ret2 = 0;

	ret = ReadControllerMsgs();

	switch(ret) {
		case ERROR:	// No recive nada
			count_errors_reading++;
		break;
		case ERROR_CRC:
			ROS_ERROR("summit_controller_dspic::StandbyState: Error CRC");
			count_errors_reading++;
		break;
		case ERROR_CMD:
			ROS_ERROR("summit_controller_dspic::StandbyState: Error CMD");
			count_errors_reading++;
		break;
		case ERROR_OVFL:
			ROS_ERROR("summit_controller_dspic::StandbyState: Error OVFL");
			count_errors_reading++;
		break;
		case ERROR_OVRUN:
			ROS_ERROR("summit_controller_dspic::StandbyState: Error OVRUN");
			count_errors_reading++;
		break;
		case ERROR_FRAME:
			ROS_ERROR("summit_controller_dspic::StandbyState: Error FRAME");
			count_errors_reading++;
		break;
		case ERROR_OUTRANGE:
			ROS_ERROR("summit_controller_dspic::StandbyState: Error OUTofRANGE");
			count_errors_reading++;
		break;
		case ERROR_SPI:
			ROS_ERROR("summit_controller_dspic::StandbyState: Error SPI");
			//count_errors_reading++;
		break;
		default:
			// Reset if no error arrives
			count_errors_reading = 0;
			// Saves current time, inits nodeguard reply timer
			tDsPicReply = ros::Time::now();
		break;
	}

	if(!bSentOffsetCalibration && !bSuccessCalibration){
		ret2 = SendCalibrateOffsetGyro();
		ROS_INFO("summit_controller_dspic::Standby: Sending calibrate offset");
	}else if(bSuccessCalibration){
		iCmd = DSPIC_CMD_DEFAULT;
		SwitchToState(READY_STATE);
		// Saves current time, inits nodeguard reply timer
		tDsPicReply = ros::Time::now();
        }

	if(ret2 == ERROR)
		count_errors_sending++;
	else
		count_errors_sending = 0;

	if((count_errors_sending >= DSPIC_MAX_ERRORS)){
		iErrorType = DSPIC_ERROR_SERIALCOMM;
		ROS_ERROR("summit_controller_dspic::StandbyState: %d errors sending, %d errors reading", count_errors_sending, count_errors_reading);
		SwitchToState(FAILURE_STATE);
		count_errors_sending = 0;
		count_errors_reading = 0;
        }

	// Checks the communication status
	ros::Time current_time = ros::Time::now(); 
	if ( (current_time - tDsPicReply).toSec() > DSPIC_TIMEOUT_COMM) {
		ROS_ERROR("summit_controller_dspic::Standby: Timeout in communication with the device");
		tDsPicReply = ros::Time::now();	 // Resets the timer
        }

}

/*!	\fn void summit_controller_dspic::FailureState()
 * 	\brief Actions in Failure State
*/
void summit_controller_dspic::FailureState(){
	int timer = 2500000; //useconds
	static int recovery_cycles = 0;

	recovery_cycles++;
	if(recovery_cycles >= 40){ //Try to recover every 'second'
		switch(iErrorType)	{
			ROS_INFO("summit_controller_dspic::FailureState: Trying to recover..");
			case DSPIC_ERROR_OPENING://Try to recover
				ROS_ERROR("summit_controller_dspic::FailureState: Recovering from failure state (DSPIC_ERROR_OPENING.)");
				this->Close();
				usleep(timer);
				this->Open();
			break;
			case DSPIC_ERROR_SERIALCOMM:
				ROS_ERROR("summit_controller_dspic::FailureState: Recovering from failure state (DSPIC_ERROR_SERIALCOMM.)");
				this->Close();
				usleep(timer);
				this->Open();
			break;
			case DSPIC_ERROR_TIMEOUT:
				ROS_ERROR("summit_controller_dspic::FailureState: Recovering from failure state (DSPIC_ERROR_TIMEOUT.)");
				this->Close();
				usleep(timer);
				this->Open();
			break;
		}
		recovery_cycles = 0;
	}
}

/*!	\fn int summit_controller_dspic::ReadControllerMsgs()
 * 	Read RS-232 messages from the controller
 * 	Returns OK if messages readed
 * 	Return ERROR otherwise
*/
int summit_controller_dspic::ReadControllerMsgs(){
	int read_bytes=0;			//Number of received bytes
	int ret = ERROR;
	char cReadBuffer[64] = "\0";

	//ROS_INFO("summit_controller_dspic::ReadControllerMsgs: Reading...");
	//Reads the response
	//while((n=serial->ReadPort(cReadBuffer, 64)) > 0){

	if (serial->ReadPort(cReadBuffer, &read_bytes, 64)==ERROR) {
	    ROS_ERROR("summit_controller_dspic::ReadControllerMsgs: Error reading port");
	    return ERROR;
        }

	while(read_bytes > 0){
        //printf(cReadBuffer);
		ret = ProcessMsg(cReadBuffer); // returns ERROR or Number of bytes !
		// Robotnik update 18-6-2013 check all cmd errors
		// if (ret < 0) return ERROR;
		if(ret == ERROR) return ERROR;
		else ret = OK;
        if (serial->ReadPort(cReadBuffer, &read_bytes, 64)==ERROR) {
				 // Verificar. En ppio da igual si lee o no.
				 // ROS_ERROR("summit_controller_dspic::ReadControllerMsgs: Error after 1st port read");
				 // return ERROR;
				 ret = OK;
                }
        }

	return ret;
}

/*!	\fn int summit_controller_dspic::SendResetController()
	* Allows the controller to be reset
	* \return OK
	* \return ERROR
*/
int summit_controller_dspic::SendResetController(){
	char cMsg[8] = "\0";
	int written_bytes=0;

	sprintf(cMsg,"%s\r", RESET_CONTROLLER);
	ROS_INFO("summit_controller_dspic::SendResetController: Sending reset command");
	// Sends the message
	if(serial->WritePort(cMsg, &written_bytes, strlen(cMsg)) != OK) {
		ROS_ERROR("summit_controller_dspic::SendResetController: Error sending message");
		return ERROR;
        }
	return OK;
}

/*!	\fn int summit_controller_dspic::SendResetOdometry()
	* Resets internal odometry
 	* \return OK
	* \return ERROR
*/
int summit_controller_dspic::SendResetOdometry(){
	char cMsg[3] = "\0";
	int written_bytes = 0;

	// If odometry calculation is local we don't need to send this reset
	if(bLocalOdometry) return OK;

	// Sends init odometry message
	sprintf(cMsg,"%s\r", INIT_ODOMETRY);
	if(serial->WritePort(cMsg, &written_bytes, strlen(cMsg)) != OK) {
		ROS_ERROR("summit_controller_dspic::SendResetOdometry: Error sending message");
		return ERROR;
        }
	return OK;
}

/*!	\fn int summit_controller_dspic::SendModifyOdometry(double x, double y, double yaw)
	* Changes odometry's current values
 	* \return OK
	* \return ERROR
*/
int summit_controller_dspic::SendModifyOdometry(double x, double y, double yaw){
	char cMsg[32] = "\0", cMsg2[64] = "\0";
	int crc = 0;
	int written_bytes=0;

	// If odometry calculation is local we don't need to send this command
	if(bLocalOdometry) return OK;

    	// Prepare modify odometry command
	sprintf(cMsg,"%s,%5.3f,%5.3f,%5.3f", SET_ODOMETRY, x, y, yaw);
	crc = ComputeCRC(cMsg, 32);
	sprintf(cMsg2,"%s,%5.3f,%5.3f,%5.3f,%d\r", SET_ODOMETRY, x, y, yaw, crc);
	ROS_INFO("summit_controller_dspic::SendModifyOdometry: Sending %s,%5.3f,%5.3f,%5.3f,%d to device", SET_ODOMETRY, x, y, yaw, crc);

	// Sends the message
	if(serial->WritePort(cMsg2, &written_bytes, strlen(cMsg2)) != OK) {
		ROS_ERROR("summit_controller_dspic::SendModifyOdometry: Error sending message");
		return ERROR;
        }

	return OK;
}

/*!	\fn  int summit_controller_dspic::SendInitOdometry()
	* Sends init odometry msg
 	* \return OK
	* \return ERROR
*/
int summit_controller_dspic::SendInitOdometry(){
	char cMsg[3] = "\0";
	int written_bytes=0;

	// If odometry calculation is local we don't need to send this command
	if(bLocalOdometry) return OK;

	// Prepare init odometry message
	sprintf(cMsg,"%s\r", INIT_ODOMETRY);
	ROS_INFO("summit_controller_dspic::SendInitOdometry: Sending %s to device", INIT_ODOMETRY);
	// Send the message
	if(serial->WritePort(cMsg, &written_bytes, strlen(cMsg)) != OK) {
		ROS_ERROR("summit_controller_dspic::SendInitOdometry: Error sending message");
		return ERROR;
        }

	return OK;
}

/*!	\fn  int summit_controller_dspic::SendSetDWheels(double distance)
	* Sets the distance between wheels msg
 	* \return OK
	* \return ERROR
*/
int summit_controller_dspic::SendSetDWheels(double distance){
	char cMsg[32] = "\0", cMsg2[32] = "\0";
	int crc = 0;
	int written_bytes=0;

	// If odometry calculation is local we don't need to send this command
	if(bLocalOdometry) return OK;

    	// Prepare message
	sprintf(cMsg,"%s,%2.5f", SET_DWHEELS,distance);
	crc = ComputeCRC(cMsg, 32);
	sprintf(cMsg2,"%s,%2.5f,%d\r", SET_DWHEELS, distance, crc);
	ROS_INFO("summit_controller_dspic::SendSetDWheels: Sending %s,%2.5f,%d to device", SET_DWHEELS,distance, crc);
	// Send the message
	if(serial->WritePort(cMsg2, &written_bytes, strlen(cMsg2)) != OK) {
		ROS_ERROR("summit_controller_dspic::SendSetDWheels: Error sending message");
		return ERROR;
        }
	return OK;
}

/*!	\fn  int summit_controller_dspic::SendSetRPMtoMPS(double value)
	* Sets the value for converting from RPM to MPS
 	* \return OK
	* \return ERROR
*/
int summit_controller_dspic::SendSetRPMtoMPS(double value){
	char cMsg[32] = "\0", cMsg2[32] = "\0";
	int crc = 0;
	int written_bytes=0;

	// If odometry calculation is local we don't need to send this command
	if(bLocalOdometry) return OK;

	// Prepare message
	sprintf(cMsg,"%s,%2.9f", SET_RPM2MPS, value);
	crc = ComputeCRC(cMsg, 32);
	sprintf(cMsg2,"%s,%2.9f,%d\r", SET_RPM2MPS, value, crc);
	ROS_INFO("summit_controller_dspic::SendSetRPMtoMPS: Sending %s,%2.9f,%d to device", SET_RPM2MPS, value, crc);
	// Send the message
	if(serial->WritePort(cMsg2, &written_bytes, strlen(cMsg2)) != OK) {
		ROS_ERROR("summit_controller_dspic::SendSetRPMtoMPS: Error sending message");
		return ERROR;
		}
	return OK;
}

/*!	\fn int summit_controller_dspic::SendReadOdometry()
	* Sends read odometry msg and reads the value
 	* \return OK
	* \return ERROR
*/
int summit_controller_dspic::SendReadOdometry(){
	char cMsg[3] = "\0";
	int written_bytes=0;

	// If odometry calculation is local we don't need to send this command
	if(bLocalOdometry) return OK;

	// Prepare message
	sprintf(cMsg,"%s\r", GET_ODOMETRY);
	//ROS_INFO("summit_controller_dspic::SendReadOdometry: Sending %s to device", GET_ODOMETRY);

	// Send the message
	if(serial->WritePort(cMsg, &written_bytes, strlen(cMsg)) != OK) {
		ROS_ERROR("summit_controller_dspic::SendReadOdometry: Error sending message");
		return ERROR;
        }
	return OK;
}

/*!	\fn int summit_controller_dspic::SendReadParameters()
	* Sends read parameters msg
 	* \return OK
	* \return ERROR
*/
int summit_controller_dspic::SendReadParameters(){
	char cMsg[3] = "\0";
	int written_bytes=0;

    	// If odometry calculation is local we don't need to send this command
	if(bLocalOdometry) return OK;

    	// Prepare message
	sprintf(cMsg,"%s\r", GET_PARAMETERS);
	//ROS_INFO("summit_controller_dspic::SendReadParameters: Sending %s to device", GET_PARAMETERS);

	// Send the message
	if(serial->WritePort(cMsg, &written_bytes, strlen(cMsg)) != OK) {
		ROS_ERROR("summit_controller_dspic::SendReadParameters: Error sending message");
		return ERROR;
        }

	return OK;
}

/*!	\fn int summit_controller_dspic::SendReadEncoder()
	* Reads encoder
 	* \return OK
	* \return ERROR
*/
int summit_controller_dspic::SendReadEncoder(){
	char cMsg[3] = "\0";
	int written_bytes=0;

    	// If odometry calculation is local we don't need to send this command
	if(bLocalOdometry) return OK;

	// Prepare and send message
	sprintf(cMsg,"%s\r", GET_ENCODER);
	if(serial->WritePort(cMsg, &written_bytes, strlen(cMsg)) != OK) {
		ROS_ERROR("summit_controller_dspic::SendReadEncoder: Error sending message");
		return ERROR;
        }
	return OK;
}

/*!	\fn int summit_controller_dspic::SendReadEncoderGyro()
	* Reads encoders and gyroscope's value
 	* \return OK
	* \return ERROR
*/
int summit_controller_dspic::SendReadEncoderGyro(){
	char cMsg[3] = "\0";
	int written_bytes=0;

        // Prepare and send message
	sprintf(cMsg,"%s\r", GET_ENCODER_GYRO);
	if(serial->WritePort(cMsg, &written_bytes, strlen(cMsg)) != OK) {
		ROS_ERROR("summit_controller_dspic::SendReadEncoderGyro: Error sending message");
		return ERROR;
        }
	return OK;
}

/*!	\fn int summit_controller_dspic::SendCalibrateOffsetGyro()
	* Sends the message to recalibrate the offset
 	* \return OK
	* \return ERROR
*/
int summit_controller_dspic::SendCalibrateOffsetGyro(){
	char cMsg[3] = "\0";
	int written_bytes=0;

	sprintf(cMsg,"%s\r", CALIBRATE_OFFSET);

	// Sends the message
	if(serial->WritePort(cMsg, &written_bytes, strlen(cMsg)) != OK) {
		ROS_ERROR("summit_controller_dspic::SendCalibrateOffsetGyro: Error sending message");
		return ERROR;
        }

	bSentOffsetCalibration = true;
	bSuccessCalibration = false;

	return OK;
}

/*!	\fn  int summit_controller_dspic::SendSetMotorReferences(double rad, double mps)
	* Sets the values for moving
 	* \return OK
	* \return ERROR
*/
int summit_controller_dspic::SendSetMotorReferences(double rad, double mps){
	char cMsg[32] = "\0";
	int crc = 0;
	int written_bytes=0;

	if (dsPic_data.bMotorsEnabled==true) {
	   if (active_kinematic_mode_== DUAL_ACKERMANN_INVERTED) sprintf(cMsg,"%s,%1.2f,%1.2f,%1.2f\r", SET_MOTOR_REFERENCES, rad, rad, mps);
	   else if (active_kinematic_mode_== SINGLE_ACKERMANN) sprintf(cMsg,"%s,%1.2f,%1.2f,%1.2f\r", SET_MOTOR_REFERENCES, rad, 0.0, mps);
	   else if (active_kinematic_mode_== DUAL_ACKERMANN_GEARED) sprintf(cMsg,"%s,%1.2f,%1.2f,%1.2f\r", SET_MOTOR_REFERENCES, rad, -rad, mps);
	   else sprintf(cMsg,"%s,0.0,0.0,0.0\r", SET_MOTOR_REFERENCES);
	   }
	else
       sprintf(cMsg,"%s,0.0,0.0,0.0\r", SET_MOTOR_REFERENCES);

	crc = ComputeCRC(cMsg, 32);
	ROS_INFO("summit_controller_dspic - Motor References %s,%2.3f,%2.3f,%d\r", SET_MOTOR_REFERENCES, rad, mps, crc);
	ROS_INFO("summit_controller_dspic::SendSetMotorReferences: Sending %s,%2.3f,%2.3f,%d to device", SET_MOTOR_REFERENCES, rad, mps, crc);

    // Sends the message
	if(serial->WritePort(cMsg, &written_bytes, strlen(cMsg)) != OK) {
           ROS_ERROR("summit_controller_dspic::SendSetMotorReferences: Error sending message");
	   return ERROR;
	   }

        return OK;
}

/*!	\fn  int summit_controller_dspic::SendSetTrimmer(int channel, int offset1, int center, int offset2)
	* Sets the distance between wheels msg
 	* \return OK
	* \return ERROR
*/
int summit_controller_dspic::SendSetTrimmer(int channel){

	char cMsg[32] = "\0", cMsg2[32] = "\0";
	int crc = 0;
	int written_bytes=0;

    // Prepare message
    int offset1 = dsPic_data.offset1[channel];
    int center = dsPic_data.center[channel];
    int offset2 = dsPic_data.offset2[channel];
	sprintf(cMsg,"%s,%d,%d,%d,%d\r", SET_TRIMMER, channel, offset1, center, offset2 );
	ROS_INFO("SendSetTrimmer %s,%d,%d,%d,%d", SET_TRIMMER, channel, offset1, center, offset2 );
	if(serial->WritePort(cMsg, &written_bytes, strlen(cMsg)) != OK) {
		ROS_ERROR("summit_controller_dspic::SendSetTrimmer: Error sending message");
		return ERROR;
        }
	return OK;
}

/*!	\fn unsigned char summit_controller_dspic::ComputeCRC(char *string, int size)
	* Calculates the CRC for the message
 	* \return OK
	* \return ERROR
*/
unsigned char summit_controller_dspic::ComputeCRC(char *string, int size){
	int j=0;
    unsigned char crc=0;

    while ( (string[j] != 0) && ( j< size ) ) {
        crc +=string[j];
		//ROS_INFO(" char[%d] -> %c = %d, crc = %d", j, string[j], string[j], (int)crc);
        j++;
     }
    return crc;
}

/*!	\fn int summit_controller_dspic::ProcessMsg(char *msg)
	* Process received messages from the device
 	* \return ERROR if nothing for processing
 	* \return NUM of Error from DSPIC
*/
int summit_controller_dspic::ProcessMsg(char *msg){
	char *ptr;
	char cReceivedTokens[10][16];
	int i = 0;
	int num_error = 0;
	char command[64] = "\0";
	double x = 0.0, y = 0.0, th = 0.0;
        double batt = 0.0;
	float d = 0.0, r = 0.0;
	int received_crc = 0, crc = 0;
	int enc = 0;
	int MAX_TOKENS = 10;

	ptr = strtok (msg,",");

	// Extract all strings separated by comas
	// All tokens sent by device MUST have length < 16 or 
	// buffer ovfl will be generated 
	while ( (ptr != NULL) && (i<(MAX_TOKENS-1)) ) {
		sprintf(cReceivedTokens[i],"%s", ptr);
		ptr = strtok (NULL, ",");
		i++;
        }

	if(i == 0){
		ROS_ERROR("summit_controller_dspic::ProcessMsg: nothing for processing");
		return ERROR;
        }
	//ROS_INFO(cReceivedTokens[0]);

	// Process the received response
	// Errors from device
	if(!strcmp(cReceivedTokens[0], "ERROR")){
		num_error = atoi(cReceivedTokens[1]);
		ROS_ERROR("summit_controller_dspic::ProcessMsg: Error from device: %s,%s", cReceivedTokens[0], cReceivedTokens[1]);
		return num_error;
        }

        // Reset detected
        if (i>=1) {
  	if(!strcmp(cReceivedTokens[1], "Robotnik")){
		ROS_ERROR("summit_controller_dspic::ProcessMsg: dsPIC board reseted !!!");
 		return BOARD_RESET;
                }
             }

	// Standard commands
	if(!strcmp(cReceivedTokens[0], GET_ODOMETRY ) && !bLocalOdometry){
		//ROS_INFO("summit_controller_dspic::ProcessMsg: Odometry: x = %s, y = %s, th = %s, crc %s", cReceivedTokens[1], cReceivedTokens[2],
		//			cReceivedTokens[3], cReceivedTokens[4]);
		x = atof(cReceivedTokens[1]);
		y = atof(cReceivedTokens[2]);
		th = atof(cReceivedTokens[3]);
		received_crc = atoi(cReceivedTokens[4]);
		sprintf(command,"%s,%5.3f,%5.3f,%5.3f", GET_ODOMETRY, x, y, th);
		crc = ComputeCRC(command, 32);
		//ROS_INFO("summit_controller_dspic::ProcessMsg: Converted odom: x = %5.3f, y = %5.3f, th = %5.3f, crc (%d, %d)", x, y, th, received_crc, crc);
		// If data arrived correctly
		if(crc == received_crc){
			// Actualizamos odometría
			dsPic_data.odometry_x = x;
			dsPic_data.odometry_y = y;
			dsPic_data.odometry_yaw = th;
		}else{
			ROS_ERROR("summit_controller_dspic::ProcessMsg: Get Odometry: Error crc: rec = %d, calc = %d", received_crc, crc);
			return ERROR_GET_ODOMETRY;
            }
		return OK_GET_ODOMETRY;
        }
	if(!strcmp(cReceivedTokens[0], SET_ODOMETRY)){
		//ROS_INFO("summit_controller_dspic::ProcessMsg: Set odometry OK: x = %s, y = %s, th = %s, crc %s", cReceivedTokens[1], cReceivedTokens[2],
		//			cReceivedTokens[3], cReceivedTokens[4] );
		x = atof(cReceivedTokens[1]);
		y = atof(cReceivedTokens[2]);
		th = atof(cReceivedTokens[3]);
		received_crc = atoi(cReceivedTokens[4]);
		sprintf(command,"%s,%5.3f,%5.3f,%5.3f", SET_ODOMETRY, x, y, th);
		crc = ComputeCRC(command, 32);
		//ROS_INFO("summit_controller_dspic::ProcessMsg: Converted odom: x = %f. y = %f, th = %f, crc (%d, %d)", x, y, th, received_crc, crc);
		if(crc != received_crc){
			ROS_ERROR("summit_controller_dspic::ProcessMsg: Set odometry: Error crc: recv = %d, calc = %d", received_crc, crc);
			return ERROR_SET_ODOMETRY;
		}else
			return OK_SET_ODOMETRY;
        }
	if(!strcmp(cReceivedTokens[0], INIT_ODOMETRY)){
		//ROS_INFO("summit_controller_dspic::ProcessMsg: Init odometry OK: %s", cReceivedTokens[1]);
		return OK_INIT_ODOMETRY;
        }
	if(!strcmp(cReceivedTokens[0], SET_DWHEELS)){
		//ROS_INFO("summit_controller_dspic::ProcessMsg: Set DWHEELS: D = %s, crc %s", cReceivedTokens[1], cReceivedTokens[2]);
		d = atof(cReceivedTokens[1]);
		received_crc = atoi(cReceivedTokens[2]);
		sprintf(command,"%s,%2.5f", SET_DWHEELS, d);
		crc = ComputeCRC(command, 32);
		//ROS_INFO("summit_controller_dspic::ProcessMsg: converted Dwheels : d = %f, crc (%d, %d)", d, received_crc, crc);
		if(crc != received_crc){
			ROS_ERROR("summit_controller_dspic::ProcessMsg: Set Dwheels: Error crc: recv = %d, calc = %d", received_crc, crc);
			return ERROR_SET_DWHEELS;
		}else
			return OK_SET_DWHEELS;
        }
	if(!strcmp(cReceivedTokens[0], SET_RPM2MPS)){
		ROS_INFO("summit_controller_dspic::ProcessMsg: Set Rpm2mps: value = %s, crc %s", cReceivedTokens[1], cReceivedTokens[2]);
		r = atof(cReceivedTokens[1]);
		received_crc = atoi(cReceivedTokens[2]);
		sprintf(command,"%s,%2.9f", SET_RPM2MPS, r);
		crc = ComputeCRC(command, 32);
		//ROS_INFO("summit_controller_dspic::ProcessMsg: converted : r = %f, crc (%d, %d)", r, received_crc, crc);
		if(crc != received_crc){
			ROS_ERROR("summit_controller_dspic::ProcessMsg: Set rpm2mps: Error crc: recv = %d, calc = %d", received_crc, crc);
			return ERROR_SET_RPM2MPS;
		}else
			return OK_SET_RPM2MPS;
        }
	if(!strcmp(cReceivedTokens[0], GET_PARAMETERS)){
		//ROS_INFO("summit_controller_dspic::ProcessMsg: Parameters: d=%s, r=%s, crc%s", cReceivedTokens[1], cReceivedTokens[2],
        	//	cReceivedTokens[3]);
		d = atof(cReceivedTokens[1]);
		r = atof(cReceivedTokens[2]);
		received_crc = atoi(cReceivedTokens[3]);
		//memset(command, 0, 64);
		sprintf(command,"%s,%2.5f,%2.9f", GET_PARAMETERS, d, r);
		crc = ComputeCRC(command, 64);
		//ROS_INFO("summit_controller_dspic::ProcessMsg: Converted params: d = %2.5f, r = %2.9f, l = %2.9f, crc (%d, %d)", d, r, l, received_crc, crc);
		//if(crc == received_crc){
		// update value (if data arrived OK)
		dsPic_data.dDWheels = d;
		dsPic_data.dRPMtoMPS = r;
		//}
		return OK_GET_PARAMETERS;
        }
	if(!strcmp(cReceivedTokens[0], GET_ENCODER)){
		ROS_INFO("summit_controller_dspic::ProcessMsg: Encoder: = %s, crc=%s", cReceivedTokens[1], cReceivedTokens[2]);
		enc = atoi(cReceivedTokens[1]);
		//received_crc = atoi(cReceivedTokens[2]);
		//memset(command, 0, 64);
		//sprintf(command,"%s,%d", GET_ENCODER, enc);
		//ROS_INFO(command);
		//crc = ComputeCRC(command, 64);
		//if(crc == received_crc){
		// Update values (if data arrived OK)
		dsPic_data.iEncoderAnt = dsPic_data.iEncoder;
		dsPic_data.iEncoder = enc;
		//}
		return OK_GET_ENCODER;
        }
	if(!strcmp(cReceivedTokens[0], GET_ENCODER_GYRO)){
		enc = atoi(cReceivedTokens[1]);
		th = atof(cReceivedTokens[2]);
                batt = atof(cReceivedTokens[3]);
		received_crc = atoi(cReceivedTokens[4]);
		memset(command, 0, 64);
		sprintf(command,"%s,%d,%2.5lf,%2.2lf", GET_ENCODER_GYRO, enc, th, batt);
		//ROS_INFO(command);
		crc = ComputeCRC(command, 64);
		// Si los datos han llegado correctamente
		if(crc == received_crc){
			// Actualizamos valor
			dsPic_data.iEncoderAnt = dsPic_data.iEncoder;
                        dsPic_data.iEncoder = enc;
                        dsPic_data.dGyro = th;
                        dsPic_data.dBatt = batt;
		}else {			
			ROS_ERROR("summit_controller_dspic::ProcessMsg: Get Encoder & Gyro: Error crc: rec = %d, calc = %d", received_crc, crc);
			return ERROR_GET_ENCODER_GYRO;
            }

		return OK_GET_ENCODER_GYRO;//
	}
	if(!strcmp(cReceivedTokens[0], CALIBRATE_OFFSET)){
		//ROS_INFO(" [%s,%s]", CALIBRATE_OFFSET,cReceivedTokens[1]);
		if(!strcmp(cReceivedTokens[1], "OK\r")){
			pthread_mutex_lock(&mutex_odometry);
				//dsPic_data.dGyro = dsPic_data.dOffsetGyro = 0.0;
				dsPic_data.dOffsetGyro = -dsPic_data.odometry_yaw;
				dsPic_data.dGyro = 0.0;
				bOffsetCalibration = true;
				bSentOffsetCalibration = false;
				bSuccessCalibration = true;
			pthread_mutex_unlock(&mutex_odometry);
			ROS_INFO("summit_controller_dspic::ProcessMsg: Calibrate offset OK");
		}else{
			ROS_ERROR("summit_controller_dspic::ProcessMsg: Error receiving calibrate offset confirmation");
			bSentOffsetCalibration = false;
			return ERROR_CALIBRATE_OFFSET;
            }
		return OK_CALIBRATE_OFFSET;
	}
	
    if(!strcmp(cReceivedTokens[0], SET_TRIMMER)){
		
		int channel = atoi(cReceivedTokens[1]);
		int offset1 = atoi(cReceivedTokens[2]);
        int center = atoi(cReceivedTokens[3]);
        int offset2 = atoi(cReceivedTokens[4]);
        // ROS_INFO("RECEIVED %d, %d, %d, %d", channel, offset1, center, offset2);
		// Check if data received matches data sent
		if ((channel >=0) and (channel <=3)) {
			if ((dsPic_data.offset1[channel] == offset1) && 
				(dsPic_data.center[channel] == center) &&
				(dsPic_data.offset2[channel] == offset2) )  
				return OK_SET_TRIMMER;
			else 
				return ERROR_SET_TRIMMER;
			}
		else return ERROR_SET_TRIMMER;
	}
	
	return OK;
}

////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS

/*!     \fn void summit_controller_dspic::ToggleMotorPower(unsigned char val)
 *      \brief Switches on/off the motor
*/
void summit_controller_dspic::ToggleMotorPower(unsigned char val)
{
        switch(val)     {

        case '1':       //Enable Motor
            this->dsPic_data.bMotorsEnabled=true;
            ROS_INFO("summit_controller_dspic::ToggleMotorPower: Motors enabled");
        break;
        case '0':       //Disable Motor
            this->dsPic_data.bMotorsEnabled=false;
            ROS_INFO("summit_controller_dspic::ToggleMotorPower: Motors disabled");
        break;
        }
}

/*!	\fn  void summit_controller_dspic::GetParameters(double *d, double *rpm2rps)
	* Gets the parameters for the odometry
*/
void summit_controller_dspic::GetParameters(double *d, double *rpm2rps){
	*d = dsPic_data.dDWheels;
	*rpm2rps = dsPic_data.dRPMtoMPS;
}

/*!	\fn int summit_controller_dspic::ModifyOdometry(double x, double y, double yaw)
	* Changes odometry's current values
*/
void summit_controller_dspic::ModifyOdometry(double x, double y, double yaw) {
	if(bLocalOdometry){
		pthread_mutex_lock(&mutex_odometry);
			dsPic_data.odometry_x = x;
			dsPic_data.odometry_y = y;
			dsPic_data.odometry_yaw = yaw;
			dsPic_data.dOffsetGyro = (dsPic_data.dGyro - yaw);	// Changes the offset to the current value
		pthread_mutex_unlock(&mutex_odometry);
	}else{
		dsPic_data_tmp.odometry_x = x;
		dsPic_data_tmp.odometry_y = y;
		dsPic_data_tmp.odometry_yaw = yaw;
		iCmd = DSPIC_CMD_MODIFY_ODOMETRY;
	}
}

/*!	\fn void summit_controller_dspic::InitOdometry()
	* Programs Init Odometry Message
*/
void summit_controller_dspic::InitOdometry()
{
	if(bLocalOdometry){
		pthread_mutex_lock(&mutex_odometry);
			dsPic_data.odometry_x = dsPic_data.odometry_y = dsPic_data.odometry_yaw = 0.0;
			dsPic_data.dOffsetGyro = dsPic_data.dGyro;
		pthread_mutex_unlock(&mutex_odometry);
	}else{
		iCmd = DSPIC_CMD_INIT_ODOMETRY;
	}
}

/*!	\fn void summit_controller_dspic::ResetOdometry()
	* Programs Reset Odometry Message
*/
void summit_controller_dspic::ResetOdometry()
{
	if(bLocalOdometry){
		pthread_mutex_lock(&mutex_odometry);
			dsPic_data.odometry_x = dsPic_data.odometry_y = dsPic_data.odometry_yaw = 0.0;
		pthread_mutex_unlock(&mutex_odometry);
	}else{
		iCmd = DSPIC_CMD_INIT_ODOMETRY;
	}
}

/*!	\fn void summit_controller_dspic::SetDWheels()
	* Programs Set DWHEELS Message
*/
void summit_controller_dspic::SetDWheels(double distance)
{
	if(bLocalOdometry){
		dsPic_data.dDWheels = distance;
	}else{
		dsPic_data_tmp.dDWheels = distance;
		iCmd = DSPIC_CMD_SET_D;
	}
}

/*!	\fn void summit_controller_dspic::SetRPMtoMPS()
	* Programs Set R Message
*/
void summit_controller_dspic::SetRPMtoMPS(double value)
{
	if(bLocalOdometry){
		dsPic_data.dRPMtoMPS = value;
	}else{
		dsPic_data_tmp.dRPMtoMPS = value;
		iCmd = DSPIC_CMD_SET_R;
	}
}

/*!	\fn void summit_controller_dspic::ReadOdometry()
	* Programs SendReadOdometry Message
*/
void summit_controller_dspic::ReadOdometry()
{
	iCmd = DSPIC_CMD_READ_ODOMETRY;
}

/*!	\fn void summit_controller_dspic::ResetController()
	* Programs SendResetController Message
*/
void summit_controller_dspic::ResetController()
{
	iCmd = DSPIC_CMD_RESET_CONTROLLER;
}

/*!	\fn void summit_controller_dspic::ReadParameters()
	* Programs SendReadParameters Message
*/
void summit_controller_dspic::ReadParameters()
{
	iCmd = DSPIC_CMD_READ_PARAMETERS;
}

/*!	\fn void summit_controller_dspic::ReadEncoder()
	* Programs SendReadEncoders Message
*/
void summit_controller_dspic::ReadEncoder()
{
	iCmd = DSPIC_CMD_READ_ENCODER;
}

/*!	\fn void summit_controller_dspic::ReadEncoderGyro()
	* Programs SendReadEncoderGyro Message
*/
void summit_controller_dspic::ReadEncoderGyro()
{
	iCmd = DSPIC_CMD_READ_ENCODER_GYRO;
}

/*!	\fn void summit_controller_dspic::CalibrateOffsetGyro()
	* Programs SendCalibrateOffsetGyro Message
*/
void summit_controller_dspic::CalibrateOffsetGyro()
{
	iCmd = DSPIC_CMD_CALIBRATE_OFFSET;
}

/*!	\fn void summit_controller_dspic::SetMotorReferences(double rads, double mps)
 * 	\brief Sets motor references 1-position in rad, 2-speed in mps
*/
void summit_controller_dspic::SetMotorReferences(double rads, double mps)
{
    // Control speed and angle limits
    if((rads < 0.0) && (rads < -MOTOR_DEF_MAX_ANGLE) )
 	rads = -MOTOR_DEF_MAX_ANGLE;
    else if((rads > 0.0) && (rads> MOTOR_DEF_MAX_ANGLE))
	rads = MOTOR_DEF_MAX_ANGLE;

    if((mps < 0.0) && (mps < -(MOTOR_DEF_MAX_SPEED * MOTOR_PERCENT_BWD) ))
	mps = -MOTOR_DEF_MAX_SPEED * MOTOR_PERCENT_BWD;
    else if( (mps> 0.0) && (mps > MOTOR_DEF_MAX_SPEED) )
	mps = MOTOR_DEF_MAX_SPEED;

    ROS_INFO("summit_controller_dspic::SetMotorReferences: v=%2.3f a=%2.3f", rads, mps);

    // Set references and command for next iteration
    dsPic_data.rad = rads;
    dsPic_data_tmp.rad = rads;
    dsPic_data.mps = mps;
    dsPic_data_tmp.mps = mps;
    iCmd = DSPIC_CMD_SET_MOTOR_REFERENCES;
}

/*!	\fn void summit_controller_dspic::SetTrimmer(int channel, int offset1, int center, int offset2)
 * 	\brief Sets the trimmer parameters
*/
void summit_controller_dspic::SetTrimmer(int channel, int offset1, int center, int offset2)
{
	// Check channel
	if ((channel<0)||(channel>3)) return;
	dsPic_data.last_channel = channel;
	dsPic_data.offset1[channel] = offset1;
	dsPic_data.center[channel] = center;
	dsPic_data.offset2[channel] = offset2; 
	
	iCmd = DSPIC_CMD_SET_TRIMMER;
}

/*!	\fn void summit_controller_dspic::GetEncoderValue(int16_t *enc)
	* Gets the value of the encoder
*/
void summit_controller_dspic::GetEncoderValue(int16_t *enc){
	*enc = dsPic_data.iEncoder;
}

/*!	\fn void summit_controller_dspic::GetEncoderGyroValue(int16_t *enc)
	* Gets the value of the encoder
*/
void summit_controller_dspic::GetEncoderGyroValue(int16_t *enc, double *gyro){
	*enc = dsPic_data.iEncoder;
	*gyro = dsPic_data.dGyro;
}

/*!	\fn float summit_controller_dspic::GetVoltage()
	* Gets the power voltage value
*/
float summit_controller_dspic::GetVoltage(){
	return (float) dsPic_data.dBatt;
}

/*!	\fn bool summit_controller_dspic::IsGyroCalibrated()
	* True if the gyroscope's offset has been calibrated
*/
bool summit_controller_dspic::IsGyroCalibrated(){
	if(bOffsetCalibration){
		bOffsetCalibration = false;
		return true;
	}else
		return false;

}

/*!	\fn void summit_controller_dspic::GetGyroValues(double *angle, double *offset)
	*
*/
void summit_controller_dspic::GetGyroValues(double *angle, double *offset){
	if( angle == NULL || offset == NULL)
		return;
	*angle = dsPic_data.dGyro;
	*offset = dsPic_data.dOffsetGyro;
}

/*!	\fn void summit_controller_dspic::GetMotorReferences(double *rad, double *mps)
	* Returns the value of current references
*/
void summit_controller_dspic::GetMotorReferences(double *rad, double *mps){
	*rad = dsPic_data.rad;
	*mps = dsPic_data.mps;
}

/*!     \fn double summit_controller_dspic::GetSamplingPeriod()
 *      \brief Returns accurate measured sampling period
 *      \return sample period in seconds
*/
double summit_controller_dspic::GetSamplingPeriod() {
    static bool first = true;
    static ros::Time now, last;
    double secs=0.0;
    
    now = ros::Time::now();    

    if(first) {
        first = false;
        last= now;
        return -1.0;
        }

    secs = (now - last).toSec();
    last= now;
    return secs;
}

/*!	\fn void summit_controller_dspic::UpdateOdometry()
	* Updates robot's odometry
	* This can be done into the dsPic controller or in this component
	* depending on the selected mode.
*/
void summit_controller_dspic::UpdateOdometry(){

    // Real sampling period
    double fSamplePeriod = GetSamplingPeriod();             // sampling period [s]

    // First iteration
    if(fSamplePeriod < 0.0) return;

    // Odometry calculation (using motor speed and position of the motor direction)
    double fBetaRads = dsPic_data.rad;  // current orientation angle (for ackerman odometry)

    int16_t ds = dsPic_data.iEncoder - dsPic_data.iEncoderAnt;
    double rpm = (((double) -ds / fCountsPerRev) / fSamplePeriod ) * 60.0;
    double v_mps = rpm * dsPic_data.dRPMtoMPS;

    // This function can be called at higher freq than readings, so avoid integrate twice
    dsPic_data.iEncoderAnt = dsPic_data.iEncoder; 

    // Compute Odometry
    double d = dsPic_data.dDWheels; // distance between center of back axis to front axis

    // Updates values
    pthread_mutex_lock(&mutex_odometry);

	// Orientation
        if(amMode == ODOMETRY){	// Takes robot angle from odometry
            dsPic_data.vel_yaw = (v_mps / d) * sin(fBetaRads);
            dsPic_data.odometry_yaw += dsPic_data.vel_yaw * fSamplePeriod;
            }
        else {  // Takes robot angle from gyro
            double pa = (dsPic_data.dGyro - dsPic_data.dOffsetGyro);	// 180.0 * DSPIC_PI;
            dsPic_data.vel_yaw = (dsPic_data.odometry_yaw - pa) / fSamplePeriod;
	    dsPic_data.odometry_yaw = pa;	    
            }

        // Normalize
	dsPic_data.odometry_yaw -= 2.0 * DSPIC_PI;
	while (dsPic_data.odometry_yaw >= (DSPIC_PI))
	while (dsPic_data.odometry_yaw <= (-DSPIC_PI))
	dsPic_data.odometry_yaw += 2.0 * DSPIC_PI;

        // Velocities
        dsPic_data.vel_x = v_mps * cos(fBetaRads) * cos(dsPic_data.odometry_yaw);
        dsPic_data.vel_y = v_mps * cos(fBetaRads) * sin(dsPic_data.odometry_yaw);

        // Positions
        dsPic_data.odometry_x += dsPic_data.vel_x * fSamplePeriod;
        dsPic_data.odometry_y += dsPic_data.vel_y * fSamplePeriod;

   pthread_mutex_unlock(&mutex_odometry);
}

/*!     \fn void summit_controller_dspic::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
        * Callback - velocity references 
*/
void summit_controller_dspic::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
//void summit_controller_dspic::commandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
//\fn void summit_controller_dspic::cmdVelCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{
    //SetMotorReferences(double rads, double mps)
    this->SetMotorReferences(cmd_vel->angular.z, cmd_vel->linear.x);
    //this->SetMotorReferences(msg->drive.steering_angle, msg->drive.speed);
    
    //ROS_INFO("summit_controller_dspic::commandCallback speed=%5.2f  angle=%5.2f", msg->drive.speed, msg->drive.steering_angle);
}

/*!     \fn void summit_controller_dspic::joystickCallback(const sensor_msgs::JoyConstPtr& msg)
        * Callback - Joystick buttons - Define control mode (kinematic configuration)
*/
void summit_controller_dspic::joystickCallback(const sensor_msgs::JoyConstPtr& msg)
{

//	static bool bRecoveryButtons[4] = {false, false, false, false}; // Need to press first 4 buttons to recovery
//	for (int i=0; i<4; i++) 
//          if ( msg->buttons[i] == 1 ) bRecoveryButtons[i] = true;	
    	  
   if ( msg->buttons[0] == 1) {
//        if (!dual_mode_) dual_mode_= true;
//        else dual_mode_ = false;
          ROS_INFO("SET MOTOR POWER ON");
          this->ToggleMotorPower( '1' );
          }
   if ( msg->buttons[1] == 1)  {
//          if (!paralel_mode_) paralel_mode_ = true;
//          else paralel_mode_ = false;
          ROS_INFO("SET MOTOR POWER OFF");
          this->ToggleMotorPower( '0' );
          }


/*
        // Checks the communication status
        ros::Time current_time = ros::Time::now();
        if ( (current_time - tDsPicReply).toSec() > DSPIC_TIMEOUT_COMM) {
                ROS_ERROR("summit_controller_dspic::Standby: Timeout in communication with the device");
                tDsPicReply = ros::Time::now();  // Resets the timer
        }
*/

}

/*!     \fn  Service set odometry
  *      Sets the odometry of the robot
*/
bool summit_controller_dspic::set_odometry(robotnik_msgs::set_odometry::Request &req, robotnik_msgs::set_odometry::Response &res )
{
        ROS_INFO("summit_xl_controller::set_odometry: request -> x = %f, y = %f, a = %f", req.x, req.y, req.orientation);

        ModifyOdometry(req.x, req.y, req.orientation);
        res.ret = true;

        return true;
}

/*!     \fn  Service SetMode
  *      Sets the kinematic mode
*/
bool summit_controller_dspic::set_mode(robotnik_msgs::set_mode::Request& request, robotnik_msgs::set_mode::Response& response)
{
  // 1 - SINGLE ACKERMANN
  // 2 - DUAL ACKERMANN INVERTED forward and backward axes turn opposite
  // 3 - DUAL ACKERMANN GEARED forward and backward axes turn simetric
  if (request.mode == SINGLE_ACKERMANN) {
	active_kinematic_mode_ = request.mode;
	ROS_INFO("summit_controller_dspic:  change kinematic mode to SINGLE_ACKERMANN");
	return true;
	}
  if (request.mode == DUAL_ACKERMANN_INVERTED) {
	active_kinematic_mode_ = request.mode;
	ROS_INFO("summit_controller_dspic:  change kinematic mode to DUAL_ACKERMANN_INVERTED");
	return true;	
	}
  if (request.mode == DUAL_ACKERMANN_GEARED) {
	active_kinematic_mode_ = request.mode;
	ROS_INFO("summit_controller_dspic:  change kinematic mode to DUAL_ACKERMANN_GEARED");
	return true;	
	}

  return false;
}

// Callbacks
std::string prefixTopic(std::string prefix, char * name)
{
        std::string topic_name = prefix;
        topic_name.append(name);
        return topic_name;
}

// MAIN
int main(int argc, char** argv)
{
	ros::init(argc, argv, "summit_controller_dspic_node");
	//ROS_INFO("Summit controller for ROS %.2f", NODE_VERSION);	
	int time_remaining = -1;
	
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	pn.param<std::string>("port", port, "/dev/ttyUSB0");
	ROS_INFO("summit_controller_dspic - port=%s", port.c_str() );
	
	std::string base_frame_id;
	std::string odom_frame_id;
	pn.param<std::string>("base_frame_id", base_frame_id, "base_footprint");
	pn.param<std::string>("odom_frame_id", odom_frame_id, "odom");

	// Create node object
	summit_controller = new summit_controller_dspic( port );
	
	// Publishing
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	ros::Publisher battery_pub = n.advertise<std_msgs::Float32>("/battery", 5);
	ros::Publisher joint_state_pub_;

	// Joint states
    sensor_msgs::JointState robot_joints_;
    
	// Initialize joints array 
    char j[3]= "\0";  
	for(int i = 0; i <SUMMIT_JOINTS; i++){
		   sprintf(j,"j%d",i+1);
		   robot_joints_.name.push_back(j);
		   robot_joints_.position.push_back(0.0);
		   robot_joints_.velocity.push_back(0.0);
		   robot_joints_.effort.push_back(0.0);	
	       }

	// Names of the joints
	robot_joints_.name[0] = "joint_back_left_wheel_dir";
	robot_joints_.name[1] = "joint_back_left_wheel";
	
	robot_joints_.name[2] = "joint_back_right_wheel_dir";
	robot_joints_.name[3] = "joint_back_right_wheel";
	
	robot_joints_.name[4] = "joint_front_left_wheel_dir";
	robot_joints_.name[5] = "joint_front_left_wheel";
	
	robot_joints_.name[6] = "joint_front_right_wheel_dir";
	robot_joints_.name[7] = "joint_front_right_wheel";
	
	robot_joints_.name[8] = "joint_camera_pan";
	robot_joints_.name[9] = "joint_camera_tilt";

        
	tf::TransformBroadcaster tf_broadcaster;
	// Subcscribing
    ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &summit_controller_dspic::cmdVelCallback, summit_controller);
	
	// Subscribe to command topic
    //ros::Subscriber cmd_sub_ = n.subscribe<ackermann_msgs::AckermannDriveStamped>("/summit/command", 1, &summit_controller_dspic::commandCallback, summit_controller);
		
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("/joy", 1, &summit_controller_dspic::joystickCallback, summit_controller);

    // Service set_odometry
    ros::ServiceServer service_odom = n.advertiseService("set_odometry", &summit_controller_dspic::set_odometry, summit_controller);
    ros::ServiceServer service_mode = n.advertiseService("/summit_controller_dspic/set_mode", &summit_controller_dspic::set_mode, summit_controller);
	
	if( summit_controller->Open()==OK ) ROS_INFO("Connected to Summit Controller Dspic.");
	else
	{
		ROS_FATAL("Could not connect to Summit Controller Dspic.");
	//	ROS_BREAK();
	}
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(40.0);
	while(n.ok())
	{
		current_time = ros::Time::now();		
		//dt = (current_time - last_time).toSec();

		// 1 State Machine
		summit_controller->StateMachine();		


		// 2 Publish
		// ******************************************************************************************
		//first, we'll publish the transforms over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = odom_frame_id;
		odom_trans.child_frame_id = base_frame_id;
		odom_trans.transform.translation.x = summit_controller->dsPic_data.odometry_x;   
		odom_trans.transform.translation.y = summit_controller->dsPic_data.odometry_y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(summit_controller->dsPic_data.odometry_yaw);
		tf_broadcaster.sendTransform(odom_trans);
		
		// ******************************************************************************************
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = odom_frame_id;
		
		//set the position
		odom.pose.pose.position.x = summit_controller->dsPic_data.odometry_x;
		odom.pose.pose.position.y = summit_controller->dsPic_data.odometry_y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(summit_controller->dsPic_data.odometry_yaw);
		
		//set the velocity
		odom.child_frame_id = base_frame_id;
		odom.twist.twist.linear.x = summit_controller->dsPic_data.vel_x;
		odom.twist.twist.linear.y = summit_controller->dsPic_data.vel_y;
		odom.twist.twist.angular.z = summit_controller->dsPic_data.vel_yaw;
		
		//publish the message
		odom_pub.publish(odom);
		//ROS_INFO("Odometria -> x = %f, y = %f", odom.pose.pose.position.x, odom.pose.pose.position.y);

		// ******************************************************************************************
		//publish battery
                std_msgs::Float32 battery;
                battery.data = (float) summit_controller->dsPic_data.dBatt;
		battery_pub.publish( battery );
	
		ros::spinOnce();
		r.sleep();
	}
	
	summit_controller->Close();
}
// EOF





