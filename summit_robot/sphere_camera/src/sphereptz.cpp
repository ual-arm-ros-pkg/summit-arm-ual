/*
 * sphereptz
 * Adapted from 
 *    Player - One Hell of a Robot Server
 *    Copyright (C) 2000  Brian Gerkey et al.
 * by Robotnik Automation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Roberto Guzman (rguzman@robotnik.es)
 * \brief Sphere PTZ driver
 */

/////////////////////////////////////
// Std Includes
/////////////////////////////////////
#include <string>
#include <iostream>
#include <sstream>
#include <stdlib.h>

/////////////////////////////////////
// ROS includes
/////////////////////////////////////
#include <ros/ros.h>

/////////////////////////////////////
// Driver
/////////////////////////////////////

#include "sphereptz/sphereptz.h"
#include <robotnik_msgs/ptz.h>

using namespace sphereptz;

#ifndef M_PI
#define M_PI        3.14159265358979323846
#endif

#ifndef DTOR
#define DTOR(d) ((d) * (M_PI) / 180.0)
#endif

#ifndef RTOD
#define RTOD(r) ((static_cast<double>(r)) * 180.0 / M_PI)
#endif

#ifndef VIDIOC_S_EXT_CTRLS

struct v4l2_ext_control
{
  __u32 id;
  __u32 reserved2[2];
  union
  {
    __s32 value;
    __s64 value64;
    void * reserved;
  };
} __attribute__ ((packed));

struct v4l2_ext_controls
{
  __u32 ctrl_class;
  __u32 count;
  __u32 error_idx;
  __u32 reserved[2];
  struct v4l2_ext_control * controls;
};

#define VIDIOC_S_EXT_CTRLS _IOWR('V', 72, struct v4l2_ext_controls)

#endif

#define LENGTH_OF_XU_CTR 6
#define LENGTH_OF_XU_MAP 10
// set ioctl retries to 4 - linux uvc as increased timeout from 1000 to 3000 ms
#define IOCTL_RETRY 4

SpherePTZ::SpherePTZ() {
	
}

SpherePTZ::~SpherePTZ() {
}

/* ioctl with a number of retries in the case of failure
* args:
* fd - device descriptor
* IOCTL_X - ioctl reference
* arg - pointer to ioctl data
* returns - ioctl result
*/
int SpherePTZ::xioctl(int fd, int IOCTL_X, void *arg){
        int ret = 0;
        int tries= IOCTL_RETRY;

        ret = ioctl(fd, IOCTL_X, arg);
        if( ret && (errno == EINTR || errno == EAGAIN || errno == ETIMEDOUT))
        {
                // I/O error RETRY
                while(tries-- &&
                        (ret = ioctl(fd, IOCTL_X, arg)) &&
                        (errno == EINTR || errno == EAGAIN || errno == ETIMEDOUT))
                {
                        //g_printerr("ioctl (%i) failed - %s :(retry %i)\n", IOCTL_X, strerror(errno), tries);
                }
                if (ret && (tries <= 0))
            ROS_ERROR("SpherePTZ::xioctl (%i) retried %i times - giving up: %s)", IOCTL_X, IOCTL_RETRY, strerror(errno));
        }
        return (ret);
}


/*!     \fn void SpherePTZ::initNode()
        * Init Node  
*/
void SpherePTZ::initNode() {

	ROS_DEBUG("SpherePTZ::initNode");

	// debug settings
	ROS_DEBUG("Initializing spherePTZ...");

	/////////////////////////////////////
	// PARAMETERS
	/////////////////////////////////////
	ros::NodeHandle _privateNH("~");
	// set the device port
	n.param("dev_video", sphereDevPort_, sphereDevPort_);

  	this->autoreset = 0;
  	this->fd = -1;
  	this->currentX = 0;
  	this->currentY = 0;
  	this->desiredX = 0;
  	this->desiredY = 0;
  	memset(this->xu_ctrls, 0, sizeof this->xu_ctrls);
  	memset(this->xu_mappings, 0, sizeof this->xu_mappings);

	this->autoreset = 1;	// yaml ?

	ROS_INFO("SpherePTZ::initNode : Attempting to open connection to camera on port %s.",
			sphereDevPort_.c_str());
	
	ptz_state_pub = n.advertise<robotnik_msgs::ptz>("/sphereptz/ptz_state", 1000);

	// suscribe to 
	// ptz_cmd_sub = n.subscribe<sphereptz::ptz_state>("/sphereptz/command_ptz", 10, &SpherePTZ::cmdPtzCallback, this);
        ptz_cmd_sub = n.subscribe<robotnik_msgs::ptz>("/sphereptz/command_ptz", 10, &SpherePTZ::cmdPtzCallback, this);

	this->Setup();
}

/*!     \fn void SpherePTZ::shutdownNode()
        * Close files 
*/
void SpherePTZ::shutdownNode() {

	if (this->fd >= 0) close(this->fd);
}

/*!     \fn void SpherePTZ::Setup()
        * Configure v4l2 device
*/
void SpherePTZ::Setup()
{

	struct v4l2_capability cap;
	struct v4l2_ext_control xctrls[1];
	struct v4l2_ext_controls ctrls;
	int i;

	const struct uvc_xu_control_info _xu_ctrls[LENGTH_OF_XU_CTR] =
	{
	{ UVC_GUID_LOGITECH_MOTOR_CONTROL, 0, XU_MOTORCONTROL_PANTILT_RELATIVE, 4, UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | UVC_CONTROL_GET_DEF },
	{ UVC_GUID_LOGITECH_MOTOR_CONTROL, 1, XU_MOTORCONTROL_PANTILT_RESET, 1, UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | UVC_CONTROL_GET_RES | UVC_CONTROL_GET_DEF },
	{ UVC_GUID_LOGITECH_MOTOR_CONTROL, 2, XU_MOTORCONTROL_FOCUS, 6, UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX |UVC_CONTROL_GET_DEF },
	{ UVC_GUID_LOGITECH_VIDEO_PIPE, 4, XU_COLOR_PROCESSING_DISABLE, 1, UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR |UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | UVC_CONTROL_GET_RES | UVC_CONTROL_GET_DEF },
	{ UVC_GUID_LOGITECH_VIDEO_PIPE, 7, XU_RAW_DATA_BITS_PER_PIXEL, 1, UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR |UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | UVC_CONTROL_GET_RES | UVC_CONTROL_GET_DEF },
	{ UVC_GUID_LOGITECH_USER_HW_CONTROL, 0, XU_HW_CONTROL_LED1, 3, UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR |UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | UVC_CONTROL_GET_RES | UVC_CONTROL_GET_DEF }
	};
	const struct uvc_xu_control_mapping _xu_mappings[LENGTH_OF_XU_MAP] =
	{
	{ V4L2_CID_PAN_RELATIVE_NEW, "Pan (relative)", UVC_GUID_LOGITECH_MOTOR_CONTROL, XU_MOTORCONTROL_PANTILT_RELATIVE, 16, 0, V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_SIGNED },
	{ V4L2_CID_TILT_RELATIVE_NEW, "Tilt (relative)", UVC_GUID_LOGITECH_MOTOR_CONTROL, XU_MOTORCONTROL_PANTILT_RELATIVE, 16, 16, V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_SIGNED },
	{ V4L2_CID_PAN_RESET_NEW, "Pan Reset", UVC_GUID_LOGITECH_MOTOR_CONTROL, XU_MOTORCONTROL_PANTILT_RESET, 1, 0, V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_UNSIGNED },
	{ V4L2_CID_TILT_RESET_NEW, "Tilt Reset", UVC_GUID_LOGITECH_MOTOR_CONTROL, XU_MOTORCONTROL_PANTILT_RESET, 1, 1, V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_UNSIGNED },
	{ V4L2_CID_PANTILT_RESET_LOGITECH, "Pan/tilt Reset", UVC_GUID_LOGITECH_MOTOR_CONTROL, XU_MOTORCONTROL_PANTILT_RESET, 8, 0, V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_UNSIGNED },
	{ V4L2_CID_FOCUS_LOGITECH, "Focus (absolute)", UVC_GUID_LOGITECH_MOTOR_CONTROL, XU_MOTORCONTROL_FOCUS, 8, 0, V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_UNSIGNED },
	{ V4L2_CID_LED1_MODE_LOGITECH, "LED1 Mode", UVC_GUID_LOGITECH_USER_HW_CONTROL, XU_HW_CONTROL_LED1, 8, 0, V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_UNSIGNED },
	{ V4L2_CID_LED1_FREQUENCY_LOGITECH, "LED1 Frequency", UVC_GUID_LOGITECH_USER_HW_CONTROL, XU_HW_CONTROL_LED1, 8, 16, V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_UNSIGNED },
	{ V4L2_CID_DISABLE_PROCESSING_LOGITECH, "Disable video processing", UVC_GUID_LOGITECH_VIDEO_PIPE, XU_COLOR_PROCESSING_DISABLE, 8, 0, V4L2_CTRL_TYPE_BOOLEAN, UVC_CTRL_DATA_TYPE_BOOLEAN },
	{ V4L2_CID_RAW_BITS_PER_PIXEL_LOGITECH, "Raw bits per pixel", UVC_GUID_LOGITECH_VIDEO_PIPE, XU_RAW_DATA_BITS_PER_PIXEL, 8, 0, V4L2_CTRL_TYPE_INTEGER, UVC_CTRL_DATA_TYPE_UNSIGNED }
	};


	this->currentX = 0;
	this->currentY = 0;
	this->desiredX = 0;
	this->desiredY = 0;
	// reset flags
	this->reset_pan = 0;
	this->reset_tilt = 0;

	ROS_DEBUG("SpherePTZ::Setup : opening %s", sphereDevPort_.c_str());
	this->fd = open(sphereDevPort_.c_str(), O_RDWR);
	if (this->fd < 0)
	{
		ROS_ERROR("Cannot open %s", sphereDevPort_.c_str());
		ROS_BREAK();
	}
	memset(&cap, 0, sizeof cap);
	if (xioctl(this->fd, VIDIOC_QUERYCAP, &cap) == -1)
	{
		ROS_ERROR("VIDIOC_QUERYCAP failed");
		close(this->fd);
		this->fd = -1;
		ROS_BREAK();
	}
	if (!(cap.capabilities & V4L2_CAP_STREAMING))
	{
		ROS_WARN("V4L2_CAP_READWRITE check failed (ignored)");
	}
	if (!(cap.capabilities & V4L2_CAP_READWRITE))
	{
		ROS_WARN("V4L2_CAP_READWRITE check failed (ignored)");
	}
	for (i = 0; i < LENGTH_OF_XU_CTR; i++) this->xu_ctrls[i] = _xu_ctrls[i];
	for (i = 0; i < LENGTH_OF_XU_MAP; i++) this->xu_mappings[i] = _xu_mappings[i];
	for (i = 0; i < LENGTH_OF_XU_CTR; i++)
	{
		ROS_WARN("Adding control for [%s]", this->xu_mappings[i].name);
		if (xioctl(this->fd, UVCIOC_CTRL_ADD, &(this->xu_ctrls[i]))
== -1) {
          if (errno!=EEXIST) ROS_ERROR("SpherePTZ: UVCIOC_CTRL_ADD - Error");  
          }
	}
	for (i = 0; i < LENGTH_OF_XU_MAP; i++)
	{
		ROS_WARN("Mapping control for [%s]", this->xu_mappings[i].name);
		if (xioctl(this->fd, UVCIOC_CTRL_MAP, &(this->xu_mappings[i])) == -1) {
          if (errno!=EEXIST) ROS_ERROR("SpherePTZ: UVCIOC_CTRL_MAP - Error");
          }
	}
/*
	memset(&ctrls, 0, sizeof ctrls);
	memset(&xctrls, 0, sizeof xctrls);
	xctrls[0].id = V4L2_CID_PAN_RESET_NEW;
	xctrls[0].value = 1;
	ctrls.count = 1;
	ctrls.controls = xctrls;
	if (xioctl(this->fd, VIDIOC_S_EXT_CTRLS, &ctrls) == -1)
	{
		ROS_ERROR("VIDIOC_S_EXT_CTRLS failed on V4L2_CID_PAN_RESET_NEW");
	}
	sleep(4);
	memset(&ctrls, 0, sizeof ctrls);
	memset(&xctrls, 0, sizeof xctrls);
	xctrls[0].id = V4L2_CID_TILT_RESET_NEW;
	xctrls[0].value = 1;
	ctrls.count = 1;
	ctrls.controls = xctrls;
	if (xioctl(this->fd, VIDIOC_S_EXT_CTRLS, &ctrls) == -1)
	{    
		ROS_ERROR("VIDIOC_S_EXT_CTRLS failed on V4L2_CID_TILT_RESET_NEW");
	}
  */
 // sleep(2);	  
}

/*! \fn void SpherePTZ::cmdPtzCallback(const ptz_state& msg)
	* Callback - Joystick buttons - Activate / Deactivate mode + Resets
*/
void SpherePTZ::cmdPtzCallback(const robotnik_msgs::ptz::ConstPtr& msg)
{
	//ROS_ERROR("SpherePTZ::cmdPtzCallback: pan = %f, tilt = %f, zoom = %f", msg->pan, msg->tilt, msg->zoom);
	double inc_pan = 0, inc_tilt = 0;

        // discard "relative" field - this camera works only in relative coords

	// PAN 
	if(msg->pan > 0.0){
		inc_pan = -DEFAULT_PANTILT_INC;
	}else if(msg->pan < 0.0){
		inc_pan = DEFAULT_PANTILT_INC;
	}
	// TILT
	if(msg->tilt > 0.0){
		inc_tilt = DEFAULT_PANTILT_INC;
	}else if(msg->tilt < 0.0){
		inc_tilt = -DEFAULT_PANTILT_INC;
	}

	this->desiredX+= (int)inc_pan;
	this->desiredY+= (int)inc_tilt;
	
	if(this->desiredX > MAX_PAN_VALUE){
		this->desiredX = MAX_PAN_VALUE;
	}else if(this->desiredX < -MAX_PAN_VALUE){
		this->desiredX = -MAX_PAN_VALUE;
	}
	
	if(this->desiredY > MAX_TILT_VALUE){
		this->desiredY = MAX_TILT_VALUE;
	}else if(this->desiredY < -MAX_TILT_VALUE){
		this->desiredY = -MAX_TILT_VALUE;
	}
	//ROS_ERROR("SpherePTZ::cmdPtzCallback: desiredX = %d, desiredY = %d", desiredX, desiredY);
}

/*!     \fn void SpherePTZ::joystickCallback(const sensor_msgs::JoyConstPtr& msg)
        * Callback - Joystick buttons - Activate / Deactivate mode + Resets
*/
void SpherePTZ::joystickCallback(const sensor_msgs::JoyConstPtr& msg)
{
	//ROS_ERROR("RECEIVED");
	// Move pan and tilt
	/*if ( msg->axes[4] > 0) this->desiredX += 2; 
	else if ( msg->axes[4] < 0) this->desiredX -= 2;
	if ( msg->axes[5] > 0) this->desiredY += 2;
	else if ( msg->axes[5] < 0) this->desiredY -= 2;

	if(this->desiredX > MAX_PAN_VALUE){
		this->desiredX = MAX_PAN_VALUE;
	}else if(this->desiredX < -MAX_PAN_VALUE){
		this->desiredX = -MAX_PAN_VALUE;
	}
	
	if(this->desiredY > MAX_TILT_VALUE){
		this->desiredY = MAX_TILT_VALUE;
	}else if(this->desiredY < -MAX_TILT_VALUE){
		this->desiredY = -MAX_TILT_VALUE;
	}

	// Reset pan and tilt
	if (( msg->buttons[4] == 1) && ( msg->buttons[6] == 1) ) {
		this->desiredX = 0;
		this->desiredY = 0;
	}*/
}

/*!     \fn void SpherePTZ::mainNodeLoop()
        * Main node loop
*/
void SpherePTZ::mainNodeLoop() {

	struct v4l2_ext_control xctrls[2];
	struct v4l2_ext_controls ctrls;
	robotnik_msgs::ptz ptz_camera_state;

	ros::Rate loop_rate(10);
	
	// main loop
	while (n.ok()) {
		
		
		if ((this->desiredX) > (this->currentX))
		{
			memset(xctrls, 0, sizeof xctrls);
			memset(&ctrls, 0, sizeof ctrls);
			xctrls[0].id = V4L2_CID_PAN_RELATIVE_NEW;
			xctrls[0].value = 128; 
			xctrls[1].id = V4L2_CID_TILT_RELATIVE_NEW;
			xctrls[1].value = 0;
			ctrls.count = 2;  // 2
			ctrls.controls = xctrls;
			if (xioctl(this->fd, VIDIOC_S_EXT_CTRLS, &ctrls) == -1)
			{
			  	ROS_ERROR("VIDIOC_S_EXT_CTRLS failed while panning right");
			} else
			{
			  	this->currentX++;
			  	if ((!(this->currentX)) && (!(this->currentY))) this->reset_pan = !0;
			}
		} else if ((this->desiredX) < (this->currentX))
		{
			memset(xctrls, 0, sizeof xctrls);
			memset(&ctrls, 0, sizeof ctrls);
			xctrls[0].id = V4L2_CID_PAN_RELATIVE_NEW;
			xctrls[0].value = -128; 
			xctrls[1].id = V4L2_CID_TILT_RELATIVE_NEW;
			xctrls[1].value = 0;
			ctrls.count = 2; // 2
			ctrls.controls = xctrls;
			if (xioctl(this->fd, VIDIOC_S_EXT_CTRLS, &ctrls) == -1)
			{
				ROS_ERROR("VIDIOC_S_EXT_CTRLS failed while panning left");
			} else
			{
				this->currentX--;
				if ((!(this->currentX)) && (!(this->currentY))) this->reset_pan = !0;
			}
		}

		if ((this->desiredY) > (this->currentY))
		{
			memset(xctrls, 0, sizeof xctrls);
			memset(&ctrls, 0, sizeof ctrls);
			xctrls[0].id = V4L2_CID_PAN_RELATIVE_NEW;
			xctrls[0].value = 0;
			xctrls[1].id = V4L2_CID_TILT_RELATIVE_NEW;
			xctrls[1].value = -128; 
			ctrls.count = 2;    
			ctrls.controls = xctrls;
			if (xioctl(this->fd, VIDIOC_S_EXT_CTRLS, &ctrls) == -1)
			{
				ROS_ERROR("VIDIOC_S_EXT_CTRLS failed while tilting up");
			} else
			{
			    	this->currentY++;
			    	if ((!(this->currentX)) && (!(this->currentY))) this->reset_tilt = !0;
			}
		} else if ((this->desiredY) < (this->currentY))
		{
			memset(xctrls, 0, sizeof xctrls);
			memset(&ctrls, 0, sizeof ctrls);
			xctrls[0].id = V4L2_CID_PAN_RELATIVE_NEW;
			xctrls[0].value = 0;
			xctrls[1].id = V4L2_CID_TILT_RELATIVE_NEW;
			xctrls[1].value = 128; 
			ctrls.count = 2; 
			ctrls.controls = xctrls;
			if (xioctl(this->fd, VIDIOC_S_EXT_CTRLS, &ctrls) == -1)
			{
				ROS_ERROR("VIDIOC_S_EXT_CTRLS failed while tilting down");
			} else
			{
				this->currentY--;
				if ((!(this->currentX)) && (!(this->currentY))) this->reset_tilt = !0;
			}
		}

		/*if ((!(this->currentX)) && (!(this->currentY)))
		{
			if ((this->reset_pan) && (this->autoreset))
			{
				this->reset_pan=0;
				memset(xctrls, 0, sizeof xctrls);
				memset(&ctrls, 0, sizeof ctrls);
				xctrls[0].id = V4L2_CID_PAN_RESET_NEW;
				xctrls[0].value = 1;
				ctrls.count = 1;
				ctrls.controls = xctrls;
				ioctl(this->fd, VIDIOC_S_EXT_CTRLS, &ctrls);
				sleep(2);
		  	}
			if ((this->reset_tilt) && (this->autoreset))
			{
				this->reset_tilt=0;
				memset(xctrls, 0, sizeof xctrls);
				memset(&ctrls, 0, sizeof ctrls);
				xctrls[0].id = V4L2_CID_TILT_RESET_NEW;
				xctrls[0].value = 1;
				ctrls.count = 1;
				ctrls.controls = xctrls;
				ioctl(this->fd, VIDIOC_S_EXT_CTRLS, &ctrls);
				sleep(2);
			}
		}*/

		// fill PTZ state msg			
		ptz_camera_state.pan = DTOR(this->currentX *2.5);  //DTOR(this->currentX);
		ptz_camera_state.tilt = DTOR(this->currentY *2.5); //DTOR(this->currentY);
		//ptz_camera_state.zoom = -1.0;
                //ptz_camera_state.panspeed = -1.0;
                //ptz_camera_state.tiltspeed = -1.0;
	
		// publish PTZ state message
		ptz_state_pub.publish(ptz_camera_state);

		// spin node once
		ros::spinOnce();
		// go to sleep if done before rate freq
		loop_rate.sleep();
	}

}

int main(int argc, char **argv) {

	// Initialize
	ros::init(argc, argv, "SpherePTZ");
	//ROS_ERROR("Initialized.");
 
	// node handle
	//ros::NodeHandle n;
	// Create a new instance of Authenticator
	SpherePTZ sphere;
	// Init	
	sphere.initNode();

	// Main Loop
	sphere.mainNodeLoop();

	// Shutdown
	ROS_INFO("Shutting Down SpherePTZ...");
	sphere.shutdownNode();
	ROS_INFO("Done.");
}


