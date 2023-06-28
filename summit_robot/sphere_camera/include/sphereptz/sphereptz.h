#ifndef SPHEREPTZ_H_
#define SPHEREPTZ_H_

/////////////////////////////////////
// Std Includes
/////////////////////////////////////
#include <fcntl.h>
#include <linux/videodev2.h>
#include <math.h>
#include <stddef.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <sstream>
#include <string>

/////////////////////////////////////
// ROS includes
/////////////////////////////////////
#include <ros/ros.h>

/////////////////////////////////////
// Sphere Messages
/////////////////////////////////////
//#include <rcf_com_spherePTZ/PTZ.h>
/////////////////////////////////////
// Sphere Drive
/////////////////////////////////////
//#include <rcf_com_spherePTZ/spherePTZ_com.h>
#include <geometry_msgs/Twist.h>  // cmd_vel
#include <sensor_msgs/Joy.h>  // joystick

#include "sphereptz/v4l2_controls.h"
#include "sphereptz/v4l2_dyna_ctrls.h"
//#include <sphereptz/ptz_state.h>
#include <robotnik_msgs/ptz.h>

#define LENGTH_OF_XU_CTR 6
#define LENGTH_OF_XU_MAP 10

#define MAX_PAN_VALUE 37
#define MAX_TILT_VALUE 20
#define DEFAULT_PANTILT_INC 1.0
/////////////////////////////////////
// Class definition
/////////////////////////////////////

namespace sphereptz
{
class SpherePTZ
{
   private:
    std::string sphereDevPort_;

    int                           autoreset;
    int                           fd;
    int                           currentX;
    int                           currentY;
    int                           desiredX;
    int                           desiredY;
    struct uvc_xu_control_info    xu_ctrls[LENGTH_OF_XU_CTR];
    struct uvc_xu_control_mapping xu_mappings[LENGTH_OF_XU_MAP];
    int                           reset_pan;
    int                           reset_tilt;
    ros::NodeHandle               n;
    ros::Publisher                ptz_state_pub;
    ros::Subscriber               joy_sub;
    ros::Subscriber               ptz_cmd_sub;

   public:
    SpherePTZ();
    ~SpherePTZ();

    int xioctl(int fd, int IOCTL_X, void* arg);

    //! Callback - command references
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
    //! Callback - Joystick buttons - Define control mode (kinematic
    //! configuration)
    void joystickCallback(const sensor_msgs::JoyConstPtr& msg);
    //! Callback - Joystick buttons - Define control mode (kinematic
    //! configuration)
    void cmdPtzCallback(const robotnik_msgs::ptz::ConstPtr& msg);

    //! Do node initializing.
    void initNode();

    //! Shut the node down properly
    void shutdownNode();

    //! Setup
    void Setup();

    //! Main loop function for node.
    void mainNodeLoop();

    /////////////////////////////////////
    // Node Commands
    /////////////////////////////////////
    // the node command index
    // ResetPT = 0,
    // ResetPan = 1,
    // ResetTilt = 2,
    // PanLeft = 3,
    // PanRight = 4,
    // TiltUp = 5,
    // TiltDown = 6
};

}  // namespace sphereptz

#endif /* SPHEREPTZ_H_ */
