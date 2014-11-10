/*
 * summit_odometry
 * Copyright (c) 2011, Robotnik Automation, SLL
 * All rights reserved.
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
 * \brief Computes the odometry of the robot using odoemtry and IMU (simple fusion). Publish to /odom
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_controller_interface/controller.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

#define PI 3.1415926535

// Wheel Speeds 
double vx = 0.0;
double vy = 0.0;

// Robot position
double robot_pose_px_;
double robot_pose_py_;
double robot_pose_pa_;

// Robot Speeds
double linearSpeedMps_;
double angularSpeedRads_;

// Joint velocities auxiliar vector
std::vector<double> vecVel;
std::vector<double> vecPos;

// IMU values
double ang_vel_x_ = 0.0;
double ang_vel_y_ = 0.0;
double ang_vel_z_ = 0.0;

double lin_acc_x_ = 0.0;
double lin_acc_y_ = 0.0;
double lin_acc_z_ = 0.0;

double orientation_x_ = 0.0;
double orientation_y_ = 0.0;
double orientation_z_ = 0.0;
double orientation_w_ = 0.0;


//////////////////////////////////////////////////////////////
//           CALLBACK TO RECEIVE THE IMU STATE              //      
//////////////////////////////////////////////////////////////
void imuCallback( const sensor_msgs::Imu& imu_msg){

	orientation_x_ = imu_msg.orientation.x;
	orientation_y_ = imu_msg.orientation.y;
	orientation_z_ = imu_msg.orientation.z;
	orientation_w_ = imu_msg.orientation.w;

	ang_vel_x_ = imu_msg.angular_velocity.x;
	ang_vel_y_ = imu_msg.angular_velocity.y;
	ang_vel_z_ = imu_msg.angular_velocity.z;

	lin_acc_x_ = imu_msg.linear_acceleration.x;
	lin_acc_y_ = imu_msg.linear_acceleration.y;
	lin_acc_z_ = imu_msg.linear_acceleration.z;
}

//////////////////////////////////////////////////////////////
//     CALLBACK TO RECEIVE THE ROBOT JOINTS STATES          //
//////////////////////////////////////////////////////////////
void jointStatesCallback( const sensor_msgs::JointState& joint_states) {

  vecPos = joint_states.position;
  vecVel = joint_states.velocity;
  //vecEff = joint_states.effort;
}


//////////////////////////////////////////////////////////////
//                       MAIN CODE                          //
//////////////////////////////////////////////////////////////

int main(int argc, char** argv){

  ROS_INFO("summit_odometry node loaded!");
  ros::init(argc, argv, "summit_odometry");
  ros::NodeHandle n;
  double frequency = 50.0; //Hz

  ros::Subscriber joint_subscriber = n.subscribe("/joint_states", 1, jointStatesCallback);
  ros::Subscriber imu_subscriber = n.subscribe("/imu_data", 1, imuCallback);

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);
  
  tf::TransformBroadcaster odom_broadcaster;
  
  double robot_pose_px_ = 0.0;
  double robot_pose_py_ = 0.0;
  double robot_pose_pa_ = 0.0;

  vecVel.assign(8, 0.0); // Initialize the velocity vector with 8 zeros 
  vecPos.assign(8, 0.0); // Initialize the position vector with 8 zeros

//  double SUMMIT_WHEEL_DIAMETER = 0.160;  // [m] up to 0.130 depending on coeficients (foam type)
  double SUMMIT_WHEEL_DIAMETER = 0.180;  // [m] 160 nominal, up to 0.130 depending on coeficients (foam type)
  double SUMMIT_D_TRACKS_M = 0.372; // [m] single mode
 
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(frequency); //50.0 The update method of every controller is called periodically by pr2_controller_manager at a frequency of 1000
  while(n.ok()){
    current_time = ros::Time::now(); 
  
    //////////////////////////////////////////////////////////////
    //                  ODOMETRY CALCULATIONS                   //
    //////////////////////////////////////////////////////////////
    
    double fSamplePeriod = 1.0 / frequency;    // Default sample period
    // Axes assigned in order of appearance in the urdf file 
    // 0-blw_dir, 1-blw, 2-brw_dir, 3-brw, 4-flw_dir, 5-flw, 6-frw_dir, 7-frw
    double v_mps = ((vecVel[1] + vecVel[3] + vecVel[5] + vecVel[7]) / 4.0) * (SUMMIT_WHEEL_DIAMETER / 2.0);
        
    ros::Time current_time;
    ros::Time last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    // Turning angle front and back
    double fBetaRads = (vecPos[4] + vecPos[6]) / 2.0;
    //double alfa_back = (vecPos[0] + vecPos[2]) / 2.0;

    // State can be obtained from joint angles
    //if ((alfa_back < 0.001) && (alfa_back > -0.001)) {
	 // Single mode
	 //}
    
    // Compute orientation just integrating imu gyro
    robot_pose_pa_ += ang_vel_z_ * fSamplePeriod;

    // Normalized
    while (robot_pose_pa_ >= PI)
         robot_pose_pa_ -= 2.0 * PI;
    while (robot_pose_pa_ <= (-PI))
         robot_pose_pa_ += 2.0 * PI;

    // Velocities
    vx = v_mps * cos(fBetaRads) * cos(robot_pose_pa_);
    vy = v_mps * cos(fBetaRads) * sin(robot_pose_pa_);

    // Positions
    robot_pose_px_ += vx * fSamplePeriod;
    robot_pose_py_ += vy * fSamplePeriod;

    //////////////////////////////////////////////////////////////
    //   At this point, we send the data by the topic /odom     //
    //////////////////////////////////////////////////////////////	

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = robot_pose_px_;
    odom_trans.transform.translation.y = robot_pose_py_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation.x = orientation_x_;
    odom_trans.transform.rotation.y = orientation_y_;
    odom_trans.transform.rotation.z = orientation_z_;
    odom_trans.transform.rotation.w = orientation_w_;
	
    //send the transform over /tf
    odom_broadcaster.sendTransform(odom_trans);  
        
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    // Position
    odom.pose.pose.position.x = robot_pose_px_;
    odom.pose.pose.position.y = robot_pose_py_;
    odom.pose.pose.position.z = 0.0;
    // Orientation
    odom.pose.pose.orientation.x = orientation_x_;
    odom.pose.pose.orientation.y = orientation_y_;
    odom.pose.pose.orientation.z = orientation_z_;
    odom.pose.pose.orientation.w = orientation_w_;
    // Pose covariance
    for(int i = 0; i < 6; i++)
      	odom.pose.covariance[i*6+i] = 0.1;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    // Linear velocities
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = 0.0;
    // Angular velocities
    odom.twist.twist.angular.x = ang_vel_x_;
    odom.twist.twist.angular.y = ang_vel_y_;
    odom.twist.twist.angular.z = ang_vel_z_;
    // Twist covariance
    for(int i = 0; i < 6; i++)
      	odom.twist.covariance[6*i+i] = 0.1;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
	
    r.sleep();
    ros::spinOnce(); 
  }
  
}

