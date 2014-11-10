/*
 * summit_sbpl_nav
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
 * \brief Converts cmd_vel messages to diff drive robots (able to make pure rotations) to
 *        Car-like / Ackerman speed references (linear speed always > 0)
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Twist base_vel_msg_;
double previous_linear_vel = 0.0;

void commandCallback(const geometry_msgs::TwistConstPtr& msg)
{
  base_vel_msg_ = *msg;
/*
  if ( (base_vel_msg_.linear.x == 0.0) && (base_vel_msg_.linear.y == 0.0) ) 
  {
    if ( base_vel_msg_.angular.z != 0.0) {
       if (previous_linear_vel<0.0) base_vel_msg_.linear.x = -0.1;
       else base_vel_msg_.linear.x = 0.1;
       } 
  }
*/
  previous_linear_vel = base_vel_msg_.linear.x;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "summit_sbpl_nav");
  ros::NodeHandle n;
  ros::Subscriber cmd_sub_;  // To handle references 
  geometry_msgs::Twist cmd_vel_; 


  ros::Publisher cmd_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

  ros::Rate r(50);

  cmd_sub_ = n.subscribe<geometry_msgs::Twist>("/cmd_vel1", 1, &commandCallback );

  ROS_INFO("summit_sbpl_nav node loaded!");

  while(n.ok()){

    //publish the message
    cmd_pub_.publish(base_vel_msg_);

    ros::spinOnce();
    r.sleep();
  }
}

