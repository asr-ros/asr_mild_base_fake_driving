/**

Copyright (c) 2016, Dehmani Souheil, Karrenbauer Oliver, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h>
#include <boost/thread/mutex.hpp>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <gazebo_msgs/ModelState.h>

//This file simulates robot base by shortcircuting BaseController and CanListener. cmd_vel from move_base are converted into odom pose increments for the robot.

boost::mutex mutex;

//Input command from move_base to fake robot base.
geometry_msgs::Twist velocity_cmd;

//State of fake robot base.
//Pose
double x = 0.0;
double y = 0.0;
double th = 0.0;
//Velocities
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

//Save velocity command from move base instead of generating motor control commands.
void fakeSetTargetVelocity(const geometry_msgs::Twist &cmd_vel) {
  ROS_INFO("fakeSetTargetVelocity");
  boost::mutex::scoped_lock scoped_lock(mutex);
  velocity_cmd = cmd_vel;
}

//Accept initial fake base pose from ros topic.
void gotInitialRobotPose(const geometry_msgs::PoseWithCovarianceStamped &initialPose) {
  ROS_INFO("initialpose");
  x = initialPose.pose.pose.position.x;
  y = initialPose.pose.pose.position.y;
  th = tf::getYaw(initialPose.pose.pose.orientation);
}

geometry_msgs::TransformStamped getOdomTF(ros::Time current_time) {
  
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;

}

nav_msgs::Odometry getOdomMsg(ros::Time current_time) {

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    return odom;

}

gazebo_msgs::ModelState getRobotStateGazebo(){

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
  
    gazebo_msgs::ModelState state;
    state.model_name = "mild";

    state.pose.position.x = x;
    state.pose.position.y = y;
    state.pose.position.z = 0.0;
    state.pose.orientation = odom_quat;
    state.twist.linear.x = vx;
    state.twist.linear.y = vy;
    state.twist.angular.z = vth;

    return state;

}

int main(int argc, char** argv){

  ros::init(argc, argv, "mild_fake_odom");

  ros::NodeHandle n;

  ros::Subscriber twist_sub = n.subscribe("/cmd_vel", 1, fakeSetTargetVelocity);
  ros::Subscriber init_sub = n.subscribe("/initialpose", 1, gotInitialRobotPose);

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Publisher gazebo_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate r(10);

  while(n.ok()){

    ros::spinOnce();
    current_time = ros::Time::now();

    {
      boost::mutex::scoped_lock scoped_lock(mutex);

      vx = velocity_cmd.linear.x;
      vy = velocity_cmd.linear.y;
      vth = velocity_cmd.angular.z;
      ROS_INFO("velocity_cmd vx : %f  vy : %f  vz : %f", vx, vy,vth);
    }

    //Compute cartesian robot pose (for odometry), given the velocities of the robot. Direct kinematics.
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(getOdomTF(current_time));

    //next, we'll publish the odometry message over ROS
    odom_pub.publish(getOdomMsg(current_time));

    //gazebo simulation
    gazebo_pub.publish(getRobotStateGazebo());

    last_time = current_time;
    r.sleep();

    //Velocity command processed -> set to zero for no further processing until new command comes from move_base
    velocity_cmd.linear.x = 0;
    velocity_cmd.linear.y = 0;
    velocity_cmd.angular.z = 0;

  }
}
