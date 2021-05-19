/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

	@section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. Neither the name of the City College of New York nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL the CCNY ROBOTICS LAB BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <imu_complementary/imu.hpp>

#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <functional>

namespace imu_tools {

Imu::Imu(std::string node_name)
  : rclcpp::Node(node_name) {
  double gain_acc = 0.01;
  double bias_alpha = 0.01;

  initialized_filter = false;
  fixed_frame_ = "odom";
  constant_dt_ = 0.0;

  filter.setDoBiasEstimation(do_bias_estimation);
  filter.setDoAdaptiveGain(do_adaptive_gain);

  if(!filter.setGainAcc(gain_acc))
    RCLCPP_WARN(get_logger(), "Invalid gain_acc passed to ComplementaryFilter.");

  if(!filter.setBiasAlpha(bias_alpha))
    RCLCPP_WARN(get_logger(), "Invalid bias_alpha passed to ComplementaryFilter.");

  imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 5);
  imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data_raw", rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
    std::bind(&Imu::imuCallback, this, std::placeholders::_1));
}

void Imu::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_raw) {
  const geometry_msgs::Vector3& a = imu_msg_raw->linear_acceleration; 
  const geometry_msgs::Vector3& w = imu_msg_raw->angular_velocity;
  const ros::Time& time = imu_msg_raw->header.stamp;

  if (!initialized_filter)
  {   
    time_prev = time;
    initialized_filter_ = true;
    return; 
  }

  double dt = (time - time_prev).seconds();
  time_prev_ = time;

  filter.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);

  publish(imu_msg_raw);
}

tf::Quaternion Imu::hamiltonToTFQuaternion(
  double q0, double q1, double q2, double q3) const {
  return tf::Quaternion(q1, q2, q3, q0);
}

void Imu::publish(const sensor_msgs::msg::Imu::SharedPtr imu_msg_raw) {
  double q0, q1, q2, q3;
  filter.getOrientation(q0, q1, q2, q3);
  tf::Quaternion q = hamiltonToTFQuaternion(q0, q1, q2, q3);

  std::shared_ptr<sensor_msgs::msg::Imu> imu_msg = std::make_shared<sensor_msgs::msg::Imu>(*imu_msg_raw);
  tf::quaternionTFToMsg(q, imu_msg->orientation);

  if (filter.getDoBiasEstimation())
  {
    imu_msg->angular_velocity.x -= filter.getAngularVelocityBiasX();
    imu_msg->angular_velocity.y -= filter.getAngularVelocityBiasY();
    imu_msg->angular_velocity.z -= filter.getAngularVelocityBiasZ();
  }

  // imu_publisher.publish(imu_msg);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  // yaw = atan2f((2*q1*q2 - 2*q0*q3), (2*q0*q0 + 2*q1*q1 -1));
  // pitch = -asinf(2*q1*q3 + 2*q0*q2);
  // roll  = atan2f((2*q2*q3 - 2*q0*q1), (2*q0*q0 + 2*q3*q3 -1));
  
  // yaw *= (180.0f / 3.14159265358979f);
  // pitch *= (180.0f / 3.14159265358979f);
  // roll *= (180.0f / 3.14159265358979f);

  tf2::Matrix3x3(tf2::Quaternion(q1,q2,q3,q0)).getRPY(roll, pitch, yaw);
  RCLCPP_INFO(get_logger(), "Roll %f, Pitch %f, Yaw %f", roll, pitch, yaw);
}

}  // namespace imu_tools
