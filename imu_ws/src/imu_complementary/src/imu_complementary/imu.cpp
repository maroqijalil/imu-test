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
#include <fstream>
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
  fixed_frame = "odom";
  constant_dt = 0.0;

  filter.setDoBiasEstimation(true);
  filter.setDoAdaptiveGain(true);

  outdata.open("complementary.txt");
  counter = 0;

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
  const geometry_msgs::msg::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 
  const geometry_msgs::msg::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::msg::Quaternion& orient = imu_msg_raw->orientation;
  const rclcpp::Time& time = imu_msg_raw->header.stamp;

  counter++;
  
  RCLCPP_INFO(get_logger(), "ax %f, ay %f, az %f", lin_acc.x, lin_acc.y, lin_acc.z);
  RCLCPP_INFO(get_logger(), "gx %f, gy %f, gz %f", ang_vel.x, ang_vel.y, ang_vel.z);
  RCLCPP_INFO(get_logger(), "qx %f, qy %f, qz %f", orient.x, orient.y, orient.z);

  if (counter <= 100) {
    outdata << "ax " << lin_acc.x << ", ay " << lin_acc.y << ", az " << lin_acc.z << std::endl;
    outdata << "gx " << ang_vel.x << ", gy " << ang_vel.y << ", gz " << ang_vel.z << std::endl;
    outdata << "qx " << orient.x << ", qy " << orient.y << ", qz " << orient.z << std::endl;
  }

  if (!initialized_filter)
  {   
    time_prev = time;
    initialized_filter = true;
    return; 
  }

  double dt = (time - time_prev).seconds();
  time_prev = time;

  // filter.update(lin_acc.x, lin_acc.y, lin_acc.z, ang_vel.x, ang_vel.y, ang_vel.z, dt);
  filter.update(lin_acc.x, lin_acc.y, lin_acc.z, 0.0, 0.0, 0.0, dt);

  double q0, q1, q2, q3;
  filter.getOrientation(q0, q1, q2, q3);

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

  if (counter <= 100) {
    outdata << "roll " << roll << ", pitch " << pitch << ", yaw " << yaw << std::endl;
  } else {
    outdata.close();
  }
}

void Imu::publish(const sensor_msgs::msg::Imu::SharedPtr imu_msg_raw) {

  // std::shared_ptr<sensor_msgs::msg::Imu> imu_msg = std::make_shared<sensor_msgs::msg::Imu>(*imu_msg_raw);

  // if (filter.getDoBiasEstimation())
  // {
  //   imu_msg->angular_velocity.x -= filter.getAngularVelocityBiasX();
  //   imu_msg->angular_velocity.y -= filter.getAngularVelocityBiasY();
  //   imu_msg->angular_velocity.z -= filter.getAngularVelocityBiasZ();
  // }

  // imu_publisher.publish(imu_msg);
}

}  // namespace imu_tools
