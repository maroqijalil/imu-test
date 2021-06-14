/*
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  Based on implementation of Madgwick's IMU and AHRS algorithms.
 *  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <imu_madgwick/imu.hpp>
#include <imu_madgwick/stateless_orientation.hpp>

#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <functional>

Imu::Imu(std::string node_name)
  : rclcpp::Node(node_name) {

  initialized = false;
  filter.setWorldFrame(WorldFrame::ENU);
  filter.setAlgorithmGain(0.1);
  filter.setDriftBiasGain(0.0);

  imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data_raw", rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
    [this] (const sensor_msgs::msg::Imu::SharedPtr imu_msg_raw) {
      const geometry_msgs::msg::Vector3& ang_vel = imu_msg_raw->angular_velocity;
      const geometry_msgs::msg::Vector3& lin_acc = imu_msg_raw->linear_acceleration;
      rclcpp::Time time = imu_msg_raw->header.stamp;

      if (!initialized) {
        geometry_msgs::msg::Quaternion init_q;
        if (!StatelessOrientation::computeOrientation(WorldFrame::ENU, lin_acc, init_q)) {
          RCLCPP_WARN_STREAM(get_logger(), "The IMU seems to be in free fall, cannot determine gravity direction!");
          return;
        }

        RCLCPP_INFO(get_logger(), "First IMU message received.");
        filter.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
        last_time = time;
        initialized = true;
      }

      filter.madgwickAHRSupdateIMU(
        ang_vel.x, ang_vel.y, ang_vel.z,
        lin_acc.x, lin_acc.y, lin_acc.z,
        (time - last_time).seconds());

      double q0;
      double q1;
      double q2;
      double q3;
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      filter.getOrientation(q0, q1, q2, q3);
      tf2::Matrix3x3(tf2::Quaternion(q1,q2,q3,q0)).getRPY(roll, pitch, yaw);
  
      std::cout << "madgwick filter" << std::endl;
      std::cout << "ax " << lin_acc.x << ", ay " << lin_acc.y << ", az " << lin_acc.z << std::endl;
      std::cout << "gx " << ang_vel.x << ", gy " << ang_vel.y << ", gz " << ang_vel.z << std::endl;
      std::cout << "q0 " << q0 << ", q1 " << q1 << ", q2 " << q2 << ", q3 " << q3 << std::endl;
      std::cout << "time " << time.seconds() << ", dt " << (time - last_time).seconds() << std::endl;
      std::cout << "roll " << roll << ", pitch " << pitch << ", yaw " << yaw << std::endl;
      std::cout << "========================" << std::endl;

      last_time = time;
    }
  );
}

double Imu::radian_to_degree(double radian)
{
  double pi = 3.14159;
  return(radian * (180 / pi));
}
