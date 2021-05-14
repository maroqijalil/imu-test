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

#ifndef IMU_FILTER_MADWICK_IMU_FILTER_ROS_H
#define IMU_FILTER_MADWICK_IMU_FILTER_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <imu_madgwick/imu_filter.hpp>

#include <string>
#include <memory>

class Imu : public rclcpp::Node
{
public:
  Imu(std::string node_name);

private:
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_publisher;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subscriber;

  WorldFrame::WorldFrame world_frame;
  std::string imu_frame;
  bool stateless;
  double constant_dt;
  bool remove_gravity_vector;

  bool initialized;
  rclcpp::Time last_time;
  ImuFilter filter;

  void publishFilteredMsg(const sensor_msgs::msg::Imu::SharedPtr imu_msg_raw);
};

#endif // IMU_FILTER_IMU_MADWICK_FILTER_ROS_H
