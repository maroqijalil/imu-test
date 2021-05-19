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

#ifndef IMU_HPP
#define IMU_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <imu_complementary/complementary_filter.hpp>

#include <string>
#include <memory>

namespace imu_tools {

class Imu : public rclcpp::Node
{
public:
  Imu(std::string node_name);

private:
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_publisher;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subscriber;
         
  // Parameters:
  bool use_mag;
  bool publish_tf;
  bool reverse_tf;
  double constant_dt;
  bool publish_debug_topics;
  std::string fixed_frame;

  // State variables:
  ComplementaryFilter filter;
  ros::Time time_prev;
  bool initialized_filter;

  void initializeParams();
  void imuCallback(const ImuMsg::ConstPtr& imu_msg_raw);
  void publish(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);

  tf::Quaternion hamiltonToTFQuaternion(
      double q0, double q1, double q2, double q3) const;
};

}  // namespace imu_tools

#endif // IMU_HPP
