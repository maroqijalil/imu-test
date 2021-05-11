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
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
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
  : rclcpp::Node(node_name),
  initialized(false) {
  RCLCPP_INFO(get_logger(), "Starting ImuFilter");

  stateless = false;
  use_mag = true;
  publish_tf = true;
  reverse_tf = false;
  fixed_frame = "odom";
  constant_dt = 0.0;
  remove_gravity_vector = false;
  publish_debug_topics = false;

  filter.setWorldFrame(WorldFrame::ENU);
  filter.setAlgorithmGain(0.1);
  filter.setDriftBiasGain(0.0);

  mag_bias.x = 0.0;
  mag_bias.y = 0.0;
  mag_bias.z = 0.0;
  orientation_variance = 0.0;

  imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 5);
  imu_subscriber.reset(new ImuSubscriber(this, "imu/data_raw"));

  if (use_mag) {
    mag_subscriber.reset(new MagSubscriber(this, "imu/mag"));

    sync.reset(new Synchronizer(SyncPolicy(5), * imu_subscriber, * mag_subscriber));
    sync->registerCallback(std::bind(&Imu::imuMagCallback, this, std::placeholders::_1, std::placeholders::_2));
  } else {
    imu_subscriber->registerCallback(&Imu::imuCallback, this);
  }

  check_topics_timer = this->create_wall_timer(10ms, std::bind(&Imu::checkTopicsTimerCallback, this));
}

Imu::~Imu() {
  RCLCPP_INFO(get_logger(), "Destroying ImuFilter");
}

void Imu::imuCallback(const ImuMsg::SharedPtr imu_msg_raw) const {
  boost::mutex::scoped_lock lock(mutex);

  const geometry_msgs::msg::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::msg::Vector3& lin_acc = imu_msg_raw->linear_acceleration;

  rclcpp::Time time = imu_msg_raw->header.stamp;
  imu_frame = imu_msg_raw->header.frame_id;

  if (!initialized || stateless) {
    geometry_msgs::msg::Quaternion init_q;
    if (!StatelessOrientation::computeOrientation(world_frame, lin_acc, init_q)) {
      RCLCPP_WARN_THROTTLE(get_logger(), rclcpp::Clock(RCL_STEADY_TIME), 5.0, "The IMU seems to be in free fall, cannot determine gravity direction!");
      return;
    }
    filter.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
  }

  if (!initialized_) {
    RCLCPP_INFO(get_logger(), "First IMU message received.");
    check_topics_timer->cancel();

    last_time = time;
    initialized = true;
  }

  float dt;
  if (constant_dt > 0.0)
    dt = constant_dt;
  else
  {
    dt = (time - last_time).toSec();
    if (time.isZero())
      RCLCPP_WARN_STREAM_THROTTLE(get_logger(), rclcpp::Clock(RCL_STEADY_TIME), 5.0, "The IMU message time stamp is zero, and the parameter constant_dt is not set!" <<
                                    " The filter will not update the orientation.");
  }

  last_time = time;

  if (!stateless)
    filter_.madgwickAHRSupdateIMU(
      ang_vel.x, ang_vel.y, ang_vel.z,
      lin_acc.x, lin_acc.y, lin_acc.z,
      dt);

  publishFilteredMsg(imu_msg_raw);
  if (publish_tf)
    publishTransform(imu_msg_raw);
}

void Imu::imuMagCallback(
  const ImuMsg::SharedPtr imu_msg_raw,
  const MagMsg::SharedPtr mag_msg)
{
  boost::mutex::scoped_lock lock(mutex);

  const geometry_msgs::msg::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::msg::Vector3& lin_acc = imu_msg_raw->linear_acceleration;
  const geometry_msgs::msg::Vector3& mag_fld = mag_msg->magnetic_field;

  rclcpp::Time time = imu_msg_raw->header.stamp;
  imu_frame = imu_msg_raw->header.frame_id;

  geometry_msgs::msg::Vector3 mag_compensated;
  mag_compensated.x = mag_fld.x - mag_bias.x;
  mag_compensated.y = mag_fld.y - mag_bias.y;
  mag_compensated.z = mag_fld.z - mag_bias.z;

  if (!initialized || stateless)
  {
    if(!std::isfinite(mag_fld.x) || !std::isfinite(mag_fld.y) || !std::isfinite(mag_fld.z))
    {
      return;
    }

    geometry_msgs::msg::Quaternion init_q;
    if (!StatelessOrientation::computeOrientation(world_frame, lin_acc, mag_compensated, init_q))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), rclcpp::Clock(RCL_STEADY_TIME), 5.0, "The IMU seems to be in free fall or close to magnetic north pole, cannot determine gravity direction!");
      return;
    }
    filter.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
  }

  if (!initialized)
  {
    RCLCPP_INFO(get_logger(), "First pair of IMU and magnetometer messages received.");
    check_topics_timer->cancel();

    last_time = time;
    initialized = true;
  }

  float dt;
  if (constant_dt > 0.0)
    dt = constant_dt;
  else
  {
    dt = (time - last_time).toSec();
    if (time.isZero())
      RCLCPP_WARN_STREAM_THROTTLE(get_logger(), rclcpp::Clock(RCL_STEADY_TIME), 5.0, "The IMU message time stamp is zero, and the parameter constant_dt is not set!" <<
                                    " The filter will not update the orientation.");
  }

  last_time = time;

  if (!stateless)
    filter.madgwickAHRSupdate(
      ang_vel.x, ang_vel.y, ang_vel.z,
      lin_acc.x, lin_acc.y, lin_acc.z,
      mag_compensated.x, mag_compensated.y, mag_compensated.z,
      dt);

  publishFilteredMsg(imu_msg_raw);
  if (publish_tf)
    publishTransform(imu_msg_raw);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  geometry_msgs::msg::Quaternion orientation;
  if (StatelessOrientation::computeOrientation(world_frame, lin_acc, mag_compensated, orientation))
  {
    tf2::Matrix3x3(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw, 0);
    RCLCPP_INFO(get_logger(), "Roll %f, Pitch %f, Yaw %f", roll, pitch, yaw);
  }
}

void Imu::publishTransform(const ImuMsg::SharedPtr imu_msg_raw)
{
  double q0,q1,q2,q3;
  filter.getOrientation(q0,q1,q2,q3);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = imu_msg_raw->header.stamp;
  if (reverse_tf)
  {
    transform.header.frame_id = imu_frame;
    transform.child_frame_id = fixed_frame;
    transform.transform.rotation.w = q0;
    transform.transform.rotation.x = -q1;
    transform.transform.rotation.y = -q2;
    transform.transform.rotation.z = -q3;
  }
  else {
    transform.header.frame_id = fixed_frame;
    transform.child_frame_id = imu_frame;
    transform.transform.rotation.w = q0;
    transform.transform.rotation.x = q1;
    transform.transform.rotation.y = q2;
    transform.transform.rotation.z = q3;
  }
  tf_broadcaster.sendTransform(transform);

}

void Imu::publishFilteredMsg(const ImuMsg::SharedPtr imu_msg_raw)
{
  double q0,q1,q2,q3;
  filter.getOrientation(q0,q1,q2,q3);

  std::shared_ptr<ImuMsg> imu_msg =
    std::make_shared<ImuMsg>(* imu_msg_raw);

  imu_msg->orientation.w = q0;
  imu_msg->orientation.x = q1;
  imu_msg->orientation.y = q2;
  imu_msg->orientation.z = q3;

  imu_msg->orientation_covariance[0] = orientation_variance;
  imu_msg->orientation_covariance[1] = 0.0;
  imu_msg->orientation_covariance[2] = 0.0;
  imu_msg->orientation_covariance[3] = 0.0;
  imu_msg->orientation_covariance[4] = orientation_variance;
  imu_msg->orientation_covariance[5] = 0.0;
  imu_msg->orientation_covariance[6] = 0.0;
  imu_msg->orientation_covariance[7] = 0.0;
  imu_msg->orientation_covariance[8] = orientation_variance;

  if(remove_gravity_vector) {
    float gx, gy, gz;
    filter.getGravity(gx, gy, gz);
    imu_msg->linear_acceleration.x -= gx;
    imu_msg->linear_acceleration.y -= gy;
    imu_msg->linear_acceleration.z -= gz;
  }

  imu_publisher.publish(imu_msg);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  geometry_msgs::msg::Vector3Stamped rpy;
  tf2::Matrix3x3(tf2::Quaternion(q1,q2,q3,q0)).getRPY(roll, pitch, yaw);
  RCLCPP_INFO(get_logger(), "Roll %f, Pitch %f, Yaw %f", roll, pitch, yaw);
}

void Imu::checkTopicsTimerCallback()
{
  if (use_mag)
    RCLCPP_WARN_STREAM(get_logger(), "Still waiting for data on topics imu/data_raw and imu/mag");
  else
    RCLCPP_WARN_STREAM(get_logger(), "Still waiting for data on topic imu/data_raw");
}
