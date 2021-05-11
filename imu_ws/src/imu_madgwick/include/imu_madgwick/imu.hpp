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
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "imu_madgwick/imu_filter.hpp"

#include <iostream>
#include <string>
#include <memory>

class Imu : public rclcpp::Node
{
  public:
    using ImuMsg = sensor_msgs::msg::Imu;
    using MagMsg = sensor_msgs::msg::MagneticField;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    using ImuSubscriber = message_filters::Subscriber<ImuMsg>;
    using MagSubscriber = message_filters::Subscriber<MagMsg>;
    
    Imu(std::string node_name);
    virtual ~Imu();

  private:
    std::shared_ptr<ImuSubscriber> imu_subscriber;
    std::shared_ptr<MagSubscriber> mag_subscriber;
    std::shared_ptr<Synchronizer> sync;

    std::shared_ptr<rclcpp::Publisher<ImuMsg>> imu_publisher;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    rclcpp::TimerBase::SharedPtr check_topics_timer;

    WorldFrame::WorldFrame world_frame;
    bool use_mag;
    bool stateless;
    bool publish_tf;
    bool reverse_tf;
    std::string fixed_frame;
    std::string imu_frame;
    double constant_dt;
    bool publish_debug_topics;
    bool remove_gravity_vector;
    geometry_msgs::msg::Vector3 mag_bias;
    double orientation_variance;

    boost::mutex mutex;
    bool initialized;
    rclcpp::TimerBase::SharedPtr last_time;

    ImuFilter filter;

    void imuMagCallback(const ImuMsg::SharedPtr imu_msg_raw,
                        const MagMsg::SharedPtr mav_msg);

    void imuCallback(const ImuMsg::SharedPtr imu_msg_raw) const;

    void publishFilteredMsg(const ImuMsg::SharedPtr imu_msg_raw);
    void publishTransform(const ImuMsg::SharedPtr imu_msg_raw);

    void publishRawMsg(const rclcpp::TimerBase::SharedPtr t,
                       float roll, float pitch, float yaw);

    void checkTopicsTimerCallback();
};

#endif // IMU_FILTER_IMU_MADWICK_FILTER_ROS_H
