#include <quadrotor_hardware_interface/QuadrotorHardwareInterface.h>

// By Rui Pimentel de Figueiredo addapted from:
//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

namespace quadrotor_hardware_interface {

QuadrotorHardwareInterface::QuadrotorHardwareInterface()
{
    //this->registerInterface(static_cast<QuadrotorInterface *>(this));
    velocity_output_ = addOutput<TwistCommandHandle>("twist");
}

QuadrotorHardwareInterface::~QuadrotorHardwareInterface()
{

}

bool QuadrotorHardwareInterface::init(const std::string& robot_namespace, ros::NodeHandle model_nh)
{
    ros::NodeHandle param_nh(model_nh, "controller");

    // subscribe state
    std::string state_topic;
    param_nh.getParam("state_topic", state_topic);
    if (!state_topic.empty())
    {
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<nav_msgs::Odometry>(state_topic, 1, boost::bind(&QuadrotorHardwareInterface::stateCallback, this, _1), ros::VoidConstPtr(), &callback_queue_);
        subscriber_state_ = model_nh.subscribe(ops);

        ROS_INFO_STREAM("[QUADROTOR INTERFACE] Using topic '" << subscriber_state_.getTopic() << "' as state input for control");
    }
    else
    {
        ROS_INFO_STREAM("[QUADROTOR INTERFACE] Using ground truth as state input for control");
    }

    // subscribe imu
    std::string imu_topic;
    param_nh.getParam("imu_topic", imu_topic);
    if (!imu_topic.empty())
    {
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(imu_topic, 1, boost::bind(&QuadrotorHardwareInterface::imuCallback, this, _1), ros::VoidConstPtr(), &callback_queue_);
        subscriber_imu_ = model_nh.subscribe(ops);
        ROS_INFO_STREAM("[QUADROTOR INTERFACE] Using topic '" << subscriber_imu_.getTopic() << "' as imu input for control");
    }
    else
    {
        ROS_INFO_STREAM("[QUADROTOR INTERFACE] Using ground truth as imu input for control");
    }


    // publish velocity
    {
        ros::AdvertiseOptions ops = ros::AdvertiseOptions::create<geometry_msgs::TwistStamped>("command/twist", 1, ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(), ros::VoidConstPtr(), &callback_queue_);
        publisher_velocity_command_ = model_nh.advertise(ops);
    }
    return true;
}

void QuadrotorHardwareInterface::stateCallback(const nav_msgs::OdometryConstPtr &state) {
    // calculate acceleration
    if (!header_.stamp.isZero() && !state->header.stamp.isZero())
    {
        const double acceleration_time_constant = 0.1;
        double dt((state->header.stamp - header_.stamp).toSec());
        if (dt > 0.0) {
            acceleration_.x = ((state->twist.twist.linear.x - twist_.linear.x) + acceleration_time_constant * acceleration_.x) / (dt + acceleration_time_constant);
            acceleration_.y = ((state->twist.twist.linear.y - twist_.linear.y) + acceleration_time_constant * acceleration_.y) / (dt + acceleration_time_constant);
            acceleration_.z = ((state->twist.twist.linear.z - twist_.linear.z) + acceleration_time_constant * acceleration_.z) / (dt + acceleration_time_constant);
        }
    }

    header_ = state->header;
    pose_ = state->pose.pose;
    twist_ = state->twist.twist;
}

void QuadrotorHardwareInterface::imuCallback(const sensor_msgs::ImuConstPtr &imu)
{
    imu_ = *imu;
}

void QuadrotorHardwareInterface::writeHW(ros::Time time, ros::Duration period)
{
    if (velocity_output_->connected() && velocity_output_->enabled())
    {
        geometry_msgs::TwistStamped twist;
        twist.header.stamp = time;
        twist.header.frame_id = "base_link";

        //wrench_horizontal_output_->getCommand(twist.twist.linear.x,twist.twist.linear.y);
        twist.twist=velocity_output_->getCommand();
        publisher_velocity_command_.publish(twist);
    }
}

} // namespace quadrotor_hardware_interface

//#include <pluginlib/class_list_macros.h>
//PLUGINLIB_EXPORT_CLASS(quadrotor_hardware_interface::QuadrotorHardwareInterface, gazebo_ros_control::RobotHWSim)
