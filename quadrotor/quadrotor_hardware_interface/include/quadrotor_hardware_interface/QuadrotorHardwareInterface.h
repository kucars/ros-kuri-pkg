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

#ifndef QUADROTOR_HARDWARE_INTERFACE_H
#define QUADROTOR_HARDWARE_INTERFACE_H

#include <hector_quadrotor_controller/quadrotor_interface.h>



#include <ros/node_handle.h>
#include <ros/callback_queue.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>

namespace quadrotor_hardware_interface {

using namespace hector_quadrotor_controller;
using namespace hardware_interface;

class QuadrotorHardwareInterface : public QuadrotorInterface
{
public:
  QuadrotorHardwareInterface();
  virtual ~QuadrotorHardwareInterface();

  virtual const ros::Time &getTimestamp() { return header_.stamp; }

  virtual PoseHandlePtr getPose()                 { return PoseHandlePtr(new PoseHandle(this, &pose_)); }
  virtual TwistHandlePtr getTwist()               { return TwistHandlePtr(new TwistHandle(this, &twist_)); }
  virtual AccelerationHandlePtr getAcceleration() { return AccelerationHandlePtr(new AccelerationHandle(this, &acceleration_)); }
  virtual ImuHandlePtr getSensorImu()             { return ImuHandlePtr(new ImuHandle(this, &imu_)); }

  //virtual bool getMassAndInertia(double &mass, double inertia[3]);

  virtual bool init(const std::string& robot_namespace, ros::NodeHandle model_nh);

  virtual void writeHW(ros::Time time, ros::Duration period);


private:
  void stateCallback(const nav_msgs::OdometryConstPtr &state);
  void imuCallback(const sensor_msgs::ImuConstPtr &imu);

protected:
  std_msgs::Header header_;
  Pose pose_;
  Twist twist_;
  Vector3 acceleration_;
  Imu imu_;

  TwistCommandHandlePtr velocity_output_;

  ros::CallbackQueue callback_queue_;

  ros::Subscriber subscriber_state_;
  ros::Subscriber subscriber_imu_;
  ros::Publisher publisher_velocity_command_;
};

} // namespace quadrotor_hardware_interface

#endif // QUADROTOR_HARDWARE_INTERFACE_H
