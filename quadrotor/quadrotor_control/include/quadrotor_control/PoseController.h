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

#include <quadrotor_hardware_interface/QuadrotorHardwareInterface.h>
#include <hector_quadrotor_controller/pid.h>

#include <controller_interface/controller.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <ros/subscriber.h>
#include <ros/callback_queue.h>

using namespace quadrotor_hardware_interface;
using namespace controller_interface;

class PoseController : public controller_interface::Controller<QuadrotorHardwareInterface::QuadrotorInterface>
{
public:
    bool init(QuadrotorInterface *interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

    void reset();

    void poseCommandCallback(const geometry_msgs::PoseStampedConstPtr& command);

    void twistCommandCallback(const geometry_msgs::TwistStampedConstPtr& command);

    void starting(const ros::Time &time);

    void stopping(const ros::Time &time);

    void update(const ros::Time& time, const ros::Duration& period);

private:
    PoseHandlePtr pose_;
    PoseCommandHandlePtr pose_input_;
    TwistHandlePtr twist_;
    TwistCommandHandlePtr twist_input_;
    TwistCommandHandlePtr twist_limit_;
    TwistCommandHandlePtr twist_output_;

    geometry_msgs::PoseStamped pose_command_;
    geometry_msgs::TwistStamped twist_command_;

    // ros::CallbackQueue callback_queue_;
    ros::Subscriber pose_subscriber_;
    ros::Subscriber twist_subscriber_;

    struct
    {
        PID x;
        PID y;
        PID z;
        PID yaw;
    } pid_;
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PoseController, controller_interface::ControllerBase)
