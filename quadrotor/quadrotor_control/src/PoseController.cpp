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

#include <quadrotor_control/PoseController.h>

bool PoseController::init(QuadrotorInterface *interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
    // get interface handles
    pose_         = interface->getPose();
    twist_        = interface->getTwist();

    pose_input_   = interface->addInput<PoseCommandHandle>("pose");
    twist_input_  = interface->addInput<TwistCommandHandle>("pose/twist");
    twist_limit_  = interface->addInput<TwistCommandHandle>("pose/twist_limit");
    twist_output_ = interface->addOutput<TwistCommandHandle>("twist");
    interface->claim(twist_output_->getName());

    // subscribe to commanded pose and velocity
    ros::SubscribeOptions pose_subscribe_options = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(
                "command/pose", 1,
                boost::bind(&PoseController::poseCommandCallback, this, _1),
                ros::VoidConstPtr(), 0 // &callback_queue_
                );
    pose_subscriber_ = root_nh.subscribe(pose_subscribe_options);
    ros::SubscribeOptions twist_subscribe_options = ros::SubscribeOptions::create<geometry_msgs::TwistStamped>(
                "command/twist", 1,
                boost::bind(&PoseController::twistCommandCallback, this, _1),
                ros::VoidConstPtr(), 0 // &callback_queue_
                );
    twist_subscriber_ = root_nh.subscribe(twist_subscribe_options);

    // initialize PID controllers
    pid_.x.init(ros::NodeHandle(controller_nh, "x"));
    pid_.y.init(ros::NodeHandle(controller_nh, "y"));
    pid_.z.init(ros::NodeHandle(controller_nh, "z"));
    pid_.yaw.init(ros::NodeHandle(controller_nh, "yaw"));

    return true;
}

void PoseController::reset()
{
    pid_.x.reset();
    pid_.y.reset();
    pid_.x.reset();
    pid_.yaw.reset();
}

void PoseController::poseCommandCallback(const geometry_msgs::PoseStampedConstPtr& command)
{
    pose_command_ = *command;
    if (!(pose_input_->connected())) *pose_input_ = &(pose_command_.pose);
    pose_input_->start();

    ros::Time start_time = command->header.stamp;
    if (start_time.isZero()) start_time = ros::Time::now();
    if (!isRunning()) this->startRequest(start_time);
}

void PoseController::twistCommandCallback(const geometry_msgs::TwistStampedConstPtr& command)
{
    twist_command_ = *command;
    if (!(twist_input_->connected())) *twist_input_ = &(twist_command_.twist);
    twist_input_->start();

    ros::Time start_time = command->header.stamp;
    if (start_time.isZero()) start_time = ros::Time::now();
    if (!isRunning()) this->startRequest(start_time);
}

void PoseController::starting(const ros::Time &time)
{
    reset();
    twist_output_->start();
}

void PoseController::stopping(const ros::Time &time)
{
    twist_output_->stop();
}

void PoseController::update(const ros::Time& time, const ros::Duration& period)
{
    Twist output;

    // execute available callbacks in the callback queue (is this real-time safe?)
    // callback_queue_.callAvailable();

    // check command timeout
    // TODO

    // return if no pose command is available
    if (pose_input_->enabled())
    {
        // control horizontal position
        double error_n, error_w;
        HorizontalPositionCommandHandle(*pose_input_).getError(*pose_, error_n, error_w);
        output.linear.x = pid_.x.update(error_n, twist_->twist().linear.x, period);
        output.linear.y = pid_.y.update(error_w, twist_->twist().linear.y, period);

        // control height
        output.linear.z = pid_.z.update(HeightCommandHandle(*pose_input_).getError(*pose_), twist_->twist().linear.z, period);

        // control yaw angle
        output.angular.z = pid_.yaw.update(HeadingCommandHandle(*pose_input_).getError(*pose_), twist_->twist().angular.z, period);
    }

    // limit twist
    if (twist_limit_->enabled())
    {
        double linear_x = fabs(output.linear.x);
        double limit_linear_x  = twist_limit_->get()->linear.x;
        if (limit_linear_x > 0.0)
        {
            output.linear.x *= limit_linear_x / linear_x;
        }

        double linear_y = fabs(output.linear.y);
        double limit_linear_y  = twist_limit_->get()->linear.y;
        if (limit_linear_y > 0.0)
        {
            output.linear.y *= limit_linear_y / linear_y;
        }

        /*double linear_xy = sqrt(output.linear.x*output.linear.x + output.linear.y*output.linear.y);
        double limit_linear_xy  = std::max(twist_limit_->get()->linear.x, twist_limit_->get()->linear.y);
        if (limit_linear_xy > 0.0 && linear_xy > limit_linear_xy)
        {
            output.linear.x *= limit_linear_xy / linear_xy;
            output.linear.y *= limit_linear_xy / linear_xy;
        }
        if (twist_limit_->get()->linear.z > 0.0 && fabs(output.linear.z) > twist_limit_->get()->linear.z)
        {
            output.linear.z *= twist_limit_->get()->linear.z / fabs(output.linear.z);
        }
        double angular_xy = sqrt(output.angular.x*output.angular.x + output.angular.y*output.angular.y);
        double limit_angular_xy  = std::max(twist_limit_->get()->angular.x, twist_limit_->get()->angular.y);
        if (limit_angular_xy > 0.0 && angular_xy > limit_angular_xy)
        {
            output.angular.x *= limit_angular_xy / angular_xy;
            output.angular.y *= limit_angular_xy / angular_xy;
        }*/
        if (twist_limit_->get()->angular.z > 0.0 && fabs(output.angular.z) > twist_limit_->get()->angular.z)
        {
            output.angular.z *= twist_limit_->get()->angular.z / fabs(output.angular.z);
        }
    }

    // set twist output
    twist_output_->setCommand(output);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PoseController, controller_interface::ControllerBase)
