#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>


class GripperController
{
    ros::NodeHandle n_;
    bool open(std_srvs::Empty::Request  &req,
               std_srvs::Empty::Response &res)
    {
        std_msgs::Float64 left_dist_close, left_med_close, right_dist_close, right_med_close;
        left_dist_close.data=1.5;
        left_med_close.data=0.0;
        right_dist_close.data=-1.5;
        right_med_close.data=0.0;

        left_dist.publish(left_dist_close);
        left_med.publish(left_med_close);

        right_dist.publish(right_dist_close);
        right_med.publish(right_med_close);
        return true;
    }

    bool close(std_srvs::Empty::Request  &req,
              std_srvs::Empty::Response &res)
    {

        std_msgs::Float64 left_dist_open, left_med_open, right_dist_open, right_med_open;

        left_dist_open.data=0.0;
        left_med_open.data=-1.0;
        right_dist_open.data=0.0;
        right_med_open.data=1.0;

        left_dist.publish(left_dist_open);
        left_med.publish(left_med_open);

        right_dist.publish(right_dist_open);
        right_med.publish(right_med_open    );
        return true;
    }


public:
    GripperController(ros::NodeHandle & n) : n_(n)
    {
        left_dist =  n_.advertise<std_msgs::Float64> ("/left_dist_controller/command", 2);
        left_med =   n_.advertise<std_msgs::Float64>  ("/left_med_controller/command", 2);
        right_dist = n_.advertise<std_msgs::Float64>("/right_dist_controller/command", 2);
        right_med =  n_.advertise<std_msgs::Float64> ("/right_med_controller/command", 2);
        close_srv =  n_.advertiseService("close_gripper", &GripperController::close, this);
        open_srv =   n_.advertiseService("open_gripper" , &GripperController::open,  this);
    }

    ros::Publisher left_dist;
    ros::Publisher left_med;

    ros::Publisher right_dist;
    ros::Publisher right_med;
    ros::ServiceServer close_srv;
    ros::ServiceServer open_srv;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    GripperController gripper_controller(n);


    ros::Rate loop_rate(20);

    /**
      * A count of how many messages we have sent. This is used to create
      * a unique string for each message.
      */
    int i=0;
    ros::spin();
    /*while (ros::ok())
    {

        std_msgs::Float64 msg;

        msg.data = 0.0+ 0.0001*i;

        std::cout << msg.data <<  std::endl;


        ros::spinOnce();

        loop_rate.sleep();
        i++;
    }*/

    return 0;
}
