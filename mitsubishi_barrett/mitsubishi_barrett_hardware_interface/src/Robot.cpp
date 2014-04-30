#include <controller_manager/controller_manager.h>
#include <mitsubishi_barrett_hardware_interface/MitsubishiBarrettHardwareInterface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mitsubishi_barrett_hardware_interface");

    // read parameters and set wam options

    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");

    std::string calibration_filename;
    int canbus_number;
    std::string hand_type;
    bool forcetorque;
    bool modified_j1;
    double pub_freq;  // DEPRECATED
    double wam_pub_freq;
    double hand_pub_freq;
    double ft_pub_freq;
    double tactile_pub_freq;
    n_priv.param("calibration_file",calibration_filename,std::string("wam_joint_calibrations"));
    n_priv.param("canbus_number",canbus_number,32);
    n_priv.param("hand_type",hand_type,std::string("280+TACT"));
    n_priv.param("forcetorque_sensor",forcetorque,false);
    n_priv.param("modified_j1",modified_j1,false);
    double wam_freq_default=10.0;
    double hand_freq_default=10.0;

    if (n_priv.getParam("publish_frequency",pub_freq))
    {
        ROS_WARN("Parameter publish_frequency has been deprecated.  Please use wam_publish_frequency, hand_publish_frequency, ft_publish_frequency, and tactile_publish_frequency.");
        // we'll use the deprecated value to set the defaults for the
        // individual hand and wam frequencies, so that it will only be
        // used if the newer params are not specified
        wam_freq_default=hand_freq_default=pub_freq;
    }
    n_priv.param("wam_publish_frequency",wam_pub_freq,wam_freq_default);
    n_priv.param("hand_publish_frequency",hand_pub_freq,40.0);
    n_priv.param("ft_publish_frequency",ft_pub_freq,10.0);
    n_priv.param("tactile_publish_frequency",tactile_pub_freq,10.0);

    if (wam_pub_freq > 500)
    {
        ROS_WARN("value of wam_publish_frequency exceeds maximum sensor rate; capping to 500Hz");
        wam_pub_freq = 500;
    }
    if (hand_pub_freq > 40)
    {
        ROS_WARN("value of hand_publish_frequency exceeds maximum sensor rate; capping to 40Hz");
        hand_pub_freq = 40;
    }
    if (ft_pub_freq > 500)
    {
        ROS_WARN("value of ft_publish_frequency exceeds maximum sensor rate; capping to 500Hz");
        ft_pub_freq = 500;
    }
    if (tactile_pub_freq > 40)
    {
        ROS_WARN("value of tactile_publish_frequency exceeds maximum sensor rate; capping to 40Hz");
        tactile_pub_freq = 40;
    }

    ROS_DEBUG("Using CANbus number %d",canbus_number);

    int BH_model(0);
    bool tactile(false);
    if (! hand_type.compare(0,3,"280"))
    {
        BH_model=280;
        if (! hand_type.compare("280+TACT"))
        {
            ROS_DEBUG("Expecting tactile sensors on this hand");
            tactile=true;
        }
    }
    else
    {
        exit(-1);
    }

    MitsubishiBarrettHardwareInterface robot(canbus_number, forcetorque, tactile,  calibration_filename, ft_pub_freq, tactile_pub_freq);
    ROS_INFO("CHEGUEI AQUI");
    controller_manager::ControllerManager cm(&robot, n);

    ros::Time previous=ros::Time::now();

    ros::Rate rate(100.0);
    std::cout << "DONE" << std::endl;
    //#endif // OWDSIM

    ros::AsyncSpinner spinner(4);
    spinner.start();
    while (ros::ok())
    {
        ros::Duration period;
        robot.mitsubishi_arm_interface.readHW();
        robot.barrett_arm_interface.readHW();

        ros::Time now=ros::Time::now();
        period=now-previous;
        //std::cout << "period:"<<period<<std::endl;
        cm.update(now, period);
        robot.mitsubishi_arm_interface.writeHW();
        robot.barrett_arm_interface.writeHW();

        ros::spinOnce();
        rate.sleep();
    }
    spinner.stop();

    return 0;
}



