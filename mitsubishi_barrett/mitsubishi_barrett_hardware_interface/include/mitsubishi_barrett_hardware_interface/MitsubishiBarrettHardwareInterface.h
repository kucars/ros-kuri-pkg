#include <barrett_hand_hardware_interface/BarrettHandHardwareInterface.h>
#include <mitsubishi_arm_hardware_interface/MitsubishiArmInterface.h>

class MitsubishiBarrettHardwareInterface : public hardware_interface::RobotHW
{
public:
    MitsubishiBarrettHardwareInterface(const int & canbus_number_, bool & forcetorque_, bool & tactile_,  std::string & calibration_filename_);
    ~MitsubishiBarrettHardwareInterface();

    void readHW();
    void writeHW();

    void readTactile()
    {
        //bus->tactile_get_data
    }
    int USB;
    boost::shared_ptr<OWD::WamDriver> wamdriver;


private:
    static const unsigned int mitsubishi_joint_number=6;
    static const unsigned int barrett_joint_number=8;

    static const unsigned int joint_number=14;
    hardware_interface::JointStateInterface jnt_state_interface;

    hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    hardware_interface::EffortJointInterface jnt_eff_interface;


    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> eff;
    std::vector<double> pos_cmd;
    std::vector<double> vel_cmd;
    std::vector<double> eff_cmd;
    std::vector<double> pos_cmd_previous;
    std::vector<double> vel_cmd_previous;
    std::vector<double> eff_cmd_previous;

    void readMitsubishiHW();
    void writeMitsubishiHW();
    void readBarrettHW();
    void writeBarrettHW();

    void readBarrettPosition();
    void readBarrettTorque();
    void readBarrettPositionAndComputeVelocity(ros::Duration &  period);

    void writeBarrettPosition();
    void writeBarrettVelocity();
    void writeBarrettEffort();



    ros::NodeHandle n;
    ros::Time prev_time;

    // Mitsubishi COM
    struct termios tty;


    boost::shared_ptr<BHD_280> bhd;
    //    boost::shared_ptr<FT> ft;
    boost::shared_ptr<Tactile> tact;

    // OWD parameters
    std::string calibration_filename;
    int canbus_number;
    std::string hand_type;
    bool forcetorque;
    int pub_freq;  // DEPRECATED
    int wam_pub_freq;
    int hand_pub_freq;
    int ft_pub_freq;
    int tactile_pub_freq;
    bool tactile;

};
