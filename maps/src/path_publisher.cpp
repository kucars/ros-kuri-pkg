#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <tf_conversions/tf_eigen.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::Publisher path_pub = node.advertise<nav_msgs::Path>("path", 10);

  tf::TransformListener listener;
  nav_msgs::Path path_msg;
  ros::Rate rate(10.0);
  while (node.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/base_link", "/odom",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    geometry_msgs::PoseStamped odom;
    //tf::poseStampedTFToMsg (&transform, odom);
    path_msg.poses.push_back(odom);
    
    rate.sleep();
  }
  return 0;
};
