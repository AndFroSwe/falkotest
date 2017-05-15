// ROS
#include "ros/ros.h"

// Libraries
#include <Eigen/Core>




int main(int argc, char *argv[])
{
    const char node_name[] = "falko_pub";
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    ROS_INFO("Node %s running", node_name);

    return 0;
}
