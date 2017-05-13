/* 
 * File: extract_keypoints.cpp
 * Author: Andreas Fröderberg
 * Description: Subscibes to a laser scan topic and extracts keypoints. These 
 * are then published.
 */

// ROS
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

// Libraries
// falkolib does the feature extraction
#include <falkolib/Common/LaserScan.h>
#include <falkolib/Feature/FALKO.h>             // Keypoint class
#include <falkolib/Feature/FALKOExtractor.h>    // Contains keypoint extraction methods

// Subscriber callback
// Create a laserscan message that will be filled with data by converting the 
// laserscan data from ROS.
//                          (angle_min, fov, num_beams)
const unsigned int num_beams = 360;
falkolib::LaserScan scan_data(-M_PI, 2*M_PI, num_beams); // WARNING! This data is hard coded now, should be on argv
void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    const double max_val = 250; // Value converted to this if reading is inf
    double max_laser_range = (double) msg->range_max; // Longest range according to ROS message
    // Takes a ros laserscan message and converts it to a falkolib laserscan
    // Must convert laser scan ranges from ros to a vector
    std::vector<double> r(msg->ranges.size());
    for (int i = 0; i < (msg->ranges.size()); ++i) {
        double val = (double) msg->ranges[i]; // Get the value
        r[i] = (val > max_laser_range) ? max_val : val; // Change elements above max range to max range
    }
    scan_data.fromRanges(r);
}



int main(int argc, char *argv[])
{
    // Names
    const char node_name[] = "keypoint_extractor";
    const char laser_topic[] = "/scan";

    // Init ROS
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    ROS_INFO("Node: %s running", node_name);

    // Create a laser subscriber
    // Message queue is set to 1 to only use callback on latest message
    ros::Subscriber laser_sub = n.subscribe(laser_topic, 1, laser_callback);

    // Prepare feature extraction library
    // Feature extraction parameters
    falkolib::FALKOExtractor fe;
	fe.setMinExtractionRange(0.1);
	fe.setMaxExtractionRange(6);
	fe.enableSubbeam(true);
	fe.setNMSRadius(0.1);
	fe.setNeighB(0.07);
	fe.setBRatio(2.5);
	fe.setGridSectors(16);

    // Set scan data to be empty for the first read of keypoints
    // If this is not set, falkolib extract will segfault on first run
    std::vector<double> temp(360, 0);
    scan_data.fromRanges(temp);

    // Main loop
    // Reads laser data, extracts keypoints and publishes them as points
    ros::Rate rate(0.5);
    while (ros::ok())
    {
        ros::spinOnce(); // Get subscription readings

        // Extract features
        std::vector<falkolib::FALKO> keypoints;
        fe.extract(scan_data, keypoints);

        // Print number of points
        ROS_INFO("Found %d keypoints", (int)keypoints.size());

        rate.sleep();
    }

    return 0;
}
