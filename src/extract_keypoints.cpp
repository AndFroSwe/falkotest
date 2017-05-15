/* 
 * File: extract_keypoints.cpp
 * Author: Andreas Fröderberg
 * Description: Subscibes to a laser scan topic and extracts keypoints. These 
 * are then published.
 */

// ROS
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

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
    const char keypoint_topic[] = "/keypoints";
    const char frame_name[] = "base_link";

    // Init ROS
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    ROS_INFO("Node: %s running", node_name);

    // Create a laser subscriber
    // Message queue is set to 1 to only use callback on latest message
    ros::Subscriber laser_sub = n.subscribe(laser_topic, 1, laser_callback);

    // Publish extracted points
    ros::Publisher keypoint_pub = n.advertise<visualization_msgs::Marker>(keypoint_topic, 1000);

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
    ros::Rate rate(1);
    while (ros::ok())
    {
        ros::spinOnce(); // Get subscription readings

        // Extract features
        std::vector<falkolib::FALKO> keypoints;
        fe.extract(scan_data, keypoints);
        ROS_INFO("Found %d keypoints", (int)keypoints.size());

        // Publish keypoints
        if ((int) keypoints.size() > 0)
        {
            ros::Time t = ros::Time::now();
            // Create marker message
            visualization_msgs::Marker vp;
            vp.header.stamp = t;
            vp.header.frame_id = frame_name;
            vp.ns = "falko";
            vp.action = visualization_msgs::Marker::ADD;
            vp.pose.orientation.w = 1.0;
            vp.id = 0;
            vp.type = visualization_msgs::Marker::POINTS;
            vp.scale.x = 0.2;
            vp.scale.y = 0.2;
            vp.color.g = 1.0f;
            vp.color.a = 1.0;
            ros::Duration d(1.0);
            vp.lifetime = d;

            // Add all keypoints to array
            for (int i = 0; i < (int)keypoints.size(); ++i) {
                ROS_INFO("Adding");
                // There are keypoints, extract them
                geometry_msgs::Point kp; // Create the pub object

                kp.x = keypoints[i].point[0]; // point is of type Eigen::Matrix
                kp.y = keypoints[i].point[1]; // point is of type Eigen::Matrix
                kp.z = 0;

                vp.points.push_back(kp);
            }

            // Publish
            keypoint_pub.publish(vp);
        }

        // Print number of points

        rate.sleep();
    }

    return 0;
}
