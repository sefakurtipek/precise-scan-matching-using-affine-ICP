#include "functions.h"
#include "functions.cpp"
#include "LaserScanProcessor.h"

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "laser_scan_processor");
    // Create an object of the LaserScanProcessor class
    LaserScanProcessor processor;
    // Spin the ROS node to process callback functions
    ros::spin();

    return 0;
}
