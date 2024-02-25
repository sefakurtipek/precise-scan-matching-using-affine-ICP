#ifndef LASER_SCAN_PROCESSOR_H
#define LASER_SCAN_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include "functions.h"

class LaserScanProcessor {
public:
    LaserScanProcessor();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan);
    tf::Vector3 transformPoint(float X, float Y, const std::string& target_frame, const std::string& source_frame);
    Eigen::Matrix4f getTransform(const std::string& target_frame, const std::string& source_frame);
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::Matrix4f> findBestTransformedModelPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& vShapeModel,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& laserScanCloud,
        const sensor_msgs::LaserScan::ConstPtr& laserScan);
 
private:
    float initial_guess_X;
    float initial_guess_Y;
    float initialGuess_yaw_degrees;
    int maxIteration;
    double epsilon;
    double sumOfDifferenceThreshold;
    std::string vShapeModelName;
    std::string laserFrameName;
    std::string laserScanName;
    std::string mapFrameName;
    std::string absolute_path;
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    ros::Publisher correspondencePC_pub_;
    ros::Publisher preprocessed_points_pub_;
    ros::Publisher vShapedModel_pub_;
    ros::Publisher vShapedModelAfterAffine_pub_;
    ros::Publisher finalTransformedModel_pub_;
    ros::Publisher smallStepsTransformedModel_pub_;

};

#endif // LASER_SCAN_PROCESSOR_H