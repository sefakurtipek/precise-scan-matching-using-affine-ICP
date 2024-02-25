#include "LaserScanProcessor.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>


#include <Eigen/Geometry>
#include <Eigen/Core>

#include <vector>
#include <queue>
#include <numeric>
#include <cmath>
#include <unordered_set>
#include <set>

bool convertScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             laser_geometry::LaserProjection& projector);
pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
Eigen::Matrix4f getInitialTransformationMatrix(tf::Vector3 translation);
void findNearestNeighbors(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& modelSet,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& dataSet,
    pcl::PointCloud<pcl::PointXYZ>& correspondenceCloud);
void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const tf::Vector3& translation, double theta);
Eigen::Matrix4f updateTransformationUsingPLMetricAndCorrentropy(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& modelDataPoints,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& correspondenceCloudPtr,
    const Eigen::Matrix4f& previousTransformation);
std::vector<Eigen::Matrix<float, 2, 6>> constructQMatrices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transformedDataPoints);
Eigen::Matrix<float, 6, 1> constructPVector(const Eigen::Matrix4f& transformation);
std::vector<Eigen::Matrix<float, 2, 1>> constructTaoVectors(const pcl::PointCloud<pcl::PointXYZ>::Ptr& correspondenceCloudPtr);
std::vector<Eigen::Matrix<float, 2, 2>> constructVMatrix(const std::vector<Eigen::Vector3f>& normals);
float calculateStandardDeviation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2);
std::vector<float> calculateDeltaI(
    const std::vector<Eigen::Matrix<float, 2, 6>>& QMatrixVector,
    const Eigen::Matrix<float, 6, 1>& PVector,
    const std::vector<Eigen::Matrix<float, 2, 1>>& taoVectors,
    float sigma);
Eigen::Matrix<float, 6, 6> constructGMatrix(
    const std::vector<Eigen::Matrix<float, 2, 6>>& QMatrixVector,
    const std::vector<float>& deltaVector);
Eigen::Matrix<float, 1, 6> constructHVector(
    const std::vector<Eigen::Matrix<float, 2, 1>>& taoVectors,
    const std::vector<Eigen::Matrix<float, 2, 6>>& QMatrixVector,
    const std::vector<float>& deltaVector);
Eigen::Matrix<float, 6, 1> calculateNewPVector(const Eigen::Matrix<float, 1, 6>& hVector, const Eigen::Matrix<float, 6, 6>& GMatrix);
Eigen::Matrix4f constructNewTransformation(const Eigen::Matrix<float, 6, 1>& NewPVector);
double calculateAbsDifferenceNorm(const Eigen::Matrix4f& matrix1, const Eigen::Matrix4f& matrix2);
size_t countUniquePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
double calculatePointCloudDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2);

pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleVShapeModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& vShapeModel, int num_points);
bool allPointsHaveCorrespondence(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transformedModel,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& laserScanCloud,
                                 const float maxDistance,
                                 pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree);
bool findCorrespondingPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& templateCloud, 
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr& targetCloud, 
                             const pcl::PointXYZ& targetPoint, 
                             float max_distance);
float calculateScore(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transformedModel,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& laserScanCloud);
Eigen::Matrix4f calculateTransformationMatrix(const pcl::PointXYZ& targetPoint, 
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampledModel);