#include "LaserScanProcessor.h"
#include "functions.h"

LaserScanProcessor::LaserScanProcessor() :  nh_(), listener_(){ 

    nh_.getParam("initial_guess_X", initial_guess_X);
    nh_.getParam("initial_guess_Y", initial_guess_Y);
    nh_.getParam("initialGuess_yaw_degrees", initialGuess_yaw_degrees);
    nh_.getParam("maxIteration", maxIteration);
    nh_.getParam("epsilon", epsilon);
    nh_.getParam("sumOfDifferenceThreshold", sumOfDifferenceThreshold);
    nh_.getParam("vShapeModelName", vShapeModelName);
    nh_.getParam("laserFrameName", laserFrameName);
    nh_.getParam("laserScanName", laserScanName);
    nh_.getParam("mapFrameName", mapFrameName);
    nh_.getParam("absolute_path", absolute_path);

    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(laserScanName, 1000, &LaserScanProcessor::scanCallback, this);
    preprocessed_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/preprocessed_points", 1);
    correspondencePC_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/correspondenceCloud", 1);
    vShapedModel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/vShapedModelCloud", 1);
    vShapedModelAfterAffine_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/vShapedModelAffineTransformed", 1);
    finalTransformedModel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/finalTransformedModel", 1);
    smallStepsTransformedModel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/smallStepsTransformedModel", 1);
   
}

tf::Vector3 LaserScanProcessor::transformPoint(float X, float Y, const std::string& target_frame, const std::string& source_frame) {
    tf::StampedTransform transform;
    try {
        listener_.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(10.0));
        listener_.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        throw;
    }
    float Z = 0;
    tf::Vector3 original_point(X, Y, Z);
    tf::Vector3 transformed_point = transform * original_point;
    //ROS_INFO("Transformed Point: x = %f, y = %f, z = %f", transformed_point.x(), transformed_point.y(), transformed_point.z());
    return transformed_point;
}

Eigen::Matrix4f LaserScanProcessor::getTransform(const std::string& target_frame, const std::string& source_frame) {
    tf::StampedTransform transform;
    try {
        listener_.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(10.0));
        listener_.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        throw;
    }
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(transform, eigen_transform);
    return eigen_transform;
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::Matrix4f> LaserScanProcessor::findBestTransformedModelPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& vShapeModel,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& laserScanCloud,
    const sensor_msgs::LaserScan::ConstPtr& laserScan) {

    size_t maxUniquePointCount = 0;
    double minSumOfDistance = 999.0;
    double maxScore = -1.0;
    std::pair<int, int> bestIndices = {0, 0};
    std::pair<int, int> indicesForMaxUniqueCorrespondences = {0, 0};
    std::pair<int, int> indicesForMinDistance = {0, 0};

    // Define weights for each metric
    double weightUniqueCount = 0.2;
    double weightSumOfDistance = 0.8;

    int translationStepLimit = 5;
    for (int i = -translationStepLimit; i < translationStepLimit; ++i) {
        for (int j = -translationStepLimit; j < translationStepLimit; ++j) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr vShapeModel_stepTransform(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*vShapeModel, *vShapeModel_stepTransform);

            Eigen::Matrix4f translationMatrix = Eigen::Matrix4f::Identity();
            translationMatrix(0, 3) = 0.1 * i;
            translationMatrix(1, 3) = 0.1 * j;

            pcl::transformPointCloud(*vShapeModel, *vShapeModel_stepTransform, translationMatrix);

            sensor_msgs::PointCloud2 finalTransformedModelMsg2;
            pcl::toROSMsg(*vShapeModel_stepTransform, finalTransformedModelMsg2);
            finalTransformedModelMsg2.header.frame_id = laserFrameName;
            finalTransformedModelMsg2.header.stamp = ros::Time::now();;
            smallStepsTransformedModel_pub_.publish(finalTransformedModelMsg2);

            // finding nearest neighbors and counting unique points
            pcl::PointCloud<pcl::PointXYZ>::Ptr correspondenceCloudAfterMovePtr(new pcl::PointCloud<pcl::PointXYZ>());
            findNearestNeighbors(vShapeModel_stepTransform, laserScanCloud, *correspondenceCloudAfterMovePtr); // FIND CORRESPONDENCES
            size_t uniquePointCountAfterMove = countUniquePoints(correspondenceCloudAfterMovePtr);

            double sumOfDistance = calculatePointCloudDistance(vShapeModel_stepTransform, correspondenceCloudAfterMovePtr);
            if(sumOfDistance < minSumOfDistance)
            {
                minSumOfDistance = sumOfDistance;
                indicesForMinDistance = {i, j};
                if(minSumOfDistance > 100.0) // there is no sumOfDistance
                {
                    indicesForMinDistance = {0, 0};
                }
            }
            if (uniquePointCountAfterMove > maxUniquePointCount) {
                maxUniquePointCount = uniquePointCountAfterMove;
                indicesForMaxUniqueCorrespondences = {i, j};
                if(maxUniquePointCount < 10) // there is no nearest point from scan close to find maxIndices
                {
                    indicesForMaxUniqueCorrespondences = {0, 0};
                }
            }

            // Normalize metrics
            double maxPossibleUniqueCount = (*vShapeModel).size();
            double normalizedUniqueCount = static_cast<double>(uniquePointCountAfterMove) / maxPossibleUniqueCount;
            double normalizedSumOfDistance = 1.0 / (1.0 + sumOfDistance);

            // Combine metrics
            double score = (normalizedUniqueCount * weightUniqueCount) + (normalizedSumOfDistance * weightSumOfDistance);

            // Update best pair if necessary
            if (score > maxScore) {
                maxScore = score;
                bestIndices = {i, j};
                if(maxScore == 0.0)
                {
                    bestIndices = {0, 0};   
                }
            }

        }
    }
    std::cout << "********************************************************************************************************" << std::endl;
    ROS_INFO("maxScore: %lf", maxScore);
    ROS_INFO("minSumOfDistance: %lf", minSumOfDistance);
    ROS_INFO("maxUniquePointCount: %d", maxUniquePointCount);
    ROS_INFO("bestIndices.first: %d, bestIndices.second: %d", bestIndices.first, bestIndices.second);    
    ROS_INFO("indicesForMinDistance.first: %d, indicesForMinDistance.second: %d", indicesForMinDistance.first, indicesForMinDistance.second);
    ROS_INFO("indicesForMaxUniqueCorrespondences.first: %d, indicesForMaxUniqueCorrespondences.second: %d", indicesForMaxUniqueCorrespondences.first, indicesForMaxUniqueCorrespondences.second);

    // Apply the final transformation with maxIndices to a fresh copy of vShapeModel
    pcl::PointCloud<pcl::PointXYZ>::Ptr vShapeModel_finalTransformed(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f finalTranslationMatrix = Eigen::Matrix4f::Identity();
    finalTranslationMatrix(0, 3) = 0.1 * bestIndices.first;
    finalTranslationMatrix(1, 3) = 0.1 * bestIndices.second;
    pcl::transformPointCloud(*vShapeModel, *vShapeModel_finalTransformed, finalTranslationMatrix);
    return std::make_pair(vShapeModel_finalTransformed, finalTranslationMatrix);
}

void LaserScanProcessor::scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan) {
    ROS_INFO("CALLBACI IS RUNNING");
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserScanCloud(new pcl::PointCloud<pcl::PointXYZ>());
    if(!convertScanToPointCloud(laserScan, laserScanCloud, projector_)) { ROS_ERROR("Failed to convert laser scan to point cloud");return;}
    laserScanCloud = preprocessPointCloud(laserScanCloud);

    sensor_msgs::PointCloud2 preprocessed_output;
    pcl::toROSMsg(*laserScanCloud, preprocessed_output);
    preprocessed_output.header.frame_id = laserFrameName;
    preprocessed_output.header.stamp = ros::Time::now();
    preprocessed_points_pub_.publish(preprocessed_output);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZ>::Ptr vShapeModel(new pcl::PointCloud<pcl::PointXYZ>);
    std::string fullPath_vShapedModel = absolute_path + vShapeModelName;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fullPath_vShapedModel, *vShapeModel) == -1 )
        {PCL_ERROR ("Couldn't read file \n");return;}

    float initial_guess_yaw_radian = (M_PI/180) * initialGuess_yaw_degrees;
    Eigen::Matrix4f transformMatrixTF = LaserScanProcessor::getTransform(laserFrameName, mapFrameName);
    tf::Vector3 transformedPoint = transformPoint(initial_guess_X, initial_guess_Y, laserFrameName, mapFrameName);
    Eigen::Matrix4f initialTransformationMatrix = getInitialTransformationMatrix(transformedPoint);
    Eigen::Matrix4f rotationYawRadianZ = Eigen::Matrix4f::Identity();
    rotationYawRadianZ.block<3,3>(0,0) = Eigen::AngleAxisf(initial_guess_yaw_radian, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    Eigen::Matrix3f rotationFromTF = transformMatrixTF.block<3,3>(0,0);
    Eigen::Matrix4f rotationTF_4x4 = Eigen::Matrix4f::Identity();
    rotationTF_4x4.block<3,3>(0,0) = rotationFromTF;
    //Eigen::Matrix4f combinedMatrix = initialTransformationMatrix * rotationTF_4x4 * rotationYawRadianZ;
    Eigen::Matrix4f combinedMatrix = rotationTF_4x4 * rotationYawRadianZ;

    pcl::transformPointCloud(*vShapeModel, *vShapeModel, combinedMatrix); // initial pose is given
/* 
    sensor_msgs::PointCloud2 vShapeModel_pc2;
    pcl::toROSMsg(*vShapeModel, vShapeModel_pc2);
    vShapeModel_pc2.header.frame_id = laserFrameName;
    vShapeModel_pc2.header.stamp = ros::Time::now();
    vShapedModel_pub_.publish(vShapeModel_pc2); */

    int modelSampleNum = 10;
    const float max_distance = 0.1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledVShapeModel = downsampleVShapeModel(vShapeModel, modelSampleNum); // TO DO : TUNE NUMBER
    
    sensor_msgs::PointCloud2 vShapeModel_pc2;
    pcl::toROSMsg(*downsampledVShapeModel, vShapeModel_pc2);
    vShapeModel_pc2.header.frame_id = laserFrameName;
    vShapeModel_pc2.header.stamp = ros::Time::now();
    vShapedModel_pub_.publish(vShapeModel_pc2);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(laserScanCloud);

    Eigen::Matrix4f bestTransformation;
    float bestScore = std::numeric_limits<float>::infinity(); // Initialize with worst score

    // Iterate through points in laserScanCloud to find a match
    //for (const auto& point : laserScanCloud->points)
    //int startPointsIdx = laserScanCloud->points.size()-50;
    int startPointsIdx = 0;
    for (int i = startPointsIdx; i< laserScanCloud->points.size();i++) {
    //for (int i = 0; i< 50;i++) {
        // Calculate the transformation needed to align the first point of the downsampled model with the current point
        Eigen::Matrix4f transformation = calculateTransformationMatrix(laserScanCloud->points[i], downsampledVShapeModel);

        // Transform the downsampled model with the current hypothesis
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedModel(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*downsampledVShapeModel, *transformedModel, transformation);

        sensor_msgs::PointCloud2 finalTransformedModelMsg2;
        pcl::toROSMsg(*transformedModel, finalTransformedModelMsg2);
        finalTransformedModelMsg2.header.frame_id = laserFrameName;
        finalTransformedModelMsg2.header.stamp = ros::Time::now();;
        smallStepsTransformedModel_pub_.publish(finalTransformedModelMsg2);      

        // Check for correspondences in the laserScanCloud
        if (allPointsHaveCorrespondence(transformedModel, laserScanCloud, max_distance, kdtree)) {
            // Calculate score for this transformation
            float score = calculateScore(transformedModel, laserScanCloud);

            // Update the best transformation if the current score is lower (better)
            if (score < bestScore) {
                bestScore = score;
                bestTransformation = transformation;
            }
        }
    }
    std::cout << "bestScore:"  << bestScore << std::endl;
    float scoreThreshold = 0.01f;
    if(bestScore > scoreThreshold)
    {
        std::cout << "BAD SCORE"<< std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr finalTransformedModel(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*downsampledVShapeModel, *finalTransformedModel, bestTransformation);
    sensor_msgs::PointCloud2 finalTransformedModel_pc2;
    pcl::toROSMsg(*finalTransformedModel, finalTransformedModel_pc2);
    finalTransformedModel_pc2.header.frame_id = laserFrameName;
    finalTransformedModel_pc2.header.stamp = ros::Time::now();
    finalTransformedModel_pub_.publish(finalTransformedModel_pc2);
}