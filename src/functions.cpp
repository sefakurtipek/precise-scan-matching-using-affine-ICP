#include "functions.h"
#include "LaserScanProcessor.h"

bool convertScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             laser_geometry::LaserProjection& projector) {
    // First, make sure the cloud is empty
    cloud->clear();
    // Create a PointCloud2 pointer to store the converted data
    sensor_msgs::PointCloud2 cloud2;  
    // Use the passed-in projector instead of projector_
    projector.projectLaser(*scan, cloud2);
    // Convert to PCL data type
    pcl::fromROSMsg(cloud2, *cloud);
    // Check if the conversion was successful
    if (cloud->points.empty()) {
        return false;
    } else {
        return true;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // PassThrough filter for cropping the point cloud
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 2.0);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2.0, 2.0);
    pass.filter(*cloud_filtered);
    return cloud_filtered;
}

Eigen::Matrix4f getInitialTransformationMatrix(tf::Vector3 translation) {
    float Yaw_degrees = translation.z();
    translation.setZ(0.0);
    // Convert Yaw from degrees to radians
    float Yaw_radians = Yaw_degrees * M_PI / 180.0f;
    // Create a rotation matrix (assuming Z-axis rotation)
    Eigen::Matrix3f rotationMatrix;
    rotationMatrix = Eigen::AngleAxisf(Yaw_radians, Eigen::Vector3f::UnitZ());
    // Convert tf::Vector3 to Eigen::Vector3f for translation
    Eigen::Vector3f translationEigen(translation.x(), translation.y(), translation.z());
   // Combine rotation and translation into an affine transformation matrix
    Eigen::Affine3f initialTransformation = Eigen::Translation3f(translationEigen) * rotationMatrix;
    // Convert to 4x4 matrix
    Eigen::Matrix4f initialTransformationMatrix = initialTransformation.matrix();
    return initialTransformationMatrix;
}

void findNearestNeighbors(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& modelSet,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& dataSet,
    pcl::PointCloud<pcl::PointXYZ>& correspondenceCloud) {

    std::vector<pcl::PointXYZ> nearestNeighbors;

    correspondenceCloud.clear();
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(dataSet); // I am searching for scan dataset to find best nearest two points from model set

    for (const auto& point : *modelSet) {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            nearestNeighbors.push_back(
                dataSet->points[pointIdxNKNSearch[0]]
            );
        }
    }

    for (const auto& nn : nearestNeighbors) {
        correspondenceCloud.points.push_back(nn);
    }
    correspondenceCloud.header = dataSet->header;
    correspondenceCloud.width = correspondenceCloud.points.size();
    correspondenceCloud.height = 1;
    correspondenceCloud.is_dense = true;
    
/*     ROS_INFO("modelSet size: %d", modelSet->size());
    ROS_INFO("dataSet size: %d", dataSet->size());
    ROS_INFO("correspondenceCloud size: %d", correspondenceCloud.size()); */
}

void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const tf::Vector3& translation, double theta) {
    double theta_radians = theta * M_PI / 180.0;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // Set the rotation part of the transformation around the Z axis
    transform.rotate(Eigen::AngleAxisf(theta_radians, Eigen::Vector3f::UnitZ()));
    // Set the translation part of the transformation
    transform.translation() << translation.x(), translation.y(), 0.0f;
    // Transform the point cloud using the transformation matrix
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    cloud = transformed_cloud;
}

Eigen::Matrix4f updateTransformationUsingPLMetricAndCorrentropy(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& modelDataPoints,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& correspondenceCloudPtr,
    const Eigen::Matrix4f& previousTransformation) {
    std::vector<Eigen::Matrix<float, 2, 6>> QMatrixVector = constructQMatrices(modelDataPoints); // MODEL DATA
    Eigen::Matrix<float, 6, 1> PVector = constructPVector(previousTransformation); // UNKONWN AFFINE TRANSFORMATION AND TRANSLATION VALUES
    std::vector<Eigen::Matrix<float, 2, 1>> taoVectors = constructTaoVectors(correspondenceCloudPtr); // SCAN DATA CORRESPONDENCES

    float sigma = calculateStandardDeviation(modelDataPoints, correspondenceCloudPtr);
    float sigmaMax = 0.5f;
    float sigmaMin = 0.1f;
    if(sigma > sigmaMax){
        sigma = sigmaMax;
        //ROS_INFO("sigma: %lf", sigma);
    }
    if(sigma < sigmaMin){
        sigma = sigmaMin;
        //ROS_INFO("sigma: %lf", sigma);
    }
    //ROS_INFO("sigma: %lf", sigma);
    std::vector<float> deltaVector = calculateDeltaI(QMatrixVector, PVector, taoVectors, sigma);
    Eigen::Matrix<float, 6, 6> GMatrix = constructGMatrix(QMatrixVector, deltaVector);
    Eigen::Matrix<float, 1, 6> hVector = constructHVector(taoVectors, QMatrixVector, deltaVector);
    Eigen::Matrix<float, 6, 1> NewPVector = calculateNewPVector(hVector, GMatrix);
    Eigen::Matrix4f newTransformation = constructNewTransformation(NewPVector);
    return newTransformation;
}

std::vector<Eigen::Matrix<float, 2, 6>> constructQMatrices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& modelDataPoints) {
    std::vector<Eigen::Matrix<float, 2, 6>> Q_matrices;
    for (const auto& point : modelDataPoints->points) {
        Eigen::Matrix<float, 2, 6> Q;
        float x_ix = point.x;
        float x_iy = point.y;

        Q << x_ix, x_iy, 0, 0, 1, 0,
             0, 0, x_ix, x_iy, 0, 1;

        Q_matrices.push_back(Q);
    }
    return Q_matrices;
}


Eigen::Matrix<float, 6, 1> constructPVector(const Eigen::Matrix4f& transformation) {
    Eigen::Matrix<float, 6, 1> P;
    // Extract the values from the previous transformation matrix
    float a11 = transformation(0, 0);
    float a12 = transformation(0, 1);
    float a21 = transformation(1, 0);
    float a22 = transformation(1, 1);
    float tx  = transformation(0, 3);
    float ty  = transformation(1, 3);
    // Construct the P vector
    P << a11, a12, a21, a22, tx, ty;
    return P;
}

std::vector<Eigen::Matrix<float, 2, 1>> constructTaoVectors(const pcl::PointCloud<pcl::PointXYZ>::Ptr& correspondenceCloudPtr) {
    std::vector<Eigen::Matrix<float, 2, 1>> taoVectors;

    // Iterate over each point in the correspondence cloud to create a 2x1 matrix for each
    for (const auto& point : correspondenceCloudPtr->points) {
        Eigen::Matrix<float, 2, 1> tao;

        // Get the x and y values from each point in the correspondence cloud
        float x = point.x;
        float y = point.y;

        // Construct 2x1 matrix and add it to the vector
        tao << x, y;
        taoVectors.push_back(tao);
    }

    return taoVectors;
}

float calculateStandardDeviation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2) {
    if (cloud1->points.size() != cloud2->points.size()) {
        std::cout << "cloud1->points.size(): " << cloud1->points.size() << " cloud2->points.size(): " << cloud2->points.size() << std::endl;
        throw std::runtime_error("Clouds must have the same number of points : calculateStandardDeviation");
    }
    // Calculate squared differences for each component (x and y only)
    float sumSquaredDiffX = 0.0f;
    float sumSquaredDiffY = 0.0f;
    for (size_t i = 0; i < cloud1->points.size(); ++i) {
        sumSquaredDiffX += std::pow(cloud1->points[i].x - cloud2->points[i].x, 2);
        sumSquaredDiffY += std::pow(cloud1->points[i].y - cloud2->points[i].y, 2);
    }
    // Compute the mean squared differences
    float meanSquaredDiffX = sumSquaredDiffX / cloud1->points.size();
    float meanSquaredDiffY = sumSquaredDiffY / cloud1->points.size();
    // Compute the standard deviation for each component
    float stdDevX = std::sqrt(meanSquaredDiffX);
    float stdDevY = std::sqrt(meanSquaredDiffY);
    // Compute the combined standard deviation (assuming independence)
    float combinedStdDev = std::sqrt(stdDevX * stdDevX + stdDevY * stdDevY) / std::sqrt(2);
    return combinedStdDev;
}

std::vector<float> calculateDeltaI(
    const std::vector<Eigen::Matrix<float, 2, 6>>& QMatrixVector,
    const Eigen::Matrix<float, 6, 1>& PVector,
    const std::vector<Eigen::Matrix<float, 2, 1>>& TaoMatrixVector,
    float sigma) {
    std::vector<float> deltas;
    for (size_t i = 0; i < QMatrixVector.size(); ++i) {
        // Extract individual matrices and vectors
        const Eigen::Matrix<float, 2, 6>& Qi = QMatrixVector[i];
        const Eigen::Matrix<float, 2, 1>& tao_i = TaoMatrixVector[i];
        // Calculate the term inside the exponential
        Eigen::Matrix<float, 2, 1> term = Qi * PVector - tao_i;
        //float exponent = -0.5 * (term.transpose() * Vi * term)(0,0) / (sigma * sigma);
        float exponent = -0.5 * (term.transpose() * term)(0,0) / (sigma * sigma);
        // Calculate delta_i and push back to deltas vector
        deltas.push_back(std::exp(exponent));
        //ROS_INFO("deltaI index = %d , deltaIValue: %d", i, std::exp(exponent));
    }
    return deltas;
}

Eigen::Matrix<float, 6, 6> constructGMatrix(
    const std::vector<Eigen::Matrix<float, 2, 6>>& QMatrixVector,
    const std::vector<float>& deltaVector) {
    Eigen::Matrix<float, 6, 6> G = Eigen::Matrix<float, 6, 6>::Zero();  // Initialize G matrix
    for (size_t i = 0; i < QMatrixVector.size(); ++i) {
        //G += QMatrixVector[i].transpose() * VMatrixVector[i] * QMatrixVector[i] * deltaVector[i];
        G += QMatrixVector[i].transpose() * QMatrixVector[i];
        //G += QMatrixVector[i].transpose() * QMatrixVector[i] * deltaVector[i];
    }
    return G;
}

Eigen::Matrix<float, 1, 6> constructHVector(
    const std::vector<Eigen::Matrix<float, 2, 1>>& taoVectors,
    const std::vector<Eigen::Matrix<float, 2, 6>>& QMatrixVector,
    const std::vector<float>& deltaVector) {
    Eigen::Matrix<float, 1, 6> h = Eigen::Matrix<float, 1, 6>::Zero();  // Initialize h vector
    for (size_t i = 0; i < taoVectors.size(); ++i) {
        //h += taoVectors[i].transpose() * VMatrixVector[i] * QMatrixVector[i] * deltaVector[i];
        h += taoVectors[i].transpose() * QMatrixVector[i];
        //h += taoVectors[i].transpose() * QMatrixVector[i] * deltaVector[i];

    }
    return h;
}

Eigen::Matrix<float, 6, 1> calculateNewPVector(
    const Eigen::Matrix<float, 1, 6>& hVector,
    const Eigen::Matrix<float, 6, 6>& GMatrix) {

    double lambda = 1e-4; // Regularization parameter
    // Apply regularization directly
    Eigen::Matrix<float, 6, 6> regularized_GMatrix = GMatrix + Eigen::Matrix<float, 6, 6>::Identity() * lambda;

    // Check if the regularized matrix is invertible, this is more of a safety check
    if(regularized_GMatrix.determinant() == 0) {
        ROS_ERROR("Regularized GMatrix is not invertible");
        //throw std::runtime_error("Regularized GMatrix is not invertible");
    }

    // Calculate p = (h * G^-1)^T
    Eigen::Matrix<float, 6, 1> p = (hVector * regularized_GMatrix.inverse()).transpose();
    return p;
}

Eigen::Matrix4f constructNewTransformation(const Eigen::Matrix<float, 6, 1>& NewPVector) {
    Eigen::Matrix4f newTransformation = Eigen::Matrix4f::Identity();  // Initialize as identity matrix
    // Assign the affine transformation parameters from NewPVector
    newTransformation(0, 0) = NewPVector(0); // a11
    newTransformation(0, 1) = NewPVector(1); // a12
    newTransformation(1, 0) = NewPVector(2); // a21
    newTransformation(1, 1) = NewPVector(3); // a22
    newTransformation(0, 3) = NewPVector(4); // tx
    newTransformation(1, 3) = NewPVector(5); // ty
    return newTransformation;
}

double calculateAbsDifferenceNorm(const Eigen::Matrix4f& matrix1, const Eigen::Matrix4f& matrix2) {
    // Calculate the absolute difference
    Eigen::Matrix4f absDiff = (matrix1 - matrix2).cwiseAbs();

    // Return the Frobenius norm of the absolute difference
    return absDiff.norm();
}
struct PointXYZComparator {
    bool operator() (const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs) const {
        if (lhs.x < rhs.x) return true;
        if (lhs.x > rhs.x) return false;
        if (lhs.y < rhs.y) return true;
        if (lhs.y > rhs.y) return false;
        return lhs.z < rhs.z;
    }
};

size_t countUniquePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::set<pcl::PointXYZ, PointXYZComparator> uniquePoints;
    for (const auto& point : cloud->points) {
        uniquePoints.insert(point);
    }
    return uniquePoints.size();
}

double calculatePointCloudDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2) {
    if (cloud1->size() != cloud2->size()) {
        throw std::runtime_error("Point clouds do not have the same number of points");
    }

    double distanceSum = 0.0;
    for (size_t i = 0; i < cloud1->size(); ++i) {
        const pcl::PointXYZ &pt1 = (*cloud1)[i];
        const pcl::PointXYZ &pt2 = (*cloud2)[i];
        distanceSum += sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
    }

    return distanceSum;
}



pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleVShapeModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& vShapeModel, int num_points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    float step = static_cast<float>(vShapeModel->size() - 1) / (num_points - 1);
    for (int i = 0; i < num_points; ++i) {
        downsampled->push_back(vShapeModel->points[static_cast<int>(i * step)]);
    }
    return downsampled;
}

// Function to check if all points from the downsampled vShapeModel can find corresponding points in the laserScanCloud
bool allPointsHaveCorrespondence(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transformedModel,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& laserScanCloud,
                                 const float maxDistance,
                                 pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree) {
    for (const auto& point : transformedModel->points) {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if (kdtree.radiusSearch(point, maxDistance, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0) {
            return false;
        }
    }
    return true;
}

bool findCorrespondingPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& templateCloud, 
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr& targetCloud, 
                             const pcl::PointXYZ& targetPoint, 
                             float max_distance) {
    // Transform the template cloud based on the target point
    // Check for corresponding points in the target cloud within the max_distance
    // Return true if all points in the template have a corresponding point in the target
    // Otherwise, return false
    // ...
    return false;
}



// Function to calculate the score based on the sum of squared distances from
// each point in the transformedModel to its nearest neighbor in the laserScanCloud
float calculateScore(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transformedModel,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& laserScanCloud) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(laserScanCloud);

    float score = 0.0f;
    for (const auto& point : transformedModel->points) {
        std::vector<int> nearestNeighborIndex(1);
        std::vector<float> nearestNeighborSquaredDistance(1);

        // If a nearest neighbor is found in the laserScanCloud
        if (kdtree.nearestKSearch(point, 1, nearestNeighborIndex, nearestNeighborSquaredDistance) > 0) {
            // Add the squared distance to the score
            score += nearestNeighborSquaredDistance[0];
        } else {
            // If no nearest neighbor is found, add a large penalty to the score
            // This depends on the scale of your point cloud
            score += std::numeric_limits<float>::max();
        }
    }

    return score;
}

Eigen::Matrix4f calculateTransformationMatrix(const pcl::PointXYZ& targetPoint, 
                                              const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampledModel) {
    Eigen::Vector3f translation(targetPoint.x - downsampledModel->points[0].x, targetPoint.y - downsampledModel->points[0].y, 0.0f);
    Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();
    transformationMatrix(0,3) = translation.x();
    transformationMatrix(1,3) = translation.y();
    return transformationMatrix;
}
