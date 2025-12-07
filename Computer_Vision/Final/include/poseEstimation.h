#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H

#include <opencv2/highgui.hpp>
#include <Eigen/Dense>

#include <vector>


// Lifts a 2D point from the RGB image to 3D
//
// Inputs:
// point2d  - the 2D point to be lifted
// cameraIntrinsics - vector containing focal lengths and optical center coords
//
// Returns:
// The 3D point as an Eigen::Vector3d
Eigen::Vector3d lift2D23D(cv::Point2f point2d, float depth, 
    std::vector<float> cameraIntrinsics={525.0, 525.0, 319.5, 239.5});


// Interpolates the depth using bilear interpolation
//
// Inputs:
// depthImage - the source image
// point2d    - the 2D point to get the depth for

// Returns:
// The depth value in meters as a float
float getDepth(const cv::Mat& depthImage, cv::Point2f point2d);


// Computes the centroid of a set of 3D points
//
// Inputs:
// points3d - vector of 3D points
//
// Returns:
// The centroid of the points as an Eigen::Vector3d
Eigen::Vector3d computeCentroid(const std::vector<Eigen::Vector3d>& points3d);


// Computes the centralized vector of a set of 3D points
//
// Inputs:
// points3d - vector of 3D points
// centroid - the centroid of the points
void centralizedVectors(const std::vector<Eigen::Vector3d>& pointsIn,
    const Eigen::Vector3d& centroid, std::vector<Eigen::Vector3d>& pointsOut);


// Computes the covariance matrix of two sets of centralized vectors
//
// Inputs:
// centralizedVectors1 - first set of centralized vectors
// centralizedVectors2 - second set of centralized vectors
//
// Returns:
// The 3x3 covariance matrix as an Eigen::Matrix3d
Eigen::Matrix3d findCovariance(const std::vector<Eigen::Vector3d>& centralizedVectors1,
    const std::vector<Eigen::Vector3d>& centralizedVectors2);


// Computes the best rotation matrix for given covariance between two sets of points
// 
// Inputs:
// covariance - 3x3 covariance matrix
//
// Returns:
// The brst fit rotation matrix as an Eigen::Matrix3d
Eigen::Matrix3d findRotation(const Eigen::Matrix3d& covariance);


// Computes the thranslation vector given two points and a rotaion
//
// Inputs:
// centroid1 - first 3d point (centroid of the first set of points)
// centroid2 - second 3d point (centroid of the second set of points)
// rotation  - previously computed rotation matrix
// 
// Returns:
// The translation vector as an Eigen::Vector3d
Eigen::Vector3d findTranslation(const Eigen::Vector3d& centroid1, 
    const Eigen::Vector3d& centroid2, const Eigen::Matrix3d& rotation);

    
// Refines the rotation and translation using an iterative method that filters outliers
//
// Inputs:
// points3d1  - first set of 3D points
// points3d2  - second set of 3D points
// iterations - number of refinement iterations
// rotationOut    - the refined rotation matrix
// translationOut - the refined translation vector
void refinedRotationTranslation(const std::vector<Eigen::Vector3d>& points3d1,
    const std::vector<Eigen::Vector3d>& points3d2, Eigen::Matrix3d& rotationOut,
    Eigen::Vector3d& translationOut, int iterations);


// Combines rotation and translation into a 4x4 transformation matrix
//
// Inputs:
// rotation    - the rotation matrix
// translation - the translation vector
// Returns:
// The 4x4 transformation matrix as an Eigen::Matrix4d
Eigen::Matrix4d makeTransform(const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation);
    
#endif