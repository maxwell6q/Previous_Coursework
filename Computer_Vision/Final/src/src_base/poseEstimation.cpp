#include "poseEstimation.h"

Eigen::Vector3d lift2D23D(cv::Point2f point2d, float depth,
    std::vector<float> cameraIntrinsics)
{
    float fx, fy, cx, cy, depthScale, X, Y, Z;
    
    fx = cameraIntrinsics[0];  // focal length x
    fy = cameraIntrinsics[1];  // focal length y
    cx = cameraIntrinsics[2];  // optical center x
    cy = cameraIntrinsics[3];  // optical center y

    int u = static_cast<int>(point2d.x);
    int v = static_cast<int>(point2d.y);

    Z = depth;
    X = (u - cx) * Z / fx;
    Y = (v - cy) * Z / fy;

    Eigen::Vector3d point3d(X, Y, Z);

    return point3d;
}


float getDepth(const cv::Mat& depthImage, cv::Point2f point)
{
    float x = point.x, y = point.y;
    int x0 = (int)std::floor(x), y0 = (int)std::floor(y);

    // Bounds check
    if (x0 < 0 || y0 < 0 || x0 + 1 >= depthImage.cols || y0 + 1 >= depthImage.rows) 
        return 0.0f;

    // Bilinear interpolation
    float dx = x - x0;
    float dy = y - y0;
    float depth = (1 - dx) * (1 - dy) * depthImage.at<unsigned short>(y0, x0) +
        dx * (1 - dy) * depthImage.at<unsigned short>(y0, x0 + 1) +
        (1 - dx) * dy * depthImage.at<unsigned short>(y0 + 1, x0) +
        dx * dy * depthImage.at<unsigned short>(y0 + 1, x0 + 1);

    if (depth <= 0.0f)
        return 0.0f;
    return depth * (1.0f / 5000.0f); // Convert to meters (depth images scaled by 5000)
}


Eigen::Vector3d computeCentroid(const std::vector<Eigen::Vector3d>& points3d)
{
    Eigen::Vector3d centroid(0.0, 0.0, 0.0);
    if (points3d.empty()) return centroid;

    for (const Eigen::Vector3d& point : points3d)
    {
        centroid += point;
    }
    centroid = centroid / static_cast<double>(points3d.size());

    return centroid;
}


void centralizedVectors(const std::vector<Eigen::Vector3d>& pointsIn,
    const Eigen::Vector3d& centroid, std::vector<Eigen::Vector3d>& pointsOut)
{
    for (const Eigen::Vector3d& point : pointsIn)
    {
        Eigen::Vector3d centralizedVector = point - centroid;
        pointsOut.push_back(centralizedVector);
    }
}

Eigen::Matrix3d findCovariance(const std::vector<Eigen::Vector3d>& centralizedVectors1,
    const std::vector<Eigen::Vector3d>& centralizedVectors2)
{
    Eigen::Matrix3Xd mat1(3, centralizedVectors1.size());
    Eigen::Matrix3Xd mat2(3, centralizedVectors2.size());
    
    // fill the matrices with the centralized vectors
    for (int i = 0; i < centralizedVectors1.size(); ++i)
    {
        mat1.col(i) = centralizedVectors1[i];
        mat2.col(i) = centralizedVectors2[i];
    }

    Eigen::Matrix3d covariance = mat1 * mat2.transpose();

    return covariance;
}

Eigen::Matrix3d findRotation(const Eigen::Matrix3d& covariance)
{
    // compute SVD of the covariance matrix
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // define the marix diag{1, .., 1, det(VU^T)}
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
    double determinant = (svd.matrixV() * svd.matrixU().transpose()).determinant();
    D(2,2) = determinant;

    // compute the rotation matrix R = V * D * U^T
    Eigen::Matrix3d rotation = svd.matrixV() * D * svd.matrixU().transpose();

    return rotation;
}


Eigen::Vector3d findTranslation(const Eigen::Vector3d& centroid1, 
    const Eigen::Vector3d& centroid2, const Eigen::Matrix3d& rotation)
{
    // translation vector t = c2 - R * c1
    Eigen::Vector3d translation = centroid2 - rotation * centroid1;

    return translation;
}


void refinedRotationTranslation(const std::vector<Eigen::Vector3d>& points3d1,
    const std::vector<Eigen::Vector3d>& points3d2, Eigen::Matrix3d& rotationOut,
    Eigen::Vector3d& translationOut, int iterations)
{
    std::vector<Eigen::Vector3d> inliers1, inliers2, newInliers1, newInliers2;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    inliers1 = points3d1;
    inliers2 = points3d2;

    for (int iter = 0; iter < iterations; ++iter)
    {
        Eigen::Vector3d centroid1 = computeCentroid(inliers1);
        Eigen::Vector3d centroid2 = computeCentroid(inliers2);

        std::vector<Eigen::Vector3d> centralizedVectors1, centralizedVectors2;
        centralizedVectors(inliers1, centroid1, centralizedVectors1);
        centralizedVectors(inliers2, centroid2, centralizedVectors2);

        Eigen::Matrix3d covariance = findCovariance(centralizedVectors1, centralizedVectors2);
        rotation = findRotation(covariance);
        translation = findTranslation(centroid1, centroid2, rotation);

        // reprojection errors
        std::vector<double> errors(inliers1.size());
        for (int i = 0; i < inliers1.size(); ++i)
        {
            Eigen::Vector3d predictedPoint = rotation * inliers1[i] + translation;
            errors[i] = (predictedPoint - inliers2[i]).norm();
        }
        
        // compute the median error
        std::vector<double> errorsClone = errors; // make a copy to sort
        std::sort(errorsClone.begin(), errorsClone.end());
        double medianError = errorsClone[errors.size() / 2];

        // threshold for inliers
        double threshold = std::max(0.01, medianError);

        // filter inliers with error < median error
        for (int i = 0; i < inliers1.size(); ++i)
        {
            if (errors[i] < threshold)
            {
                newInliers1.push_back(inliers1[i]);
                newInliers2.push_back(inliers2[i]);
            }
        }

        // convergence
        if (newInliers1.size() == inliers1.size())
        {
            break;
        }
        // too few inliers
        if (newInliers1.size() < 5)
        {
            break;
        }

        // prepare for next iteration
        inliers1.swap(newInliers1);
        inliers2.swap(newInliers2);
        newInliers1.clear();
        newInliers2.clear();
    }

    rotationOut = rotation;
    translationOut = translation;
}


Eigen::Matrix4d makeTransform(const Eigen::Matrix3d& rotation, 
    const Eigen::Vector3d& translation)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3,3>(0,0) = rotation;
    transform.block<3,1>(0,3) = translation;
    return transform;
}