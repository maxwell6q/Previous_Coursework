#include "featureMatching.h"

#include <opencv2/calib3d.hpp>


void matchFromDescriptors(const cv::Mat &dscr1, const cv::Mat &dscr2, 
    std::vector<std::vector<cv::DMatch>> &knnMatches, int detectorMode, int k)
{
    // Create BFMatcher object
    if (detectorMode == 1) // ORB
    {
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        matcher.knnMatch(dscr1, dscr2, knnMatches, k);
    }
    else // SIFT or SURF
    {
        cv::BFMatcher matcher(cv::NORM_L2);
        matcher.knnMatch(dscr1, dscr2, knnMatches, k);
    }
}


void ratioTest(const std::vector<std::vector<cv::DMatch>> &matchesIn,
    std::vector<cv::DMatch> &matchesOut, double cutOffRatio)
{
    for (const std::vector<cv::DMatch>& match : matchesIn)
    {
        // If the best match is considerably better than the second best, keep it
        if (match.size() == 2 && match[0].distance < cutOffRatio * match[1].distance)
            matchesOut.push_back(match[0]);
    }
}


void crossCheck(const std::vector<cv::DMatch> &matches12,
    const std::vector<cv::DMatch> &matches21, std::vector<cv::DMatch> &matchesOut)
{
    for (const cv::DMatch& match12 : matches12)
    {
        for (const cv::DMatch& match21 : matches21)
        {
            if (match12.queryIdx == match21.trainIdx && match12.trainIdx == match21.queryIdx)
            {
                matchesOut.push_back(match12);
                break;
            }
        }
    }
}


void spatialFilter(const std::vector<cv::DMatch> &matchesIn,
    std::vector<cv::DMatch> &matchesOut, const std::vector<cv::KeyPoint> keypoints1,
    const std::vector<cv::KeyPoint> keypoints2, double maxDist)
{
    for (const cv::DMatch& m : matchesIn)
    {
        cv::Point2f pt1 = keypoints1[m.queryIdx].pt;
        cv::Point2f pt2 = keypoints2[m.trainIdx].pt;
        double dist = cv::norm(pt1 - pt2);
        if (dist <= maxDist)
            matchesOut.push_back(m);
    }
}


void matches2points(std::vector<cv::Point2f> &points, 
    std::vector<cv::DMatch> matches, std::vector<cv::KeyPoint> keypoints, bool useQuery)
{
    for (const cv::DMatch& m : matches)
    {
        if (useQuery)
            points.push_back(keypoints[m.queryIdx].pt); // image 1
        else
            points.push_back(keypoints[m.trainIdx].pt); // image 2
    }
}


void applyUSAC(std::vector<cv::Point2f> &points1in, std::vector<cv::Point2f> &points2in,
    std::vector<cv::Point2f> &points1out, std::vector<cv::Point2f> &points2out,
    const float usacThresh, const std::vector<float> &cameraIntrinsics)
{
    cv::Mat K = (cv::Mat_<double>(3,3) << cameraIntrinsics[0], 0, cameraIntrinsics[2],
                                          0, cameraIntrinsics[1], cameraIntrinsics[3],
                                          0, 0, 1);
    std::vector<cv::Point2f> points1norm, points2norm;
    
    // Normalize points
    cv::undistortPoints(points1in, points1norm, K, cv::noArray());
    cv::undistortPoints(points2in, points2norm, K, cv::noArray());

    cv::Mat inlierMask;
    cv::Mat E = cv::findEssentialMat(points1norm, points2norm, cameraIntrinsics[0], 
        cv::Point2d(cameraIntrinsics[2], cameraIntrinsics[3]), cv::USAC_ACCURATE, 0.999, 
        usacThresh, inlierMask);

    for (size_t i = 0; i < points1in.size(); i++)
    {
        if (inlierMask.at<uchar>(i)) 
        {
            points1out.push_back(points1in[i]);
            points2out.push_back(points2in[i]);
        }
    }
}