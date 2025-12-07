#ifndef FEATUREMATCHING_H
#define FEATUREMATCHING_H

#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <vector>


// Finds matches for a test image to a training dataset, based on their 
// descriptors.
// Inputs:
// dscrTest   - a set of descriptors for the test image
// dscrTrain  - a set of descriptors for the training dataset
// knnMatches - the target for the matches. It is a vector of vectors.
//              In the outer layer, there is one element for each keypoint in
//              the test image. Each of these elements is a vector of cv::DMatch
//              objects in order of strongest to weakest match. (More than one 
//              match for each keypoint in the test image)
// k          - maximum number of matches for each keypoint in the test image
//              default: k=2, to apply the ratio test
void matchFromDescriptors(const cv::Mat &dscr1, const cv::Mat &dscr2, 
    std::vector<std::vector<cv::DMatch>> &knnMatches, int detectorMode, int k=2);


// Applies the ratio test as described by D. Lowe. Disregards matches if the 
// second best match is also good.
// Inputs:
// matchesIn   - a vector containing one vector for each keypoint. The inner
//               vector contains the two best DMatch elements.
// matchesOut  - a vector containing only those DMatch elements, which are
//               noticably better than the second best matches
// cutOffRatio - the ratio to be considered in the ratio test
void ratioTest(const std::vector<std::vector<cv::DMatch>> &matchesIn,
    std::vector<cv::DMatch> &matchesOut, const double cutOffRatio);


// Cross-checks the matches from image 1 and image 2.
//
// Inputs:
// matches12   - a vector of DMatch objects from image 1 to image 2
// matches21   - a vector of DMatch objects from image 2 to image 1
// matchesOut  - a vector of DMatch objects that are the output of the function
void crossCheck(const std::vector<cv::DMatch> &matches12,
    const std::vector<cv::DMatch> &matches21, std::vector<cv::DMatch> &matchesOut);


// Filters matches based on spatial distance in pixels between keypoints.
// Inputs:
// matchesIn   - a vector of DMatch objects that are to be filtered
// matchesOut  - a vector of DMatch objects that are the output of the function
// keypoints1  - vector of keypoints in the first image
// keypoints2  - vector of keypoints in the second image
// maxDist     - the maximum distance between matched keypoints to be considered
void spatialFilter(const std::vector<cv::DMatch> &matchesIn,
    std::vector<cv::DMatch> &matchesOut, const std::vector<cv::KeyPoint> keypoints1,
    const std::vector<cv::KeyPoint> keypoints2, double maxDist);


// Selects the keypoints corresponding to good matches.
// Inputs:
// goodKeypoints - the target vector of keypoints
// goodMatches   - a vector of DMatch objects that that are "good"
// keypoints     - the source vector of keypoints from which the good ones are
//                 extracted
void matches2points(std::vector<cv::Point2f> &points, 
    std::vector<cv::DMatch> matches, std::vector<cv::KeyPoint> keypoints, bool useQuery);


// Filters the matches so they "move in similar direction"
// Inputs:
// points1       - vector of locations (points) of keypoints in first image
// points2       - vector of locations (points) of keypoints in second image
// matches       - vector of DMatch objects that are to be filtered
// inlierMatches - vector of DMatch objects that are the output of the function
// usacThresh    - the threshold for the USAC algorithm to consider a match as an inlier
//                  (default: 3.0)
void applyUSAC(std::vector<cv::Point2f> &points1in, std::vector<cv::Point2f> &points2in,
    std::vector<cv::Point2f> &points1out, std::vector<cv::Point2f> &points2out,
    const float usacThresh, const std::vector<float> &cameraIntrinsics);

#endif