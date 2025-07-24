#ifndef FEATUREMATCHING_MAXPRIES_H
#define FEATUREMATCHING_MAXPRIES_H

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
void matchFromDescriptors(const cv::Mat &dscrTest, const cv::Mat &dscrTrain, 
    std::vector<std::vector<cv::DMatch>> &knnMatches, const int k=2);


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


// Selects the keypoints corresponding to good matches.
// Inputs:
// goodKeypoints - the target vector of keypoints
// goodMatches   - a vector of DMatch objects that that are "good"
// keypoints     - the source vector of keypoints from which the good ones are
//                 extracted
void keypointSelection(std::vector<cv::KeyPoint> &goodKeypoints, 
    std::vector<cv::DMatch> goodMatches, const std::vector<cv::KeyPoint> keypoints);

#endif