#ifndef FEATUREDETECTION_H
#define FEATUREDETECTION_H

#include <opencv2/highgui.hpp>
#include <vector>

// Creates a binary mask from the depth image where valid depth pixels are white (255)
// and invalid are black (0).
//
// Inputs:
// depthImage - the input depth image (CV_16U)
// mask       - the output binary mask (CV_8U)
void maskFromDepth(const cv::Mat &depthImgage, cv::Mat &mask);


// Detects all keypoints of an image and computes their descriptors.
//
// Inputs:
// img          - the image to be analyzed
// descriptors  - a cv::Mat object containing all descriptors
// keypoints    - a vector containing all keypoints as cv::Keypoint
// sigma        - the sigma value for gaussian blur befor feature detection
// detectorMode - choice of the type of keypoints to be used, optional
void computeFeaturesSingleImage(const cv::Mat &img, const cv::Mat &mask, cv::Mat &descriptors, 
    std::vector<cv::KeyPoint> &keypoints, const double sigma, int detectorMode=0);



#endif