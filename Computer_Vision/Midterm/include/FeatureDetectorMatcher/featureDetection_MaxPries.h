#ifndef FEATUREDETECTION_MAXPRIES_H
#define FEATUREDETECTION_MAXPRIES_H

#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <string>
#include <vector>


// Detects all keypoints of an image and computes their descriptors.
// The intended use is for the test images.
// Inputs:
// img          - the image to be analyzed
// descriptors  - a cv::Mat object containing all descriptors
// keypoints    - a vector containing all keypoints as cv::Keypoint
// sigma        - the sigma value for gaussian blur befor feature detection
// detectorMode - choice of the type of keypoints to be used, optional
void computeFeaturesSingleImage(const cv::Mat &img, cv::Mat &descriptors, 
    std::vector<cv::KeyPoint> &keypoints, const double sigma, int detectorMode=0);


// Computes all descriptors inside a mask for a set of images
// Inputs:
// pathList       - a vector of strings containing the paths to the images
// maskList       - a vector of strings containing the paths to the masks, the
//                  keypoints are only detected inside the mask
// descriptorList - the target for the descriptors. All descriptors for each
//                  image are concatenated 
// detectorMode   - choice of the type of keypoints to be used, optional
//                  0:SIFT(default), 1:SURF, 2:ORB, 3:AKAZE
void computeFullDescriptorList(const std::vector<std::string> pathList, 
    const std::vector<std::string> maskList, cv::Mat &descriptorList, int detectorMode=0);

#endif