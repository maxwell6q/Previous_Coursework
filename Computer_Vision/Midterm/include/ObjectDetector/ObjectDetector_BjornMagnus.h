#ifndef OBJECTDETECTOR_BJORNMAGNUS_H
#define OBJECTDETECTOR_BJORNMAGNUS_H

#include <opencv2/core.hpp>       
#include <opencv2/imgproc.hpp>     
#include <opencv2/highgui.hpp>     
#include <opencv2/features2d.hpp>  

#include "FeatureDetectorMatcher/featureDetection_MaxPries.h"
#include "FeatureDetectorMatcher/featureMatching_MaxPries.h"
#include "Utils/inputProcessing_MaxPries.h"

// Fits a box with given size to contain highest amount of features, the box get place moth horizontally and vertically 
// Inputs:
// img          -Image to place the box on (Use only the dimention for grid search)
// Keypoints -Vector of keypoints who you want the box fitted to 
// boxWidth      -Width of the box 
// boxHeight     -Height of the boz
// bestBox      -A rect to where the best box get written to 
void DOslidingBox(const cv::Mat &img, std::vector<cv::KeyPoint> &Keypoints, const int boxWidth, const int boxHeight, cv::Rect &bestBox);

// Find a box centred around the features that can be fitted in to a single box given by the size of the input
// Inputs:
// img           -Image to place the box on (Use only the dimention for grid search)
// Keypoints -Vector of keypoints who you want the box fitted to 
// boxWidth      -Width of the box 
// boxHeight     -Height of the boz
// bestBoxCented -A rect to where the best centred box get written to 
void DOcenteredBox(const cv::Mat &img, std::vector<cv::KeyPoint> &Keypoints, const int boxWidth, const int boxHeight, cv::Rect &bestBoxCentred);

// Dettects all object in the image based on canny and findContours
// Inputs: 
// inputImg      -Image to detect objects 
// detectedBoxes -Vector containing all the Rects from the object detection 
// maxArea       -Maximum area for 
void DOconturs(const cv::Mat &inputImg,std::vector<cv::Rect> &detectedBoxes, int maxArea);


// Given a list of boxes and keypoints find the box that contains the most keypoints 
// Inputs:
// Keypoints -Vector of keypoints who you want the box fitted to 
// boxes         -List of boxes to searc through 
// bestBox       -A rect to where the best box get written to 
void fitBestBox(std::vector<cv::KeyPoint> &Keypoints,std::vector<cv::Rect> boxes,cv::Rect &bestBox);



// Given Keupoints and a box counts if the minimal of keypoints contained in the box is reached 
// If the minimum is not reached set the box to an empty one 
// Inputs: 
// Keypoints    -Vector of keypoint to be used 
// box          -The box to be evaluated 
// minKeypoints -Minimal keypoints to decide if the box contain the object or not 
void boxDecider(std::vector<cv::KeyPoint> &Keypoints,cv::Rect &box, int minKeypoints);

// Processes one image 
// Inputs:
// imagePath      - path of the image to be processed
// desciptorList  - list of all descriptors for one object
// mode           - choice of algorithm to draw boxes
// minKeypoints   - minimum number of keypoints to be considered a match
// objectDetected - target for the box of a detected object
// goodKeypoints  - target for the selected good keypoints
// sigma          - variance for the gaussian blur
// cuttOffRatio   - cut off ratio for the ratio test
// boxHeight      - height for the box, only relevant if mode=1 or 2
// boxWidth       - width for the box, only relevant if mode=1 or 2
// detectorMode   - choice which type of features is detected
void runDetection(std::string imagePath, cv::Mat &descriptorList, int mode, 
                  int minKeypoints, 
                  cv::Rect &objectDeteced, std::vector<cv::KeyPoint> &goodKeypoints,
                  double sigma, double cutOffRatio, int boxHeight, int boxWidth,
                  int detectorMode=0);




#endif