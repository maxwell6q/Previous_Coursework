#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "Utils/inputProcessing_MaxPries.h"
#include "FeatureDetectorMatcher/featureDetection_MaxPries.h"



void computeFeaturesSingleImage(const cv::Mat &img, cv::Mat &descriptors, 
    std::vector<cv::KeyPoint> &keypoints, const double sigma, int detectorMode)
{
    // Create the detectors
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
    cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create();

    // Blurr the image before detection
    if (sigma > 0)
    {
        cv::Size g_size(0,0);
        cv::GaussianBlur(img, img, g_size, sigma, 0);
    }
    
    // Choose the detector and detect, default is SIFT
    // cv::noArray() stands for an empty mask, 
    if (detectorMode == 1){
        surf->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
    }
    else if (detectorMode == 2){
        orb->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
    }
    else if (detectorMode == 3){
        akaze->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
    }
    else{
        sift->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
    }
    
}


void computeFullDescriptorList(const std::vector<std::string> pathList, 
    const std::vector<std::string> maskList,
    cv::Mat &descriptorList, int detectorMode)
{
    // Create a cv::SIFT object, which is like a pointer
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
    cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create();

    for (int i=0; i<pathList.size(); i++)
    {
        cv::Mat trainImg;
        cv::Mat trainMask;
        loadInput(trainImg, pathList[i]);
        loadInput(trainMask, maskList[i], true);
        

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        // Detect keypoints within the mask and computes their descriptors
        if (detectorMode == 1){
            surf->detectAndCompute(trainImg, trainMask, keypoints, descriptors);
        }
        else if (detectorMode == 2){
            orb->detectAndCompute(trainImg, trainMask, keypoints, descriptors);
        }
        else if (detectorMode == 3){
            akaze->detectAndCompute(trainImg, trainMask, keypoints, descriptors);
        }
        else{
            sift->detectAndCompute(trainImg, trainMask, keypoints, descriptors);
        }

        // Concattenate the descriptors of each training image
        if (!descriptors.empty())
        {
            descriptorList.push_back(descriptors);
        }
    }
}
