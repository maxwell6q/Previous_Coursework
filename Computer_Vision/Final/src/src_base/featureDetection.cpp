#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "Utils/inputProcessing.h"
#include "featureDetection.h"


void maskFromDepth(const cv::Mat &depthImgage, cv::Mat &mask)
{
    for (int r = 0; r < depthImgage.rows; r++)
    {
        for (int c = 0; c < depthImgage.cols; c++)
        {
            if (depthImgage.at<uint16_t>(r,c) > 0)
                mask.at<uint8_t>(r,c) = 255;
            else
                mask.at<uint8_t>(r,c) = 0;
        }
    }
}

void computeFeaturesSingleImage(const cv::Mat &img, const cv::Mat &mask, cv::Mat &descriptors, 
    std::vector<cv::KeyPoint> &keypoints, const double sigma, int detectorMode)
{
    // Create the detectors
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create(3000, 4, 0.04, 10, 1.6); 
    cv::Ptr<cv::ORB> orb = cv::ORB::create(2000);   // Number of features
    cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(400); // Hessian threshold

    // Color to grayscale if needed
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // CLAHE for adaptive histogram equalization
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(3.0);
    clahe->setTilesGridSize(cv::Size(8, 8)); 
    cv::Mat enhanced;
    clahe->apply(gray, enhanced);

    // Blurr the image before detection
    cv::Mat blurred;
    if (sigma > 0)
        cv::GaussianBlur(enhanced, blurred, cv::Size(0,0), sigma, 0);
    
    // Choose the detector and detect, default is SURF
    if (detectorMode == 1)
        orb->detectAndCompute(blurred, mask, keypoints, descriptors);
    else if (detectorMode == 2)
        sift->detectAndCompute(blurred, mask, keypoints, descriptors);
    else
        surf->detectAndCompute(blurred, mask, keypoints, descriptors);
}