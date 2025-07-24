#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>


#include "Utils/inputProcessing_MaxPries.h"
#include "Utils/outputProcessing_NihalSuri.h"
#include "FeatureDetectorMatcher/featureDetection_MaxPries.h"
#include "ObjectDetector/ObjectDetector_BjornMagnus.h"


void runDetectionAllImages(const std::vector<std::vector<std::string>> &allModels, 
    const std::vector<std::vector<std::string>> &allMasks, 
    const std::vector<std::string> &allTests, 
    int mode, int minKeypoints, double sigma, double cutOffRatio, 
    int boxHeight, int boxWidth, bool logImages, std::string savePath, int detectorMode)
{
    // Compute the descriptors for all three objects 
    // and store them in a vector ({sugar, mustard, drill})
    std::vector<cv::Mat> allDescriptors(3);
    const std::string& coordinatesPath =  savePath + "/coordinates/"; 
    const std::string& descriptorImagePath =  savePath + "/images/"; 

    
    for (int idxObj=0; idxObj<allDescriptors.size(); idxObj++)
        computeFullDescriptorList(allModels[idxObj], allMasks[idxObj], allDescriptors[idxObj], detectorMode);
    
    

    // Loop over all images
    for (const std::string &imagePath : allTests)
    {
        // Process each image => get three boxes, one for each object
        std::vector<std::vector<cv::KeyPoint>> goodKeypoints(3);
        std::vector<cv::Rect> targetBoxes(3);
        for (int idxObj=0; idxObj<allDescriptors.size(); idxObj++)
            runDetection(imagePath, allDescriptors[idxObj], mode, minKeypoints, 
                         targetBoxes[idxObj], goodKeypoints[idxObj], sigma,
                         cutOffRatio, boxHeight, boxWidth, detectorMode);

        
        if (logImages)
        {
            // Draw matched keypoints and the resulting boxes for the same object in the same color and save image
            std::vector<cv::Scalar> color = {cv::Scalar(0,0,255), cv::Scalar(0,255,0), cv::Scalar(255,0,0)};
            cv::Mat img;
            loadInput(img,imagePath);
            for (int idxObj=0; idxObj<targetBoxes.size(); idxObj++)
            {
                cv::drawKeypoints(img, goodKeypoints[idxObj], img, color[idxObj]);
                cv::rectangle(img, targetBoxes[idxObj], color[idxObj],2);
            }
            saveImage(img,imagePath, descriptorImagePath ,"_output.jpg");
        }
       writeCoordinates(targetBoxes, imagePath, coordinatesPath , "_output");
    }
}