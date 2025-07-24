#include <iostream>
#include <string>
#include <vector>

#include "Utils/inputProcessing_MaxPries.h"
#include "ObjectDetector/objectDetectorAll_MaxPries.h"
#include "Evaluator/evaluator_NihalSuri.h"


int main(int argc, char** argv)
{
    // Best parameters SIFT
    int detector = 0;                       // 0:SIFT, 1:SURF, 2:ORB, 3:AKAZE
    int mode = 2;                           // The type of box-fitting
    int minKeypoints = 20;                  // Min number of Keypoints to draw the box
    double sigma = 0.8;                     // For gaussian smoothing of test images
    double cutOffRatio = 0.83;              // For ratio test
    int boxHeight = 225;                    // Box dimension 1 (might rotate)
    int boxWidth = 150;                     // Box dimension 2,(might rotate)
    bool logImages = true;                  // Save images with KPs and boxes (y/n)
    std::string savePath = "../results";    // Where to save results (will always log txt data)


    // Declare the targets for training and test data
    std::vector<std::vector<std::string>> allModels, allMasks;
    std::vector<std::string> allTests;


    // Load training and test data from fixed path
    loadTrainTestData(allModels, allMasks, allTests);


    // Process all images and store the results in the defined path
    runDetectionAllImages(allModels, allMasks, allTests, mode, minKeypoints, sigma, cutOffRatio,
                          boxHeight, boxWidth, logImages, savePath, detector);

    
    // Evaluate the mIoU and detection accuracy and display them on terminal
    performanceMetrics(savePath);
    

    return 0;
}