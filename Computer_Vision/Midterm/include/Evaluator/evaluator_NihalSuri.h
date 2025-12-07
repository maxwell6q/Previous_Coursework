#ifndef EVALUATOR_NIHALSURI_H
#define EVALUATOR_NIHALSURI_H

#include <opencv2/core.hpp>       
#include <opencv2/imgproc.hpp>     
#include <opencv2/highgui.hpp>     
#include <opencv2/features2d.hpp>  

#include <string>
#include <vector>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <map>
#include <cmath>


// Structure to store bounding box information
struct BoundingBox {
    int xmin, ymin, xmax, ymax;
    
    BoundingBox() : xmin(0), ymin(0), xmax(0), ymax(0) {}
    BoundingBox(int x1, int y1, int x2, int y2) : xmin(x1), ymin(y1), xmax(x2), ymax(y2) {}
    
    // Calculate area of the bounding box
    int area() const {
        return (xmax - xmin) * (ymax - ymin);
    }
};

// Function to parse a annotation from a file
std::pair<std::string, BoundingBox> parseLine(const std::string& line);

// Function to calculate IoU between two bounding boxes
float calculateIoU(const BoundingBox& box1, const BoundingBox& box2);

// Calculate mean intersection over union and detection accurracy given the path of the text files
// Inputs:
// resultsPath   -Path of the annotation results
std::vector<float> performanceMetrics(const std::string& resultsPath, bool returnValue = false);

// Evaluate different parameter combinations
// NOTE: Depending on the parameter ranges, this function might run exremely long
// Inputs:
// mode           - the mode to be used in testing
// savePath       - path to target directory for result-text file
// startKeypoints - min parameter for minKeypoints to be tested
// stopKeypoints  - max parameter for minKeypoints to be tested
// startSigma     - min parameter for Sigma to be tested
// stopSigma      - max parameter for Sigma to be tested
// startRatio     - min parameter for cutOffRatio to be tested
// stopRatio      - max parameter for cutOffRatio to be tested
// startHeight    - min parameter for boxHeight to be tested, optional
// stopHeight     - max parameter for boxHeight to be tested, optional
// boxWidth       - boxWidth to be used in testing, constant in testing optional
//                  If no box dimensions are chosen, the best ones are used to 
//                  reduce loop time
void evaluateBestParameters(int mode, std::string savePath,
    int startKeypoints, int stopKeypoints,
    double startSigma, double stopSigma,
    double startRatio, double stopRatio,
    int startHeight=225, int stopHeight=225,
    int boxwidth=150, int detectorMode=0);


#endif
