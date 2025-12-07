#ifndef INPUTPROCESSING_H
#define INPUTPROCESSING_H

#include <opencv2/highgui.hpp>
#include <string>
#include <vector>

// Loads an image into a cv::Mat object from a provided path
//
// Inputs:
// img     - the target cv::Mat object
// path    - the path where the image is stored
// isDepth - wheather the image should be loaded as a depth image
void loadInput(cv::Mat &img, const std::string path, bool isDepth=false);



// Generates a vector containing triplets of associated rbg and depth image paths and timestamps
// Inputs:
// sourceDir - the directory to be searced, needs to contain a file "associated_stamps.txt"
//
// Output:
// pathList - a vector of vectors, each containing a triplet of file-paths:
//            [0]-timestamp, [1]-rgb image path, [2]-depth image path
std::vector<std::vector<std::string>> fetchAssociated(const std::string sourceDir);


#endif