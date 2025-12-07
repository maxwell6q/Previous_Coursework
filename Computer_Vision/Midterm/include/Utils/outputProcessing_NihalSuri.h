#ifndef OUTPUTPROCESSING_NIHALSURI_H
#define OUTPUTPROCESSING_NIHALSURI_H

#include <opencv2/core.hpp>       
#include <opencv2/imgproc.hpp>     
#include <opencv2/highgui.hpp>     
#include <opencv2/features2d.hpp>  

#include <string>
#include <vector>
#include <filesystem>
#include <iostream>
#include <fstream>

//Save image given origianl path, path where it will be saved and name 
// Inputs:
// img          -Image to be saved
// imagePath    -Path to original image
// savePath     -Path to be saved in 
// addedname    -String that get added to the original name when saved 
void saveImage(cv::Mat& img, 
               const std::string& imagePath, 
               const std::string& savePath, 
               const std::string& addedName);




// Creates a text file which contains a set of bounding box coordinates of each detected object in the image 
// Inputs: 
// rect         -Bounding boxs detected in the object, 0:004_sugar_box, 1:006_mustard_bottle, 2:035_power_drill, vector of dimension 3
// imagePath    -Path to the original image 
// savePath     -Path to be saved in
// addedname    -String that gets added to the original name when saved
void writeCoordinates(const std::vector<cv::Rect>& rect, 
                      const std::string& imagePath, 
                      const std::string& savePath, 
                      const std::string& addedName); 


#endif