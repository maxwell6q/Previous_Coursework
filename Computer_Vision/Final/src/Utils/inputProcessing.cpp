#include "Utils/inputProcessing.h"

#include <filesystem>
#include <iostream>
#include <fstream>
#include <sstream>


std::vector<std::vector<std::string>> fetchAssociated(const std::string sourceDir)
{
    std::vector<std::vector<std::string>> pathList;
    std::ifstream associatedTxt(sourceDir + "/associated_stamps.txt");
    std::string line;

    while(std::getline(associatedTxt, line))
    {
        std::istringstream iss(line);
        std::string stamp1, stamp2, rgbPath, depthPath;
        if (iss >> stamp1 >> rgbPath >> stamp2 >> depthPath)
        {
            rgbPath = sourceDir + "/" + rgbPath;
            depthPath = sourceDir + "/" + depthPath;
            pathList.push_back({stamp1, rgbPath, depthPath});
        }
    }
    return pathList;
}


void loadInput(cv::Mat& img, const std::string path, bool isDepth)
{
    if (!isDepth)
        img = cv::imread(path, cv::IMREAD_COLOR);   // 8UC3
    else 
    {
        img = cv::imread(path, cv::IMREAD_UNCHANGED); // expect 16UC1
        if (img.type() != CV_16UC1)
        {
            std::cerr << "Depth not 16UC1 at " << path << ", got type " << img.type() << "\n";
            std::exit(1);
        }
    }

    if (img.empty())
    {
        std::cerr << "Could not load " << path << "\n"; 
        std::exit(1);
    }
}