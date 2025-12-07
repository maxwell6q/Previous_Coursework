#include "Utils/outputProcessing.h"

#include <fstream>
#include <iostream>


void logResults(const std::string timestamp, const Eigen::Matrix4d transform,
    const std::string path)
{
    // extract rotation and translation from the transformation matrix
    Eigen::Vector3d translation = transform.block<3,1>(0,3);
    Eigen::Matrix3d rotation = transform.block<3,3>(0,0);

    // convert rotation vector to quaterneons
    Eigen::Quaterniond quaternion(rotation);

    // write the results to a file
    std::ofstream file(path, std::ios::app);
    if (file.is_open())
    {
        file << timestamp << " "
             << translation.transpose() << " "
             << quaternion.x() << " "
             << quaternion.y() << " "
             << quaternion.z() << " "
             << quaternion.w() << "\n";
    }
    else
    {
        std::cerr << "Error opening file: " << path << std::endl;
    }
}