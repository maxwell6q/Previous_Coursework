#ifndef OUTPUTPROCESSING_H
#define OUTPUTPROCESSING_H

#include <Eigen/Dense>
#include <string>


// Logs the rototranslation data int a provided .txt file. Rotations are logged as quaternions
//
// Inputs:
// timestamp   - timestamp to be logged
// transform   - the 4x4 transformation matrix
// path        - the path to the .txt file
void logResults(const std::string timestamp, const Eigen::Matrix4d transform,
    const std::string path);


#endif