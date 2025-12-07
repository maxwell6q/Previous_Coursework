#include "Utils/inputProcessing.h"
#include "Utils/outputProcessing.h"
#include "featureDetection.h"
#include "featureMatching.h"
#include "poseEstimation.h"


#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <numeric>



int main(int argc, char** argv){

    // =============================================================================
    // PARAMETERS
    // =============================================================================
    
    // general setup
    int dataset = 1;        // 0: freiburg1_rpy; 1: freiburg1_xyz; 
                            // 2: freiburg2_rpy; 3: freiburg2_xyz;
                            // 4: freiburg2_pioneer_360; 5: freiburg2_pioneer_slam
                            // 6: freiburg2_pioneer_slam3
    int detectorMode = 2;   // 0: SURF; 1: ORB; 2: SIFT

    // Override dataset index if provided as first argument
    if(argc > 1){          
        dataset = std::stoi(argv[1]);
    }
    // Override detector mode if provided as second argument
    if(argc > 2){         
    detectorMode = std::stoi(argv[2]);
    }

    // tuning parameters
    double sigma = 0.5;                  // Gaussian blur sigma
    double ratioThreshold = 0.8;         // Lowe's ratio test threshold
    double spatialThreshold = 100.0;     // spatial filter threshold in pixels
    float usacThreshold = 1.0f;          // USAC outlier threshold
    int corrThreshold = 30;              // 3D coorespondence thresholdhasNaN
    double translationThreshold = 0.9;   // max translation between frames in meters

    // camera intrinsics [fx, fy, cx, cy]
    std::vector<float> cameraIntrinsics;
    if (dataset <= 1)
        cameraIntrinsics = {517.3, 516.5, 318.6, 255.3}; // freiburg1
    else
        cameraIntrinsics = {520.9, 521.0, 325.1, 249.7}; // freiburg2



    // =============================================================================
    // PREPARATION
    // =============================================================================

    // Intro message
    std::cout << "\n=== RGB-D Visual Odometry Processing ===" << std::endl;

    // dataset and file paths
    std::vector<std::string> datasetNames = {"freiburg1_rpy", "freiburg1_xyz", 
        "freiburg2_rpy", "freiburg2_xyz", "freiburg2_pioneer_360", 
        "freiburg2_pioneer_slam", "freiburg2_pioneer_slam3"};

    std::string dataPath = "../data/rgbd_dataset_" + datasetNames[dataset] + "/";
    std::string resultsPath = "../results/results_" + datasetNames[dataset] + ".txt";

    std::cout << "\nLooking for dataset in: " << dataPath << std::endl;

    // Clear previous results file
    std::ofstream clearFile(resultsPath, std::ios::trunc);
    clearFile.close();

    // Load the image paths from the directory
    std::vector<std::vector<std::string>> paths;
    paths = fetchAssociated(dataPath);

    // Check if paths were loaded successfully
    if (paths.empty())
    {
        std::cerr << "Error: No image pairs found in dataset!" << std::endl;
        return -1;
    }
    std::cout << "Found " << paths.size() << " image pairs in dataset." << std::endl;

    // Get user input for number of frames to process
    int maxFramesToProcess;
    std::cout << "\nEnter number of frame pairs to process (0 for all): ";
    std::cin >> maxFramesToProcess;
    
    if (maxFramesToProcess <= 0 || maxFramesToProcess >= paths.size())
        maxFramesToProcess = paths.size() - 1;

    std::cout << "Processing " << maxFramesToProcess << " frame pairs..." << std::endl;
    
    
    
    
    // =============================================================================
    // TIMING SETUP
    // =============================================================================
    
    auto totalStartTime = std::chrono::high_resolution_clock::now();
    std::vector<double> frameTimes;
    


    // =============================================================================
    // MAIN PROCESSING LOOP
    // =============================================================================

    std::cout << "Starting processing...\n";

    // Initialize the global transformation matrix
    Eigen::Matrix4d transformWorld = Eigen::Matrix4d::Identity();

    for (int i = 0; i < maxFramesToProcess; i++)
    {
        auto frameStartTime = std::chrono::high_resolution_clock::now();
        
        // Progress reporting
        if (i % 50 == 0)
            std::cout << "\nProcessing frame pair " << i << "/" << maxFramesToProcess << std::endl;
        
        try
        {
            // -------------------------------------------------------------------------
            // LOAD CURRENT FRAME PAIR
            // -------------------------------------------------------------------------
            
            cv::Mat imgRGB1, imgRGB2, imgDepth1, imgDepth2;
            
            // Load RGB images
            loadInput(imgRGB1, paths[i][1]);     // frame i RGB
            loadInput(imgRGB2, paths[i+1][1]);   // frame i+1 RGB
            
            // Load depth images
            loadInput(imgDepth1, paths[i][2], true);   // frame i depth
            loadInput(imgDepth2, paths[i+1][2], true); // frame i+1 depth
            cv::GaussianBlur(imgDepth1, imgDepth1, cv::Size(0,0), sigma, 0);
            cv::GaussianBlur(imgDepth2, imgDepth2, cv::Size(0,0), sigma, 0);
            

            // -------------------------------------------------------------------------
            // FEATURE DETECTION
            // -------------------------------------------------------------------------
            
            cv::Mat descriptors1, descriptors2;
            std::vector<cv::KeyPoint> keypoints1, keypoints2;

            cv::Mat depthMask1 = cv::Mat::zeros(imgDepth1.size(), CV_8U);
            cv::Mat depthMask2 = cv::Mat::zeros(imgDepth2.size(), CV_8U);

            // Only consider pixels with valid depth
            maskFromDepth(imgDepth1, depthMask1);
            maskFromDepth(imgDepth2, depthMask2);
            
            computeFeaturesSingleImage(imgRGB1, depthMask1, descriptors1, keypoints1, sigma, detectorMode);
            computeFeaturesSingleImage(imgRGB2, depthMask2, descriptors2, keypoints2, sigma, detectorMode);
            
            // Check keypoints count should be greater than correspondence threshold
            if (keypoints1.size() < 5*corrThreshold || keypoints2.size() < 5*corrThreshold)
            {   
                if (detectorMode != 2) // If already using SURF, skip
                {
                    std::cout << "Waring: Not enough keypoints at frame pair " << i << std::endl;
                    continue; // Skip this frame pair
                }
                
                std::cout << "Warning: Not enough keypoints at frame pair " << i << std::endl;
                std::cout << "         Switching to SURF detector for this frame pair." << std::endl;
                descriptors1.release();
                descriptors2.release();
                keypoints1.clear();
                keypoints2.clear();
                computeFeaturesSingleImage(imgRGB1, depthMask1, descriptors1, keypoints1, sigma, 0);
                computeFeaturesSingleImage(imgRGB2, depthMask2, descriptors2, keypoints2, sigma, 0);
                if (keypoints1.size() < corrThreshold || keypoints2.size() < corrThreshold)
                {
                    std::cout << "Waring: SURF also failed. Skipping frame pair " << std::endl;
                    continue; // Skip this frame pair
                }
            }

            
            // -------------------------------------------------------------------------
            // FEATURE MATCHING & FILTERING
            // -------------------------------------------------------------------------
            
            std::vector<std::vector<cv::DMatch>> matches12, matches21;
            matchFromDescriptors(descriptors1, descriptors2, matches12, detectorMode, 2);
            matchFromDescriptors(descriptors2, descriptors1, matches21, detectorMode, 2);
            
            std::vector<cv::DMatch> ratioTestedMatches12, ratioTestedMatches21;
            ratioTest(matches12, ratioTestedMatches12, ratioThreshold);
            ratioTest(matches21, ratioTestedMatches21, ratioThreshold);

            std::vector<cv::DMatch> ratioTestedMatches;
            crossCheck(ratioTestedMatches12, ratioTestedMatches21, ratioTestedMatches);
            
            std::vector<cv::DMatch> filteredMatches;
            spatialFilter(ratioTestedMatches, filteredMatches, 
                keypoints1, keypoints2, spatialThreshold);


            if (filteredMatches.size() < corrThreshold)
            {
                std::cout << "Warning: Not enough filtered matches at frame pair " << i << std::endl;
                continue; // Skip this frame pair
            }
            
            std::vector<cv::Point2f> goodPoints1, goodPoints2;
            matches2points(goodPoints1, filteredMatches, keypoints1, true);
            matches2points(goodPoints2, filteredMatches, keypoints2, false);

            std::vector<cv::Point2f> inlierPoints1, inlierPoints2;
            applyUSAC(goodPoints1, goodPoints2, inlierPoints1, inlierPoints2, usacThreshold, cameraIntrinsics);
            
            if (inlierPoints1.size() < corrThreshold)
            {
                std::cout << "Warning: Not enough inlier matches at frame pair " << i << std::endl;
                continue; // Skipt this frame pair
            }
            

            // -------------------------------------------------------------------------
            // POSE ESTIMATION
            // -------------------------------------------------------------------------
            
            // Lift 2D points to 3D
            std::vector<Eigen::Vector3d> points3d1, points3d2;
            for (size_t p = 0; p < inlierPoints1.size(); p++)
            {
                // Get depth values
                float depth1 = getDepth(imgDepth1, inlierPoints1[p]);
                float depth2 = getDepth(imgDepth2, inlierPoints2[p]);
    
                // Check depth in valid range and filter too big depth differences
                if (depth1 > 0.0f && depth2 > 0.0f && depth1 < 6.0f && depth2 < 6.0f &&
                    std::abs(depth1 - depth2) < translationThreshold)
                {
                    points3d1.push_back(lift2D23D(inlierPoints1[p], depth1, cameraIntrinsics));
                    points3d2.push_back(lift2D23D(inlierPoints2[p], depth2, cameraIntrinsics));
                }
            }               

            // Check 3D correspondences count
            if (points3d1.size() < corrThreshold)
            {
                std::cout << "Warning: Not enough 3D correspondences at frame pair " << i << std::endl;
                continue; // Skip this frame pair
            }
            

            // Compute Rototranslation using SVD
            try
            {
                Eigen::Matrix3d rotation;
                Eigen::Vector3d translation;
                refinedRotationTranslation(points3d1, points3d2, 
                    rotation, translation, 10);

                // sanity check on rotation and translation
                if (std::abs(rotation.determinant() - 1.0) > 0.1 ||
                    translation.norm() > translationThreshold)
                {
                    std::cerr << "Warning: Invalid rotation or translation at frame " << i << std::endl;
                    continue; // Skip this frame pair
                }
                
                // update global transformation
                Eigen::Matrix4d transformRelative = makeTransform(rotation, translation);
                transformWorld = transformWorld * transformRelative;
                
                // log results
                logResults(paths[i+1][0], transformWorld, resultsPath);

                // some sanity Checks every 50 frames
                if (i % 50 == 0)
                {
                    std::cout << "Correspondences: " << points3d1.size() << std::endl;

                    // transformation details
                    std::cout << "t_norm=" << translation.norm() << ", " << "det(R)=" << rotation.determinant()<< std::endl;
                    
                    // root mean square error
                    double rmse = 0.0;
                    for (size_t j=0; j<points3d1.size(); ++j) {
                        Eigen::Vector3d p2_pred = rotation * points3d1[j] + translation;
                        rmse += (p2_pred - points3d2[j]).squaredNorm();
                    }
                    rmse = std::sqrt(rmse / points3d1.size());
                    std::cout << "3D RMSE (m): " << rmse << std::endl;
                }
                
                

            } 
            // error during SVD
            catch (const std::exception& e) 
            {
                std::cerr << "SVD error at frame " << i << ": " << e.what() << std::endl;
            }
            

        }
        // error during frame processing
        catch (const std::exception& e) 
        {
            std::cerr << "Error processing frame pair " << i << ": " << e.what() << std::endl;
        }
        
        // Record frame timing
        auto frameEndTime = std::chrono::high_resolution_clock::now();
        auto frameDuration = std::chrono::duration_cast<std::chrono::milliseconds>(frameEndTime - frameStartTime);
        frameTimes.push_back(frameDuration.count());
    }

    
    // =============================================================================
    // TIMING RESULTS
    // =============================================================================
    
    std::cout << "\nAll frames processed." << std::endl;
    
    auto totalEndTime = std::chrono::high_resolution_clock::now();
    auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(totalEndTime - totalStartTime);
    
    std::cout << "\nProcessing complete!" << std::endl;
    std::cout << "Total processing time: " << totalDuration.count() << " ms (" 
              << std::fixed << std::setprecision(2) << totalDuration.count() / 1000.0 << " seconds)" << std::endl;
    
    if (!frameTimes.empty()) 
    {
        double avgTime = std::accumulate(frameTimes.begin(), frameTimes.end(), 0.0) / frameTimes.size();
        std::cout << "Average time per frame: " << std::fixed << std::setprecision(2) 
                  << avgTime << " ms" << std::endl;
    }
        
    return 0;
}

