# Computer Vison Coursework
This directory contains projects and assignments completed during my master's coursework in Computer Vision.
The focus of these projects is on implementing classical and modern computer vision algorithms for robotics and localization applications.

## Final Project
This repository contains a complete C++ and OpenCV-based visual odometry system designed to estimate sequential 6-DoF camera motion from monocular RGB-D images.

The pipeline recovers absolute scale and estimates rigid-body transformations by combining traditional feature-based tracking with 3D point lifting.

![Visual Odometry Pipeline](https://github.com/maxwell6q/Previous_Coursework/tree/main/Computer_Vision/Figures/vo_pipeline.png)

* **Feature Detection:** Utilizes SIFT keypoints restricted to valid depth regions using dynamic masking, preprocessed with CLAHE and Gaussian blurring for illumination invariance.
* **Matching & Geometric Filtering:** Employs K-Nearest Neighbors (KNN) with Lowe's ratio test.
* **Outlier Rejection:** Sequentially applies cross-checking, spatial displacement filtering, and USAC-based Essential Matrix estimation.
* **Pose Estimation:** Lifts 2D keypoints to 3D camera coordinates using bilinear depth interpolation.
* **Trajectory Computation:** Resolves 3D transformations between frames via an iterative Kabsch algorithm (SVD), and updates the global transformation recursively.
* **Performance Evaluation:** The system was developed and evaluated using the TUM RGB-D SLAM Dataset and accompanying tools.
  
![Example Result](https://github.com/maxwell6q/Previous_Coursework/tree/main/Computer_Vision/Figures/vo_results.png)


## Midterm Project
This repository contains a complete C++ and OpenCV-based object detection system that identifies known objects in synthetic scenes using local feature matching.

* **Feature Extraction & Matching:** The pipeline supports SIFT, SURF, ORB, and AKAZE detectors, optionally applying Gaussian smoothing for noise reduction.
It stores training descriptors and matches them to test images using OpenCV's BFMatcher.
* **Outlier Rejection:** False positives are aggressively filtered using Lowe's ratio test to preserve only the most distinct feature correspondences.
* **Bounding Box Generation:** The system implements three distinct localization methods:
a standard sliding box, a contour-based box, and a centered sliding box that adjusts its position based on the center of mass of the detected keypoints.
* **Performance Evaluation:** The system's effectiveness is quantitatively measured using standard computer vision metrics,
specifically Mean Intersection over Union (mIoU) and Detection Accuracy. The known objects included a Power Drill, a Mustard Bottle and a Sugar Box.

![Example Result](https://github.com/maxwell6q/Previous_Coursework/tree/main/Computer_Vision/Figures/obj_matching.png)
