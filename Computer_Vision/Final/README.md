# CV25 Final Project – Visual Odometry

In this group project a feature-based visual odometry pipeline was developed. In terms of hyperparameters and certain code aspects (e.g. hard-coded relative file paths/names) it is tailored to the [TUM RGB-D SLAM Dataset](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) (Strum, 2012). A detailed description of the task can be found in the [Project Proposal](/Final_Project_Proposal.pdf), and a detailed description of the solution pipeline can be found in the [Final Report](/Final_Project_Report.pdf).

![Trajectory for the `freiburg2_pioneer_360` dataset](/results/ate/freiburg2_pioneer_360.png) 

## **Build Instructions**

### Requirenments
This project is requires a `C++` compiler, `CMake` and the `OpenCV`and `Eigen` libraries.

### Building
This project was designed for compatibility with libraries installed in an appainer on the University of Padova's virtual machines. If it is built in this environment, open the apptainer in the first step by running:

```
start_opencv
```

If it is built in a cusom environment, skip this step. Then direct to the `visual-odometry-main` directory. 

To use the provided `bash` script, download one of the mentioned datasets, unpack it and place the folder in the [`/data`](/data/) folder without renaming. Then run:

```
./run_vo.sh dataset-index detector-mode
```

Where the first input argument is an integer encoding the dataset to be evalated, and the second is an integer encoding the detector mode:

```
Datasets:  0: freiburg1_rpy;  1: freiburg1_xyz; 
           2: freiburg2_rpy;  3: freiburg2_xyz;
           4: freiburg2_pioneer_360;  5: freiburg2_pioneer_slam
           6: freiburg2_pioneer_slam3

Detectors: 0: SURF; 1: ORB; 2: SIFT
```

If a dataset that is not in the list should be evaluated, the easiest way is to edit the [`run_vo.sh`](/run_vo.sh) file, adding the datasets name to the `dataset_names` list and choosing as input to the shell command the corresponding index.



## **Results**
After running the algorithm, the estimated trajectory is saved as a sequence of relative transformations in the [`/results`](/results/) folder, and the resulting performance parameters for ATE (Absolute Trajectory Error) and RPE (Relative Pose Error) are displayed in the terminal. 

For the 6 listed datasets, plots showing the trajectory ([`/ate`](/results/ate/)) and the RPE over frames ([`/rpe`](/results/rpe/)) are provided. 


---
