#!/bin/bash
# Exit if an error occurs 
set -e 

# Global error handler
trap 'echo "Error occurred at line $LINENO"; exit 1' ERR

# =============================================================================
# PARAMETERS
# =============================================================================
    
dataset=1        # 0: freiburg1_rpy;          1: freiburg1_xyz; 
                 # 2: freiburg2_rpy;          3: freiburg2_xyz;
                 # 4: freiburg2_pioneer_360;  5: freiburg2_pioneer_slam
                 # 6: freiburg2_pioneer_slam3

detectorMode=2   # 0: SURF; 1: ORB; 2: SIFT

# Dataset name
datasetNames=("freiburg1_rpy" "freiburg1_xyz" "freiburg2_rpy" "freiburg2_xyz" \
              "freiburg2_pioneer_360" "freiburg2_pioneer_slam" "freiburg2_pioneer_slam3")

# =============================================================================
# Read input arguments
# =============================================================================
    
if [ ! -z "$1" ]; then
    dataset=$1
fi

if [ ! -z "$2" ]; then
    detectorMode=$2
fi

# =============================================================================
# Build and Compilation
# =============================================================================

if [ ! -d "build" ]; then
    mkdir "build"
fi

cmake -B build
make -s -j$(nproc) -C build

# =============================================================================
# Prepare Results Directories
# =============================================================================

mkdir -p results/ate
mkdir -p results/rpe

# =============================================================================
# Execute Scripts
# =============================================================================

# Create associated_stamps for the selected dataset  
python3 src/associate.py \
    data/rgbd_dataset_${datasetNames[$dataset]}/rgb.txt \
    data/rgbd_dataset_${datasetNames[$dataset]}/depth.txt \
    > data/rgbd_dataset_${datasetNames[$dataset]}/associated_stamps.txt

# Run the Visual Odometry program
cd build
./VisualOdometry_Final $dataset $detectorMode
cd ..

# Run evaluation using ATE
echo "Running Python evaluation using ATE"
python3 src/evaluate_ate.py \
    data/rgbd_dataset_${datasetNames[$dataset]}/groundtruth.txt \
    results/results_${datasetNames[$dataset]}.txt \
    --verbose
echo

# Run evaluation using RPE
echo "Running Python evaluation using RPE"
python3 src/evaluate_rpe.py \
    data/rgbd_dataset_${datasetNames[$dataset]}/groundtruth.txt \
    results/results_${datasetNames[$dataset]}.txt \
    --fixed_delta \
    --verbose
