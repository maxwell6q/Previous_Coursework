# CV25 Intermediate Project – Object Detection

<details>
  <summary><strong> Table of Contents</strong></summary>

- [Build Instructions](#build-instructions)
- [Objective](#objective)
  - [Inputs](#inputs)
  - [Outputs](#outputs)
  - [Suggested Approach](#suggested-approach)
  - [Dataset](#dataset)
  - [Evaluation Metrics](#evaluation-metrics)
  - [Project Requirements](#project-requirements)

</details>

## **Build Instructions**
```
# To start the container on the VLAB env
start_opencv
# Create the build directory in the root of the project 
mkdir build
cd build

# Configure the project using CMake
cmake ..

# Compile the project
make

# Run the executable
./CV_Midterm
```
The above instructions will create a `/results` directory with two subdirectories one for the `/images` and the other for the `/coordinates` of the bounding boxs computed by the algorithm. 


##  **Objective**
Develop an **object detection system** using **C++ with OpenCV** to identify and localize three known objects:
- Power Drill
- Mustard Bottle
- Sugar Box

The system should detect each object in a scene image and draw a bounding box around it.

>  **Deep learning methods are not allowed.**
---

### Inputs
- RGB scene image
- Synthetic views of each object
- Binary segmentation masks (optional)

---

###  Outputs
- Text file containing bounding box coordinates
- Scene image with bounding boxes overlaid

---


### Dataset
[Dataset Download](https://drive.google.com/drive/folders/1heXAbX4WKXf3-z2sl68Qg-cvbcVwosxO?usp=sharing)

Each object has:
- `models/` — 60 synthetic views + segmentation masks
- `test_images/` — Images with scenes for detection
- `labels/` — Ground truth in the format:  <object_id>_<object_name> <xmin> <ymin> <xmax> <ymax>

---

### Evaluation Metrics
- **Mean Intersection over Union (mIoU):**
Average IoU across all object categories
- **Detection Accuracy:**
Number of correct detections (IoU > 0.5 with ground truth)

---

### Project Requirements
- Implemented in **C++ with OpenCV**
- Must compile and run in the official **Virtual Lab**
- Only original code written by group members is allowed

---






