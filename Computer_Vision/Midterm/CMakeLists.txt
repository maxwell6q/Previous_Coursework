cmake_minimum_required(VERSION 3.10)
project(CV_Midterm)


# Find OpenCV
find_package(OpenCV REQUIRED)


# Specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)


# Add the executable and include source files from multiple directories
set(EVALUATOR
    src/Evaluator/evaluator_NihalSuri.cpp
)

set(FEATURE_DETECTOR_MATCHER
    src/FeatureDetectorMatcher/featureDetection_MaxPries.cpp
    src/FeatureDetectorMatcher/featureMatching_MaxPries.cpp
)

set(OBJECT_DETECTOR
    src/ObjectDetector/ObjectDetector_BjornMagnus.cpp
    src/ObjectDetector/objectDetectorAll_MaxPries.cpp
)

set(UTILS
    src/Utils/inputProcessing_MaxPries.cpp
    src/Utils/OutputProcessing_BjornMagnus.cpp
    src/Utils/outputProcessing_NihalSuri.cpp
)

add_executable(CV_Midterm 
    src/main.cpp
    ${EVALUATOR}
    ${FEATURE_DETECTOR_MATCHER}
    ${OBJECT_DETECTOR}
    ${UTILS}
)


# Link OpenCV to project
target_link_libraries(CV_Midterm ${OpenCV_LIBS})


# Include the directory for header files
target_include_directories(CV_Midterm 
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)