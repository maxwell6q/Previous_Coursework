cmake_minimum_required(VERSION 3.10)
project(VisualOdometry_Final)

# Use custom OpenCV install if specified
option(USE_LOCAL_OPENCV "Use custom local OpenCV install" OFF)
if(USE_LOCAL_OPENCV)
    message(STATUS "Using custom OpenCV in ~/.local")
    set(OpenCV_DIR "/home/$ENV{HOME}/.local/lib/cmake/opencv4")
endif()


# Find OpenCV & Eigen3 packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
message(STATUS "Eigen include dirs: ${EIGEN3_INCLUDE_DIRS}")


# Specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)


# Add the executable and include source files from multiple directories
set(UTILS
    src/Utils/inputProcessing.cpp
    src/Utils/outputProcessing.cpp
)

set(SRC_BASE
    src/src_base/featureDetection.cpp
    src/src_base/featureMatching.cpp
    src/src_base/poseEstimation.cpp
)

add_executable(VisualOdometry_Final 
    src/main.cpp
    ${UTILS}
    ${SRC_BASE}
)

# Link OpenCV to project
target_link_libraries(VisualOdometry_Final ${OpenCV_LIBS})


# Include the directory for header files
target_include_directories(VisualOdometry_Final 
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
