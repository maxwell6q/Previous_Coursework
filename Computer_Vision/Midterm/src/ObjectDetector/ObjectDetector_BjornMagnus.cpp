#include "ObjectDetector/ObjectDetector_BjornMagnus.h"


void DOslidingBox(const cv::Mat &img, std::vector<cv::KeyPoint> &Keypoints, const int boxWidth, const int boxHeight, cv::Rect &bestBox){
    cv::Rect leadingBox;
    int bestCount = 0;
    int featureCountH, featureCountV; 

    for (int y = 0; y < img.rows - boxHeight; y += 10){
        for (int x = 0; x < img.cols - boxWidth; x += 10) {
            // Make a rectangle in the given cordinates 
            cv::Rect boxH(x, y, boxWidth, boxHeight);
            cv::Rect boxV(x,y,boxHeight,boxWidth);
            featureCountH = 0;
            featureCountV = 0; 

            // Count the number of keypints in the rectangle
            for (int a = 0; a<Keypoints.size() ;++a){
                if(boxH.contains(Keypoints[a].pt)) featureCountH++; 
                if(boxV.contains(Keypoints[a].pt)) featureCountV++; 
            }
            // Set the given horizontal recangle to the best of it it contains more keypoint than the best one
            if (featureCountH>bestCount){
                bestCount=featureCountH;
                leadingBox = boxH;
            }
            // Set the given vertical recangle to the best of it it contains more keypoint than the best one
            if (featureCountV>bestCount){
                bestCount=featureCountV;
                leadingBox = boxV;
            }
            
        }
    }
    // Set the best box to the rectangle with containing the most features after searching the image 
    bestBox=leadingBox;
}


void DOcenteredBox(const cv::Mat &img, std::vector<cv::KeyPoint> &Keypoints, const int boxWidth, const int boxHeight, cv::Rect &bestBoxCentred){
    cv::Rect leadingBox;
    int bestCount = 0;
    int featureCountH, featureCountV; 
    std::vector<cv::KeyPoint> keypointInRect;

    for (int y = 0; y < img.rows - boxHeight; y += 10){
        for (int x = 0; x < img.cols - boxWidth; x += 10) {
            // Make a rectangle in the given cordinates 
            cv::Rect boxH(x, y, boxWidth, boxHeight);
            cv::Rect boxV(x,y,boxHeight,boxWidth);
            featureCountH = 0;
            featureCountV = 0; 

            // Count the number of keypints in the rectangle
            for (int a = 0; a<Keypoints.size() ;++a){
                if(boxH.contains(Keypoints[a].pt)) featureCountH++; 
                if(boxV.contains(Keypoints[a].pt)) featureCountV++; 
            }
            // Set the given horizontal recangle to the best of it it contains more keypoint than the best one
            if (featureCountH>bestCount){
                bestCount=featureCountH;
                leadingBox = boxH;
            }
            // Set the given vertical recangle to the best of it it contains more keypoint than the best one
            if (featureCountV>bestCount){
                bestCount=featureCountV;
                leadingBox = boxV;
            }
            
        }
    }
    // Make a list of all the keypoints in rectangle that contains the most features 
    for (int a = 0; a<Keypoints.size() ;++a){
        if(leadingBox.contains(Keypoints[a].pt)) keypointInRect.push_back(Keypoints[a]);
    }
    // Calcutate the centre of all the given keypoints 
    cv::Point2d centroid(0,0);
    for (auto &kp : keypointInRect) {
        centroid.x += kp.pt.x;
        centroid.y += kp.pt.y;
    }
    centroid.x /= keypointInRect.size();
    centroid.y /= keypointInRect.size();

    // Compute the rectangle with the given centre 
    int W = leadingBox.width , H = leadingBox.height;
    int x0 = cvRound(centroid.x - W/2.0);
    int y0 = cvRound(centroid.y - H/2.0);
    cv::Rect centeredBox(x0, y0, W, H);
    bestBoxCentred=centeredBox;
}


void DOconturs(const cv::Mat &inputImg,std::vector<cv::Rect> &detectedBoxes, int maxArea = 100000) {
    cv::Mat edges, img_gray;
    std::vector<std::vector<cv::Point>> contours;
    cv::cvtColor(inputImg, img_gray, cv::COLOR_BGR2GRAY);
    //aplly canny to highligth edges 
    cv::Canny(img_gray, edges, 37, 302);
    //find the contours from the output of canny 
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //Make rectangles bounding the contours 
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Rect box = cv::boundingRect(contours[i]);
        if (box.area()<maxArea) detectedBoxes.push_back(box);
    }
}

void fitBestBox(std::vector<cv::KeyPoint> &Keypoints,std::vector<cv::Rect> boxes,cv::Rect &bestBox){
    cv::Rect leadingBox;
    int featureCount = 0; 
    int bestCount = 0;

    // Go though all the boxes given 
    for (const auto& box : boxes) {
        featureCount = 0; 
        // Count the keypoints in the given rectangle 
        for (int a = 0; a<Keypoints.size() ;++a){
            if(box.contains(Keypoints[a].pt))featureCount++; 

            }
        // If the rectangle contains the most features set it as the leading one
        if (featureCount>bestCount){
            bestCount=featureCount;
            leadingBox = box;
        }
    }
    // Set the box containg the most features to the bestBox
    bestBox = leadingBox;
}


void boxDecider(std::vector<cv::KeyPoint> &Keypoints,cv::Rect &box, int minKeypoints){
    int featureCount = 0;

    //Counts the number of keypoints contained in the box 
    for (int a = 0; a<Keypoints.size() ;++a){
        if(box.contains(Keypoints[a].pt))featureCount++;
    }
    // Set the box to an empty Rect if the minimum keypoints contained is not reached 
    if (featureCount<minKeypoints)box = cv::Rect();
    
}


void runDetection(std::string imagePath, cv::Mat &descriptorList, int mode,
                  int minKeypoints, cv::Rect &objectDeteced,
                  std::vector<cv::KeyPoint> &goodKeypoints,
                  double sigma, double cutOffRatio, int boxHeight, int boxWidth,
                  int detectorMode){
   
    cv::Mat Image;
    cv::Mat descriptorsTest;
    std::vector<cv::KeyPoint> keypointsTest;


    //Load one test image and compute its keypoints
    loadInput(Image, imagePath);
    computeFeaturesSingleImage(Image, descriptorsTest, keypointsTest, sigma, detectorMode);
    
    //Match the features of the image to those of the model
    std::vector<std::vector<cv::DMatch>> matches;
    matchFromDescriptors(descriptorsTest, descriptorList, matches);

    //Filter only good matches, using the ratio test
    std::vector<cv::DMatch> goodMatches;
    ratioTest(matches, goodMatches, cutOffRatio);


    //Select all keypoints of the test image corresponding to good matches
    keypointSelection(goodKeypoints, goodMatches, keypointsTest);

    cv::Rect bestBox;
    if(mode==1){
    //Find position of rectangl that fit most keypoints 
    DOslidingBox(Image,goodKeypoints,boxHeight,boxWidth,bestBox);
    }
    else if(mode==2){
    //Find position of rectangl that fit most keypoints and centre the rectangle around the keypoints 
    DOcenteredBox(Image,goodKeypoints,boxHeight,boxWidth,bestBox);
    }
    else {
    //Find all the object based on canny edge detection 
    std::vector<cv::Rect> boxes;
    DOconturs(Image,boxes, 100000);
    //Find the rectangle that fits the most keypoints 
    fitBestBox(goodKeypoints,boxes, bestBox);
    }

    //Decide if the box actually is the object or a false positive 
    boxDecider(goodKeypoints, bestBox,  minKeypoints);

    //Sets the objectdetected 
    objectDeteced = bestBox; 
    
}

