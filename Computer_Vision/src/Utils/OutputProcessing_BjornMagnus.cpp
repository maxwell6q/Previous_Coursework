#include "Utils/OutputProcessing_BjornMagnus.h"

void openImage(cv::Mat &img,float Resize = 1, std::string name = "Window"){
    cv::resize(img, img, cv::Size(), Resize, Resize);  //Resize the image to given format 
    cv::namedWindow(name, cv::WINDOW_AUTOSIZE);        //Gives the name to the window being opened 
    cv::imshow(name, img);                             //Open the given image with given name 
    cv::waitKey(0);                                    //allows window to render the image and wait for a key to be pressed
}
