#include "Utils/outputProcessing_NihalSuri.h"


void saveImage(cv::Mat& img, 
               const std::string& imagePath, 
               const std::string& savePath, 
               const std::string& addedName) {
    // Convert savePath to a filesystem path
    std::filesystem::path saveDir(savePath);

    // Create the directory and any necessary parent directories if they don't exist
    try {
        if (!std::filesystem::exists(saveDir)) {
            std::filesystem::create_directories(saveDir);
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error creating directory: " << e.what() << std::endl;
        return;
    }

    // Extract the filename without extension from the original image path
    std::filesystem::path origPathFs(imagePath);
    std::string filename = origPathFs.stem().string();

    // Construct the full path for the new image
    std::filesystem::path newFilename = saveDir / (filename + addedName);

    // Save the image to the specified path
    try {
        cv::imwrite(newFilename.string(), img);
        std::cout << "Image saved to: " << newFilename << std::endl;
        
    } catch (const cv::Exception& e) {
        std::cerr << "Error saving image: " << e.what() << std::endl;
    }

}


void writeCoordinates(const std::vector<cv::Rect>& rect, 
                      const std::string& imagePath, 
                      const std::string& savePath, 
                      const std::string& addedName){

    // Define the object classes in the required order
    const std::vector<std::string> objectClasses = {
        "004_sugar_box",
        "006_mustard_bottle",
        "035_power_drill"
    };

    // Create the directory if it doesn't exist
    try {
        if (!std::filesystem::exists(savePath)) {
            std::filesystem::create_directories(savePath);
            std::cout << "Created directory: " << savePath << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error creating directory: " << e.what() << std::endl;
        return;
    }

    // Extract the base filename from the image path
    std::string baseFilename = imagePath;
    
    // Remove directory part if present
    size_t lastSlash = baseFilename.find_last_of("/\\");
    if (lastSlash != std::string::npos) {
        baseFilename = baseFilename.substr(lastSlash + 1);
    }
    
    // Remove extension if present
    size_t lastDot = baseFilename.find_last_of('.');
    if (lastDot != std::string::npos) {
        baseFilename = baseFilename.substr(0, lastDot);
    }
        
    // Create the output filename with the added name
    std::string outputFilename = savePath + baseFilename + addedName + ".txt";

    // Open the output file
    std::ofstream outFile(outputFilename);
    if (!outFile.is_open()) {
        std::cerr << "Failed to open output file: " << outputFilename << std::endl;
        return;
    }
    
    // Write the coordinates for each object class
    for (size_t i = 0; i < objectClasses.size(); i++) {
        outFile << objectClasses[i] << " ";
        
        // Check if we have the rectangle for this object class
        if (i < rect.size()) {
            // Convert from (x, y, width, height) to (xmin, ymin, xmax, ymax)
            int xmin = rect[i].x;
            int ymin = rect[i].y;
            int xmax = rect[i].x + rect[i].width;
            int ymax = rect[i].y + rect[i].height;
            
            outFile << xmin << " " << ymin << " " << xmax << " " << ymax;
        } else {
            // Write empty coordinates if this object was not detected
            outFile << "0 0 0 0";
        }
        
        outFile << std::endl;
    }
    
    outFile.close();
    std::cout << "Coordinates written to: " << outputFilename << std::endl;


}