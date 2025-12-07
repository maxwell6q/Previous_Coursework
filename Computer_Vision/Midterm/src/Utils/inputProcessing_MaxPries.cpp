#include <iostream>
#include <filesystem>

#include "Utils/inputProcessing_MaxPries.h"


void loadInput(cv::Mat &img, const std::string path, bool grayscale)
{
    if (!grayscale) img = cv::imread(path);
    else img = cv::imread(path, cv::IMREAD_GRAYSCALE);

    // Check image and abort if necessary
    if (img.empty())
    {
        std::cerr << "Error: Could not load image!" << std::endl;
        exit(1);
    }
    
}


std::vector<std::string> fetchFilepaths(const std::string sourceDir, const std::string ending)
{
    namespace fs = std::filesystem;

    std::vector<std::string> pathList;

    fs::path rootPath = sourceDir;

    // Loops recursively over every entry in the provided directory
    for (auto dirEntry : fs::recursive_directory_iterator(rootPath))
    {
        // Ignore directories etc.
        if (!dirEntry.is_regular_file()) continue;

        std::string imgPath = dirEntry.path().string();

        // Sort out only those entries that are the relevant training data and
        // store their names in a vector
        if (imgPath.size() >= ending.length() && imgPath.ends_with(ending))
        {
            pathList.push_back(imgPath);
        }
    }
    return pathList;
}



void loadTrainTestData(std::vector<std::vector<std::string>> &allModels,
    std::vector<std::vector<std::string>> &allMasks,
    std::vector<std::string> &allTests)
{
    // Collect all file-paths corresponding to each object and type of image
    // sugar box
    std::vector<std::string> pathListSugarModels = fetchFilepaths("../data/004_sugar_box/models", "color.png");
    std::vector<std::string> pathListSugarMasks = fetchFilepaths("../data/004_sugar_box/models", "mask.png");
    std::vector<std::string> pathListSugarTests = fetchFilepaths("../data/004_sugar_box/test_images", "color.jpg");

    // mustard bottle
    std::vector<std::string> pathListMustardModels = fetchFilepaths("../data/006_mustard_bottle/models", "color.png");
    std::vector<std::string> pathListMustardMasks = fetchFilepaths("../data/006_mustard_bottle/models", "mask.png");
    std::vector<std::string> pathListMustardTests = fetchFilepaths("../data/006_mustard_bottle/test_images", "color.jpg");

    // power drill
    std::vector<std::string> pathListDrillModels = fetchFilepaths("../data/035_power_drill/models", "color.png");
    std::vector<std::string> pathListDrillMasks = fetchFilepaths("../data/035_power_drill/models", "mask.png");
    std::vector<std::string> pathListDrillTests = fetchFilepaths("../data/035_power_drill/test_images", "color.jpg");

    // Concattenate them based on type
    allModels = {pathListSugarModels, pathListMustardModels, pathListDrillModels};
    allMasks = {pathListSugarMasks, pathListMustardMasks, pathListDrillMasks};
    allTests = pathListSugarTests;
    allTests.insert(allTests.end(), pathListMustardTests.begin(), pathListMustardTests.end());
    allTests.insert(allTests.end(), pathListDrillTests.begin(), pathListDrillTests.end());
}