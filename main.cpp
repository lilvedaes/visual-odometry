#include <charconv>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "src/Calibration.h"
#include "src/DataFileHandler.h"

using namespace cv;
using namespace std;

int getSequenceID(const char* arg)
{
    if (arg == nullptr || *arg == '\0') {
        throw invalid_argument("Sequence ID cannot be empty");
    }

    const char* end = arg;
    while (*end != '\0') {
        ++end;
    }

    int sequenceID = 0;
    const auto [parseEnd, errorCode] = from_chars(arg, end, sequenceID);

    if (errorCode != errc() || parseEnd != end) {
        throw invalid_argument("Sequence ID must be an integer between 00 and 21");
    }

    if (sequenceID < 0 || sequenceID > 21) {
        throw invalid_argument("Sequence ID must be between 00 and 21");
    }

    return sequenceID;
}

int main(int argc, char** argv)
{
    // Read arguments for which sequence from 00 to 21 to track
    int sequenceID = 0;
    if (argc <= 1) {
        cout << "Sequence ID not provided, falling back to sequence ID = 00" << endl;
    } else {
        try {
            sequenceID = getSequenceID(argv[1]);
        } catch (const invalid_argument& error) {
            cerr << "Invalid sequence ID: " << error.what() << endl;
            return 1;
        }
    }
    
    DataFileHandler dataFileHandler;
    const int numImages = dataFileHandler.getSequenceImageCount(sequenceID);
    const std::filesystem::path imageFolderPath = dataFileHandler.getSequenceImageFolderPathBySequenceID(sequenceID);
    const std::filesystem::path calibrationFilePath =
        dataFileHandler.getSequenceCalibFilePathBySequenceID(sequenceID);
    const Calibration calibration(calibrationFilePath);

    cout << "P0:" << endl << calibration.getP0() << endl;

    // Loop through all consecutive image pairs.
    for (int i = 0; i + 1 < 5; ++i) {
        // Process image i and image i + 1.
        // Format into a string of 6 digits filled with zeros.
        const std::filesystem::path img1FilePath =
            dataFileHandler.getImageFilePath(imageFolderPath, i);

        cout << img1FilePath << endl;

        const std::filesystem::path img2FilePath =
            dataFileHandler.getImageFilePath(imageFolderPath, i + 1);

        cout << img2FilePath << endl;
    }

    // // Read the image file
    // Mat image = imread("test.jpg");

    // // Check for failure
    // if (image.empty()) 
    // {
    //     cout << "Could not open or find the image" << endl;
    //     return -1;
    // }

    // // Creating feature detector
    // Ptr<FastFeatureDetector> fast = FastFeatureDetector::create();

    // // Find and draw keypoints
    // vector<KeyPoint> kps;
    // fast->detect(image, kps);
    // Mat img2;
    // drawKeypoints(image, kps, img2, (255,0,0));

    // // Create a window
    // namedWindow("OpenCV Test", WINDOW_AUTOSIZE);

    // // Show our image inside the created window.
    // imshow("OpenCV Test", image);
    // imshow("OpenCV Test KeyPoints", img2);

    // // Wait for any keystroke in the window
    // waitKey(0);

    return 0;
}