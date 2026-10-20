#include <charconv>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "Calibration.h"
#include "DataFileHandler.h"
#include "VisualOdometry.h"
#include "constants.h"

using namespace cv;
using namespace std;

int getSequenceIDArg(const char* arg)
{
    if (arg == nullptr || *arg == '\0') {
        throw invalid_argument("Sequence ID cannot be empty");
    }

    const char* end = arg;
    while (*end != '\0') {
        end++;
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
            sequenceID = getSequenceIDArg(argv[1]);
        } catch (const invalid_argument& error) {
            cerr << "Invalid sequence ID: " << error.what() << endl;
            return 1;
        }
    }
    
    // Initializing important values
    const DataFileHandler dataFileHandler;
    const bool hasGroundTruthPoses =
        dataFileHandler.sequenceHasGroundTruthPoses(sequenceID);
    if (!hasGroundTruthPoses) {
        cerr
            << "Warning: sequence " << sequenceID
            << " has no ground-truth poses. Visual odometry will not have "
            << "real-world scale for this sequence." << endl;
    }

    const int numImages = dataFileHandler.getSequenceImageCountBySequenceID(sequenceID);
    const std::filesystem::path imageFolderPath = dataFileHandler.getSequenceImageFolderPathBySequenceID(sequenceID);
    const std::filesystem::path calibrationFilePath =
        dataFileHandler.getSequenceCalibFilePathBySequenceID(sequenceID);
    const Calibration calibration(calibrationFilePath);
    VisualOdometry visualOdometry;

    // Global cumulative R and t
    cv::Mat R_f = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t_f = cv::Mat::zeros(3, 1, CV_64F);

    // Get intrinsic camera matrix
    cv::Mat K = calibration.getP0()(
        cv::Range(0, 3),
        cv::Range(0, 3)
    ).clone();

    // Process image 1 (i)
    // Format into a string of 6 digits filled with zeros
    std::filesystem::path img1FilePath =
        dataFileHandler.getImageFilePathByImageIndex(imageFolderPath, 0);
    vector<cv::Point2f> img1Points = 
        visualOdometry.getKeyPointsByImageIndex(img1FilePath, 0);
    cv::Mat img1GroundTruthPose;
    if (hasGroundTruthPoses) {
        img1GroundTruthPose = 
            dataFileHandler.getSequencePoseBySequenceIDAndImageIndex(sequenceID, 0);
    }

    // Visual Odometry Pipeline
    // Loop through all consecutive image pairs
    // TODO: Switch back to image count
    for (int i = 0; i + 1 < 50; i++) {
        // 1. Read image 2 (i + 1)
        // Format into a string of 6 digits filled with zeros
        const std::filesystem::path img2FilePath =
            dataFileHandler.getImageFilePathByImageIndex(imageFolderPath, i + 1);

        // 2. Lucas Kanade with img 1 and img 2 (i and i+1)
        std::vector<cv::Point2f> img2Points = 
            visualOdometry.getKeyPointsLucasKanade(img1Points, img1FilePath, img2FilePath);

        // 3. Find essential mat, Outlier filtering with RANSAC
        cv::Mat inlierMask;
        cv::Mat essentialMatrix = cv::findEssentialMat(
            img1Points,
            img2Points,
            K,
            cv::RANSAC,
            0.999,
            1.0,
            1000,
            inlierMask
        );
        
        // 4. Recover camera pose from essential mat
        cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
        int inliers = cv::recoverPose(
            essentialMatrix, 
            img1Points, 
            img2Points, 
            K, 
            R, 
            t, 
            inlierMask
        );

        // 5. If available, calculate scale from poses
        cv::Mat img2GroundTruthPose;
        if (hasGroundTruthPoses) {
            img2GroundTruthPose = 
                dataFileHandler.getSequencePoseBySequenceIDAndImageIndex(sequenceID, i + 1);
            double scale = 
                visualOdometry.calculateScaleFromPoses(img1GroundTruthPose, img2GroundTruthPose);
            t = scale * t;
        }

        // 6. Accumulate trajectory in global transformation matrix

        // a. Rotate that vector so it aligns with the drone's current global orientation (R_f * t)
        // b. Add it to the drone's current global position (t_f)
        t_f = t_f + (R_f * t);

        // Update the drone's global rotation
        R_f = R * R_f;

        // 7. Check threshold to recalculate FAST
        if (inliers < FEATURE_MINIMUM_THRESHOLD) {
            img2Points = 
                visualOdometry.getKeyPointsByImageIndex(img2FilePath, i + 1);
        }

        // 8. Draw trajectory + ground truth if available

        // 9. Reset
        img1FilePath = img2FilePath;
        img1Points = img2Points;

        if (hasGroundTruthPoses) {
            img1GroundTruthPose = img2GroundTruthPose;
        }
    }

    return 0;
}