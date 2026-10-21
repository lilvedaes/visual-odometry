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
#include "TrajectoryDrawer.h"
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
    
    try {
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
        TrajectoryDrawer trajectoryDrawer(
            "Trajectory",
            cv::Size(800, 800),
            cv::Size(800, 800),
            cv::Scalar(0, 0, 255),
            cv::Scalar(0, 255, 0),
            2,
            1
        );
        const std::string imageWindowName = "Current Image";
        cv::namedWindow(imageWindowName, cv::WINDOW_NORMAL);
        cv::resizeWindow(imageWindowName, 800, 600);

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
        for (int i = 0; i + 1 < numImages; i++) {
            // 1. Read image 2 (i + 1)
            // Format into a string of 6 digits filled with zeros
            const std::filesystem::path img2FilePath =
                dataFileHandler.getImageFilePathByImageIndex(imageFolderPath, i + 1);
            const cv::Mat currentImage = cv::imread(
                img2FilePath.string(),
                cv::IMREAD_GRAYSCALE
            );
            if (currentImage.empty()) {
                throw std::runtime_error(
                    "Could not read image: " + img2FilePath.string());
            }

            // 2. Lucas Kanade with img 1 and img 2 (i and i+1)
            const int originalPointCount = static_cast<int>(img1Points.size());
            
            // Points that LucasKanade was able to track
            std::vector<cv::Point2f> trackedImg1Points;
            std::vector<cv::Point2f> trackedImg2Points;
            if (!visualOdometry.trackPointsLucasKanade(
                    img1Points,
                    img1FilePath,
                    img2FilePath,
                    trackedImg1Points,
                    trackedImg2Points)) {
                // Force reset and continue if problem with lucas kanade
                img1FilePath = img2FilePath;
                img1Points = visualOdometry.getKeyPointsByImageIndex(img2FilePath, i + 1);
                if (hasGroundTruthPoses) {
                    img1GroundTruthPose =
                        dataFileHandler.getSequencePoseBySequenceIDAndImageIndex(
                            sequenceID, i + 1);
                }
                continue;
            }

            // 3. Find essential mat, Outlier filtering with RANSAC
            cv::Mat essentialMatrix;

            // Points that survived RANSAC filtering
            std::vector<cv::Point2f> img1InlierPoints;
            std::vector<cv::Point2f> img2InlierPoints;
            if (!visualOdometry.findEssentialMatrix(
                    trackedImg1Points,
                    trackedImg2Points,
                    K,
                    essentialMatrix,
                    img1InlierPoints,
                    img2InlierPoints)) {
                // Force reset and continue if problem with essential matrix
                img1FilePath = img2FilePath;
                img1Points = visualOdometry.getKeyPointsByImageIndex(img2FilePath, i + 1);
                if (hasGroundTruthPoses) {
                    img1GroundTruthPose =
                        dataFileHandler.getSequencePoseBySequenceIDAndImageIndex(
                            sequenceID, i + 1);
                }
                continue;
            }
            
            // 4. Recover camera pose from essential mat
            cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
            cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
            int inliers = 0;
            if (!visualOdometry.recoverCameraPose(
                    essentialMatrix,
                    img1InlierPoints,
                    img2InlierPoints,
                    K,
                    R,
                    t,
                    inliers)) {
                // Force reset and continue if problem with recover camera pose
                img1FilePath = img2FilePath;
                img1Points = visualOdometry.getKeyPointsByImageIndex(img2FilePath, i + 1);
                if (hasGroundTruthPoses) {
                    img1GroundTruthPose =
                        dataFileHandler.getSequencePoseBySequenceIDAndImageIndex(
                            sequenceID, i + 1);
                }
                continue;
            }

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
            const cv::Mat previousEstimatedPosition = t_f.clone();

            // a. Convert recover pose output to camera-center displacement (-R.t() * t)
            // b. Rotate that vector so it aligns with the drone's current global orientation (R_f * )
            // c. Add it to the drone's current global position (t_f +)
            t_f = t_f + (R_f * (-R.t() * t));

            // Update the drone's global rotation
            R_f = R_f * R.t();

            // 7. Check threshold to recalculate FAST
            std::vector<cv::Point2f> nextPoints = img2InlierPoints;
            if (inliers < static_cast<int>(
                    originalPointCount * FEATURE_REFRESH_RATIO)) {
                nextPoints =
                    visualOdometry.getKeyPointsByImageIndex(img2FilePath, i + 1);
            }

            // 8. Draw trajectory + ground truth if available
            trajectoryDrawer.drawEstimatedTrajectory(
                previousEstimatedPosition,
                t_f
            );
            if (hasGroundTruthPoses) {
                trajectoryDrawer.drawGroundTruthTrajectory(
                    img1GroundTruthPose,
                    img2GroundTruthPose
                );
            }
            trajectoryDrawer.drawCoordinates(
                t_f,
                hasGroundTruthPoses ? &img2GroundTruthPose : nullptr
            );
            cv::imshow(imageWindowName, currentImage);
            trajectoryDrawer.show();

            // 9. Reset
            img1FilePath = img2FilePath;
            img1Points = nextPoints;

            if (hasGroundTruthPoses) {
                img1GroundTruthPose = img2GroundTruthPose;
            }
        }

        return 0;
    } catch (const std::exception& error) {
        cerr << "Error: " << error.what() << endl;
        return 1;
    }
}