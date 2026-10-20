#include "DataFileHandler.h"

#include "constants.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

DataFileHandler::DataFileHandler() = default;

std::filesystem::path DataFileHandler::getSequenceImageFolderPathBySequenceID(int sequenceID) const
{
    if (sequenceID < 0 || sequenceID > 21) {
        throw std::invalid_argument("Sequence ID must be between 0 and 21");
    }

    std::stringstream sequenceName;
    sequenceName << std::setfill('0') << std::setw(2) << sequenceID;

    const std::filesystem::path imageDirectory =
        SEQUENCES_FOLDER_PATH / sequenceName.str() / "image_0";

    if (!std::filesystem::exists(imageDirectory)) {
        throw std::runtime_error("Image directory does not exist: " + imageDirectory.string());
    }

    if (!std::filesystem::is_directory(imageDirectory)) {
        throw std::runtime_error("Image path is not a directory: " + imageDirectory.string());
    }

    return imageDirectory;
}

int DataFileHandler::getSequenceImageCountBySequenceID(int sequenceID) const
{
    const std::filesystem::path imageDirectory = getSequenceImageFolderPathBySequenceID(sequenceID);

    int imageCount = 0;
    for (const auto& entry : std::filesystem::directory_iterator(imageDirectory)) {
        if (entry.is_regular_file()) {
            imageCount++;
        }
    }

    return imageCount;
}

bool DataFileHandler::sequenceHasGroundTruthPoses(int sequenceID) const
{
    if (sequenceID < 0 || sequenceID > 21) {
        throw std::invalid_argument("Sequence ID must be between 0 and 21");
    }

    return sequenceID <= 10;
}

cv::Mat DataFileHandler::getSequencePoseBySequenceIDAndImageIndex(
    int sequenceID, int imageIndex) const
{
    if (sequenceID < 0 || sequenceID > 21) {
        throw std::invalid_argument("Sequence ID must be between 0 and 21");
    }

    if (imageIndex < 0) {
        throw std::invalid_argument("Image index cannot be negative");
    }

    if (!sequenceHasGroundTruthPoses(sequenceID)) {
        return cv::Mat();
    }

    std::stringstream sequenceName;
    sequenceName << std::setfill('0') << std::setw(2) << sequenceID;

    const std::filesystem::path poseFilePath =
        POSES_FOLDER_PATH / (sequenceName.str() + ".txt");

    if (!std::filesystem::exists(poseFilePath)) {
        throw std::runtime_error(
            "Pose file does not exist: " + poseFilePath.string());
    }

    std::ifstream poseFile(poseFilePath);
    if (!poseFile) {
        throw std::runtime_error(
            "Could not open pose file: " + poseFilePath.string());
    }

    std::string poseLine;
    for (int lineIndex = 0; lineIndex <= imageIndex; lineIndex++) {
        if (!std::getline(poseFile, poseLine)) {
            throw std::out_of_range(
                "Image index has no corresponding pose: " +
                std::to_string(imageIndex));
        }
    }

    std::stringstream poseValues(poseLine);
    cv::Mat pose = cv::Mat::zeros(3, 4, CV_64F);
    for (int row = 0; row < pose.rows; row++) {
        for (int column = 0; column < pose.cols; column++) {
            if (!(poseValues >> pose.at<double>(row, column))) {
                throw std::runtime_error(
                    "Pose line must contain 12 values: " +
                    std::to_string(imageIndex));
            }
        }
    }

    return pose;
}

std::filesystem::path DataFileHandler::getImageFilePathByImageIndex(
    const std::filesystem::path& imageFolderPath, int imageIndex) const
{
    if (imageIndex < 0) {
        throw std::invalid_argument("Image index cannot be negative");
    }

    std::stringstream imageFilename;
    imageFilename << std::setfill('0') << std::setw(6) << imageIndex
                  << PNG_FILE_EXTENSION;

    return imageFolderPath / imageFilename.str();
}

std::filesystem::path
DataFileHandler::getSequenceCalibFilePathBySequenceID(int sequenceID) const
{
    if (sequenceID < 0 || sequenceID > 21) {
        throw std::invalid_argument("Sequence ID must be between 0 and 21");
    }

    std::stringstream sequenceName;
    sequenceName << std::setfill('0') << std::setw(2) << sequenceID;

    const std::filesystem::path calibrationFile =
        SEQUENCES_FOLDER_PATH / sequenceName.str() / CALIBRATION_FILENAME;

    if (!std::filesystem::exists(calibrationFile)) {
        throw std::runtime_error(
            "Calibration file does not exist: " + calibrationFile.string());
    }

    if (!std::filesystem::is_regular_file(calibrationFile)) {
        throw std::runtime_error(
            "Calibration path is not a regular file: " + calibrationFile.string());
    }

    return calibrationFile;
}