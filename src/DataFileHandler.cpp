#include "DataFileHandler.h"

#include "constants.h"

#include <filesystem>
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

int DataFileHandler::getSequenceImageCount(int sequenceID) const
{
    const std::filesystem::path imageDirectory = getSequenceImageFolderPathBySequenceID(sequenceID);

    int imageCount = 0;
    for (const auto& entry : std::filesystem::directory_iterator(imageDirectory)) {
        if (entry.is_regular_file()) {
            ++imageCount;
        }
    }

    return imageCount;
}

std::filesystem::path DataFileHandler::getImageFilePath(
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