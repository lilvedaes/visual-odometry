#pragma once

#include <filesystem>
#include <opencv2/core.hpp>

class DataFileHandler {
public:
    DataFileHandler();
    std::filesystem::path getSequenceImageFolderPathBySequenceID(int sequenceID) const;
    std::filesystem::path getImageFilePathByImageIndex(
        const std::filesystem::path& imageFolderPath, int imageIndex) const;
    std::filesystem::path getSequenceCalibFilePathBySequenceID(int sequenceID) const;
    int getSequenceImageCountBySequenceID(int sequenceID) const;
    bool sequenceHasGroundTruthPoses(int sequenceID) const;
    cv::Mat getSequencePoseBySequenceIDAndImageIndex(
        int sequenceID, int imageIndex) const;
};