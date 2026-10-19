#include <filesystem>

class DataFileHandler {
public:
    DataFileHandler();
    std::filesystem::path getSequenceImageFolderPathBySequenceID(int sequenceID) const;
    std::filesystem::path getImageFilePath(
        const std::filesystem::path& imageFolderPath, int imageIndex) const;
    std::filesystem::path getSequenceCalibFilePathBySequenceID(int sequenceID) const;
    int getSequenceImageCount(int sequenceID) const;
};