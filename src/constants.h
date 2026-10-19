#include <filesystem>
#include <string>

const std::filesystem::path DATA_FOLDER_PATH = std::filesystem::current_path() / "data";
const std::filesystem::path POSES_FOLDER_PATH = DATA_FOLDER_PATH / "poses";
const std::filesystem::path SEQUENCES_FOLDER_PATH = DATA_FOLDER_PATH / "sequences";
const std::string PNG_FILE_EXTENSION = ".png";
const std::string CALIBRATION_FILENAME = "calib.txt";
