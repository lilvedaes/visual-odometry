#include "Calibration.h"

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
cv::Mat readProjectionMatrix(
    std::ifstream& calibrationFile, const std::string& expectedLabel)
{
    std::string label;
    if (!(calibrationFile >> label) || label != expectedLabel) {
        throw std::runtime_error(
            "Expected calibration entry " + expectedLabel);
    }

    std::vector<double> values(12);
    for (double& value : values) {
        if (!(calibrationFile >> value)) {
            throw std::runtime_error(
                "Calibration entry " + expectedLabel + " must contain 12 values");
        }
    }

    return cv::Mat(3, 4, CV_64F, values.data()).clone();
}
}

Calibration::Calibration(const std::filesystem::path& calibrationFilePath)
{
    std::ifstream calibrationFile(calibrationFilePath);
    if (!calibrationFile) {
        throw std::runtime_error(
            "Could not open calibration file: " + calibrationFilePath.string());
    }

    p0 = readProjectionMatrix(calibrationFile, "P0:");
    p1 = readProjectionMatrix(calibrationFile, "P1:");
    p2 = readProjectionMatrix(calibrationFile, "P2:");
    p3 = readProjectionMatrix(calibrationFile, "P3:");
}

const cv::Mat& Calibration::getP0() const
{
    return p0;
}

const cv::Mat& Calibration::getP1() const
{
    return p1;
}

const cv::Mat& Calibration::getP2() const
{
    return p2;
}

const cv::Mat& Calibration::getP3() const
{
    return p3;
}
