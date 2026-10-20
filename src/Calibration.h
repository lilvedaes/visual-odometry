#pragma once

#include <filesystem>
#include <opencv2/core.hpp>

class Calibration {
private:
    cv::Mat p0;
    cv::Mat p1;
    cv::Mat p2;
    cv::Mat p3;

public:
    explicit Calibration(const std::filesystem::path& calibrationFilePath);

    const cv::Mat& getP0() const;
    const cv::Mat& getP1() const;
    const cv::Mat& getP2() const;
    const cv::Mat& getP3() const;
};
