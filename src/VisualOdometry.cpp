#include "VisualOdometry.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/tracking.hpp>
#include <stdexcept>

VisualOdometry::VisualOdometry() = default;

std::vector<cv::Point2f> VisualOdometry::getKeyPointsByImageIndex(
    const std::filesystem::path& imagePath, int) const
{
    const cv::Mat image = cv::imread(
        imagePath.string(), cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        throw std::runtime_error("Could not open image: " + imagePath.string());
    }

    const cv::Ptr<cv::FastFeatureDetector> fast =
        cv::FastFeatureDetector::create();
    std::vector<cv::KeyPoint> keypoints;
    fast->detect(image, keypoints);

    std::vector<cv::Point2f> points;
    cv::KeyPoint::convert(keypoints, points);
    return points;
}

bool VisualOdometry::trackPointsLucasKanade(
    const std::vector<cv::Point2f>& previousPoints,
    const std::filesystem::path& previousImagePath,
    const std::filesystem::path& currentImagePath,
    std::vector<cv::Point2f>& trackedPreviousPoints,
    std::vector<cv::Point2f>& trackedCurrentPoints) const
{
    const cv::Mat previousImage = cv::imread(
        previousImagePath.string(), cv::IMREAD_GRAYSCALE);
    const cv::Mat currentImage = cv::imread(
        currentImagePath.string(), cv::IMREAD_GRAYSCALE);
    if (previousImage.empty() || currentImage.empty()) {
        throw std::runtime_error("Could not open images for optical flow");
    }

    trackedPreviousPoints.clear();
    trackedCurrentPoints.clear();
    std::vector<unsigned char> status;
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(
        previousImage,
        currentImage,
        previousPoints,
        trackedCurrentPoints,
        status,
        error
    );

    // Keep in previous and current the points that survived lucas kanade tracking
    std::vector<cv::Point2f> validCurrentPoints;
    for (std::size_t i = 0; i < trackedCurrentPoints.size(); ++i) {
        if (status[i]) {
            trackedPreviousPoints.push_back(previousPoints[i]);
            validCurrentPoints.push_back(trackedCurrentPoints[i]);
        }
    }
    trackedCurrentPoints = validCurrentPoints;
    return trackedCurrentPoints.size() >= 5;
}

bool VisualOdometry::findEssentialMatrix(
    const std::vector<cv::Point2f>& previousPoints,
    const std::vector<cv::Point2f>& currentPoints,
    const cv::Mat& cameraMatrix,
    cv::Mat& essentialMatrix,
    std::vector<cv::Point2f>& filteredPreviousPoints,
    std::vector<cv::Point2f>& filteredCurrentPoints
) const
{
    if (previousPoints.size() != currentPoints.size()
        || previousPoints.size() < 5
        || cameraMatrix.empty()) {
        return false;
    }

    cv::Mat inlierMask;
    essentialMatrix = cv::findEssentialMat(
        previousPoints,
        currentPoints,
        cameraMatrix,
        cv::RANSAC,
        0.999,
        1.0,
        1000,
        inlierMask
    );
    if (essentialMatrix.empty() || inlierMask.empty()) {
        return false;
    }

    filteredPreviousPoints.clear();
    filteredCurrentPoints.clear();

    // Get filtered points using the inlier mask (0 if outlier, 1 if inlier)
    for (int i = 0; i < static_cast<int>(previousPoints.size()); ++i) {
        const unsigned char isInlier =
            inlierMask.rows == 1
                ? inlierMask.at<unsigned char>(0, i)
                : inlierMask.at<unsigned char>(i, 0);
        if (isInlier) {
            filteredPreviousPoints.push_back(previousPoints[i]);
            filteredCurrentPoints.push_back(currentPoints[i]);
        }
    }

    return filteredPreviousPoints.size() >= 5;
}

bool VisualOdometry::recoverCameraPose(
    const cv::Mat& essentialMatrix,
    const std::vector<cv::Point2f>& previousPoints,
    const std::vector<cv::Point2f>& currentPoints,
    const cv::Mat& cameraMatrix,
    cv::Mat& rotation,
    cv::Mat& translation,
    int& inlierCount
) const
{
    if (essentialMatrix.empty()
        || previousPoints.size() != currentPoints.size()
        || previousPoints.size() < 5
        || cameraMatrix.empty()) {
        return false;
    }

    inlierCount = cv::recoverPose(
        essentialMatrix,
        previousPoints,
        currentPoints,
        cameraMatrix,
        rotation,
        translation
    );

    return inlierCount >= 5
        && !rotation.empty()
        && !translation.empty();
}

double VisualOdometry::calculateScaleFromPoses(
    const cv::Mat& previousPose, const cv::Mat& currentPose) const
{
    if (previousPose.rows != 3 || previousPose.cols != 4
        || currentPose.rows != 3 || currentPose.cols != 4) {
        throw std::invalid_argument("Poses must be 3x4 matrices");
    }

    const cv::Mat previousPosition = previousPose.col(3);
    const cv::Mat currentPosition = currentPose.col(3);
    return cv::norm(currentPosition - previousPosition);
}