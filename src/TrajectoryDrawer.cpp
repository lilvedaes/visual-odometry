#include "TrajectoryDrawer.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <limits>
#include <stdexcept>

TrajectoryDrawer::TrajectoryDrawer(
    const std::string& windowName,
    const cv::Size& windowSize,
    const cv::Size& trajectorySize,
    const cv::Scalar& estimatedTrajectoryColor,
    const cv::Scalar& groundTruthTrajectoryColor,
    int trajectoryLineThickness,
    int displayDelayMilliseconds
)
    : windowName(windowName),
      windowSize(windowSize),
      trajectorySize(trajectorySize),
      trajectory(cv::Mat::zeros(trajectorySize, CV_8UC3)),
      trajectoryScale(1.0),
      estimatedTrajectoryColor(estimatedTrajectoryColor),
      groundTruthTrajectoryColor(groundTruthTrajectoryColor),
      segments(),
      minX(std::numeric_limits<double>::max()),
      maxX(std::numeric_limits<double>::lowest()),
      minZ(std::numeric_limits<double>::max()),
      maxZ(std::numeric_limits<double>::lowest()),
      trajectoryLineThickness(trajectoryLineThickness),
      displayDelayMilliseconds(displayDelayMilliseconds)
{
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, windowSize.width, windowSize.height);
}

cv::Point3d TrajectoryDrawer::toPoint3D(const cv::Mat& position)
{
    if (position.rows != 3 || position.cols != 1
        || position.type() != CV_64F) {
        throw std::invalid_argument(
            "Trajectory position must be a 3x1 CV_64F matrix");
    }

    return cv::Point3d(
        position.at<double>(0, 0),
        position.at<double>(1, 0),
        position.at<double>(2, 0)
    );
}

cv::Point TrajectoryDrawer::toTrajectoryPoint(
    const cv::Point3d& position) const
{
    const double largestDimension =
        std::max({maxX - minX, maxZ - minZ, 1.0});
    const cv::Point origin(
        trajectorySize.width / 2,
        trajectorySize.height / 2
    );
    const cv::Point3d center(
        (minX + maxX) / 2.0,
        0.0,
        (minZ + maxZ) / 2.0
    );
    return cv::Point(
        origin.x + static_cast<int>((position.x - center.x) * trajectoryScale),
        origin.y - static_cast<int>((position.z - center.z) * trajectoryScale)
    );
}

void TrajectoryDrawer::updateBounds(const cv::Point3d& position)
{
    minX = std::min(minX, position.x);
    maxX = std::max(maxX, position.x);
    minZ = std::min(minZ, position.z);
    maxZ = std::max(maxZ, position.z);
}

void TrajectoryDrawer::addSegment(
    const cv::Mat& previousPosition,
    const cv::Mat& currentPosition,
    const cv::Scalar& color)
{
    const cv::Point3d previous = toPoint3D(previousPosition);
    const cv::Point3d current = toPoint3D(currentPosition);
    updateBounds(previous);
    updateBounds(current);
    segments.push_back({previous, current, color});
    redraw();
}

void TrajectoryDrawer::redraw()
{
    const double largestDimension =
        std::max({maxX - minX, maxZ - minZ, 1.0});
    trajectoryScale =
        0.9 * std::min(trajectorySize.width, trajectorySize.height)
        / largestDimension;
    trajectory.setTo(cv::Scalar::all(0));
    for (const TrajectorySegment& segment : segments) {
        cv::line(
            trajectory,
            toTrajectoryPoint(segment.previousPosition),
            toTrajectoryPoint(segment.currentPosition),
            segment.color,
            trajectoryLineThickness
        );
    }
}

void TrajectoryDrawer::drawEstimatedTrajectory(
    const cv::Mat& previousPosition,
    const cv::Mat& currentPosition)
{
    addSegment(
        previousPosition,
        currentPosition,
        estimatedTrajectoryColor
    );
}

void TrajectoryDrawer::drawGroundTruthTrajectory(
    const cv::Mat& previousPose,
    const cv::Mat& currentPose)
{
    if (previousPose.rows != 3 || previousPose.cols != 4
        || currentPose.rows != 3 || currentPose.cols != 4
        || previousPose.type() != CV_64F
        || currentPose.type() != CV_64F) {
        throw std::invalid_argument(
            "Ground-truth poses must be 3x4 CV_64F matrices");
    }

    addSegment(
        previousPose.col(3),
        currentPose.col(3),
        groundTruthTrajectoryColor
    );
}

void TrajectoryDrawer::show() const
{
    cv::imshow(windowName, trajectory);
    cv::waitKey(displayDelayMilliseconds);
}
