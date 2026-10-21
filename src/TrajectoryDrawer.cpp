#include "TrajectoryDrawer.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>
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
      hasGroundTruthTrajectory(false),
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
    drawLegend();
}

void TrajectoryDrawer::drawLegend()
{
    const cv::Point legendOrigin(20, 30);
    const int lineLength = 35;
    const int textOffset = 50;
    const int lineSpacing = 30;
    const int legendHeight = hasGroundTruthTrajectory ? 70 : 40;
    cv::rectangle(
        trajectory,
        cv::Point(10, 10),
        cv::Point(260, 10 + legendHeight),
        cv::Scalar(0, 0, 0),
        cv::FILLED
    );
    cv::line(
        trajectory,
        legendOrigin,
        cv::Point(legendOrigin.x + lineLength, legendOrigin.y),
        estimatedTrajectoryColor,
        trajectoryLineThickness
    );
    cv::putText(
        trajectory,
        "Estimated",
        cv::Point(legendOrigin.x + textOffset, legendOrigin.y + 5),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(255, 255, 255),
        1
    );

    if (hasGroundTruthTrajectory) {
        const cv::Point groundTruthLegendOrigin(
            legendOrigin.x,
            legendOrigin.y + lineSpacing
        );
        cv::line(
            trajectory,
            groundTruthLegendOrigin,
            cv::Point(
                groundTruthLegendOrigin.x + lineLength,
                groundTruthLegendOrigin.y
            ),
            groundTruthTrajectoryColor,
            trajectoryLineThickness
        );
        cv::putText(
            trajectory,
            "Ground truth",
            cv::Point(
                groundTruthLegendOrigin.x + textOffset,
                groundTruthLegendOrigin.y + 5
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(255, 255, 255),
            1
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

    hasGroundTruthTrajectory = true;
    addSegment(
        previousPose.col(3),
        currentPose.col(3),
        groundTruthTrajectoryColor
    );
}

void TrajectoryDrawer::drawCoordinates(
    const cv::Mat& estimatedPosition,
    const cv::Mat* groundTruthPose)
{
    const cv::Point3d estimated = toPoint3D(estimatedPosition);
    std::ostringstream estimatedText;
    estimatedText << "Estimated: ("
                  << std::fixed << std::setprecision(2)
                  << estimated.x << ", "
                  << estimated.y << ", "
                  << estimated.z << ")";

    cv::putText(
        trajectory,
        estimatedText.str(),
        cv::Point(20, trajectorySize.height - 45),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        estimatedTrajectoryColor,
        1
    );

    if (groundTruthPose != nullptr) {
        if (groundTruthPose->rows != 3 || groundTruthPose->cols != 4
            || groundTruthPose->type() != CV_64F) {
            throw std::invalid_argument(
                "Ground-truth pose must be a 3x4 CV_64F matrix");
        }

        const cv::Mat groundTruthPosition = groundTruthPose->col(3);
        std::ostringstream groundTruthText;
        groundTruthText << "Ground truth: ("
                        << std::fixed << std::setprecision(2)
                        << groundTruthPosition.at<double>(0, 0) << ", "
                        << groundTruthPosition.at<double>(1, 0) << ", "
                        << groundTruthPosition.at<double>(2, 0) << ")";

        cv::putText(
            trajectory,
            groundTruthText.str(),
            cv::Point(20, trajectorySize.height - 20),
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            groundTruthTrajectoryColor,
            1
        );
    }
}

void TrajectoryDrawer::show() const
{
    cv::imshow(windowName, trajectory);
    cv::waitKey(displayDelayMilliseconds);
}
