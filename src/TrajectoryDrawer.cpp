#include "TrajectoryDrawer.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

TrajectoryDrawer::TrajectoryDrawer(
    const std::string& windowName,
    const cv::Size& windowSize,
    const cv::Size& trajectorySize,
    const cv::Point& trajectoryOrigin,
    double trajectoryScale,
    const cv::Scalar& estimatedTrajectoryColor,
    const cv::Scalar& groundTruthTrajectoryColor,
    int trajectoryLineThickness,
    int displayDelayMilliseconds
)
    : windowName(windowName),
      windowSize(windowSize),
      trajectorySize(trajectorySize),
      trajectory(cv::Mat::zeros(trajectorySize, CV_8UC3)),
      trajectoryOrigin(trajectoryOrigin),
      trajectoryScale(trajectoryScale),
      estimatedTrajectoryColor(estimatedTrajectoryColor),
      groundTruthTrajectoryColor(groundTruthTrajectoryColor),
      trajectoryLineThickness(trajectoryLineThickness),
      displayDelayMilliseconds(displayDelayMilliseconds)
{
}

cv::Point TrajectoryDrawer::toTrajectoryPoint(
    const cv::Point& origin, double scale, const cv::Mat& position)
{
    return cv::Point(
        origin.x + static_cast<int>(position.at<double>(0, 0) * scale),
        origin.y - static_cast<int>(position.at<double>(2, 0) * scale)
    );
}

void TrajectoryDrawer::drawEstimatedTrajectory(
    const cv::Mat& previousPosition,
    const cv::Mat& currentPosition
)
{
    cv::line(
        trajectory,
        toTrajectoryPoint(trajectoryOrigin, trajectoryScale, previousPosition),
        toTrajectoryPoint(trajectoryOrigin, trajectoryScale, currentPosition),
        estimatedTrajectoryColor,
        trajectoryLineThickness
    );
}

void TrajectoryDrawer::drawGroundTruthTrajectory(
    const cv::Mat& previousPose,
    const cv::Mat& currentPose
)
{
    cv::line(
        trajectory,
        toTrajectoryPoint(trajectoryOrigin, trajectoryScale, previousPose.col(3)),
        toTrajectoryPoint(trajectoryOrigin, trajectoryScale, currentPose.col(3)),
        groundTruthTrajectoryColor,
        trajectoryLineThickness
    );
}

void TrajectoryDrawer::show() const
{
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    cv::resizeWindow(windowName, windowSize.width, windowSize.height);
    cv::imshow(windowName, trajectory);
    cv::waitKey(displayDelayMilliseconds);
}
