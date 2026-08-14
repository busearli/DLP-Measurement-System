#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

constexpr float SCREW_WIDTH_MM  = 220.41f;
constexpr float SCREW_HEIGHT_MM = 145.60f;

std::vector<cv::Point2f> detectScrewCandidates(
    const cv::Mat& grayUndistorted,
    cv::Mat& debugDisplay
);

bool selectBestScrewQuad(
    const std::vector<cv::Point2f>& candidates,
    std::vector<cv::Point2f>& bestQuad
);

bool sanityCheckQuad(
    const std::vector<cv::Point2f>& ordered,
    std::string& reason
);
