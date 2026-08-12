#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <deque>
#include <algorithm>
//--------------------------------------------------
// SABITLER
//--------------------------------------------------

constexpr double CENTER_TOLERANCE_MM = 2.0;
constexpr double ANGLE_TOLERANCE_DEG = 1.0;
constexpr double SIZE_TOLERANCE_PCT  = 3.0;
constexpr double SIZE_SUSPICIOUS_PCT = 35.0;

// Vida referans alaninin fiziksel boyutu
constexpr double SCREW_FRAME_WIDTH_MM  = 222.5;
constexpr double SCREW_FRAME_HEIGHT_MM = 150.0;

//--------------------------------------------------
// DLP MODEL
//--------------------------------------------------

struct DLPModel
{
    std::string name;
    float widthMM;
    float heightMM;
};

// Su an hizli ilerlemek icin model secenekleri kaldirildi.
// Sistem sabit DLP boyutuyla calisiyor.
inline DLPModel defaultDLPModel()
{
    return {
        "134.4 x 75.6 mm",
        134.4f,
        75.6f
    };
}

//--------------------------------------------------
// OLCUM VERISI
//--------------------------------------------------

struct MeasurementData
{
    double widthMM = 0.0;
    double heightMM = 0.0;

    cv::Point2d centerMM = cv::Point2d(0.0, 0.0);

    double pitch = 0.0;
    double roll = 0.0;

    double diagonalMM = 0.0;
    double perspectiveErrorPct = 0.0;
    double rotationDeg = 0.0;

    cv::RotatedRect rect;
    std::vector<cv::Point2f> imageCorners;

    bool valid = false;
    bool poseValid = false;
    bool mmValid = false;
    bool sizeSuspicious = false;
};
//--------------------------------------------------
// OLCUM STABILIZASYONU
//--------------------------------------------------

class MeasurementHistory
{
public:
    explicit MeasurementHistory(size_t maxSize = 7, size_t minSamples = 3)
        : m_maxSize(maxSize), m_minSamples(minSamples) {}

    void push(const MeasurementData& m)
    {
        if(!m.valid)
        {
            m_buffer.clear();
            return;
        }

        m_buffer.push_back(m);

        if(m_buffer.size() > m_maxSize)
            m_buffer.pop_front();
    }

    bool ready() const
    {
        return m_buffer.size() >= m_minSamples;
    }

    bool empty() const
    {
        return m_buffer.empty();
    }

    MeasurementData median() const
    {
        MeasurementData result = m_buffer.back();

        result.widthMM  = medianOf([](const MeasurementData& m){ return m.widthMM; });
        result.heightMM = medianOf([](const MeasurementData& m){ return m.heightMM; });
        result.pitch = medianOf([](const MeasurementData& m){ return m.pitch; });
        result.roll = medianOf([](const MeasurementData& m){ return m.roll; });
        result.diagonalMM = medianOf([](const MeasurementData& m){ return m.diagonalMM; });
        result.perspectiveErrorPct =
            medianOf([](const MeasurementData& m){ return m.perspectiveErrorPct; });
        result.rotationDeg =
            medianOf([](const MeasurementData& m){ return m.rotationDeg; });

        double cx =
            medianOf([](const MeasurementData& m){ return m.centerMM.x; });

        double cy =
            medianOf([](const MeasurementData& m){ return m.centerMM.y; });

        result.centerMM = cv::Point2d(cx, cy);

        result.sizeSuspicious = m_buffer.back().sizeSuspicious;

        return result;
    }

private:
    template <typename Fn>
    double medianOf(Fn getField) const
    {
        std::vector<double> values;
        values.reserve(m_buffer.size());

        for(const auto& m : m_buffer)
            values.push_back(getField(m));

        std::sort(values.begin(), values.end());

        size_t n = values.size();

        return (n % 2 == 1)
            ? values[n / 2]
            : (values[n / 2 - 1] + values[n / 2]) / 2.0;
    }

    std::deque<MeasurementData> m_buffer;
    size_t m_maxSize;
    size_t m_minSamples;
};
//--------------------------------------------------
// YONLENDIRME DURUMU
//--------------------------------------------------

enum class AlignmentStatus
{
    OK,
    MOVE_LEFT,
    MOVE_RIGHT,
    MOVE_UP,
    MOVE_DOWN,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    ROTATE_CW,
    ROTATE_CCW,
    STABILIZING,
    SIZE_MISMATCH,
    NO_DATA
};

//--------------------------------------------------
// PROJEKSIYON TESPITI
//--------------------------------------------------

cv::Mat detectBrightRegion(
    const cv::Mat& gray,
    int thresholdValue = 180
);

cv::Mat cleanMask(
    const cv::Mat& mask
);

int findLargestContour(
    const cv::Mat& binary,
    std::vector<std::vector<cv::Point>>& contours,
    std::vector<cv::Vec4i>& hierarchy
);

//--------------------------------------------------
// OLCUM
//--------------------------------------------------

MeasurementData measureObject(
    const std::vector<std::vector<cv::Point>>& contours,
    int maxIndex,
    const cv::Mat& grayFrame,
    const cv::Mat& homographyMatrix,
    bool homographyLoaded,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    bool calibrationLoaded,
    const DLPModel& selectedModel
);

//--------------------------------------------------
// YONLENDIRME
//--------------------------------------------------

AlignmentStatus updateAlignmentStatus(
    const MeasurementData& measurement,
    const DLPModel& selectedModel
);

std::string alignmentStatusToTurkish(
    AlignmentStatus status
);

std::string formatAlignmentDetail(
    AlignmentStatus status,
    const MeasurementData& measurement,
    const DLPModel& selectedModel
);
//--------------------------------------------------
// ADAPTIF HSV PROJEKSIYON TESPITI
//--------------------------------------------------

cv::Mat detectColorRegionAdaptive(
    const cv::Mat& hsv,
    int hMin,
    int hMax,
    int sMin,
    int vMinFloor,
    int& outUsedVThresh
);
