#include "measurement_engine.hpp"

#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdio>

//==================================================
static const bool g_verboseLog = false;
// DLP PROJECTION MEASUREMENT ENGINE
//==================================================

cv::Mat detectBrightRegion(const cv::Mat& gray, int thresholdValue)
{
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

    cv::Mat mask;
    cv::threshold(blurred, mask, thresholdValue, 255, cv::THRESH_BINARY);

    return mask;
}

//--------------------------------------------------
// HSV Ayarlarini Kaydet / Yukle (YENI)
//--------------------------------------------------
// Amac: program her acildiginda trackbar'lari sifirdan ayarlama ihtiyacini
// ortadan kaldirmak. Bulunan degerler (manuel veya autoCalibrateHueBand /
// Otsu tarafindan onerilen) bir .yml dosyasina yazilir, bir sonraki
// acilista buradan okunur.

bool loadHsvSettings(
    const std::string& filename,
    int& hMin, int& hMax,
    int& sMin, int& sMax,
    int& vMin, int& vMax)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if(!fs.isOpened())
    {
        std::cout << "[BILGI] " << filename << " bulunamadi, varsayilan HSV degerleri kullanilacak." << std::endl;
        return false;
    }

    fs["h_min"] >> hMin;
    fs["h_max"] >> hMax;
    fs["s_min"] >> sMin;
    fs["s_max"] >> sMax;
    fs["v_min"] >> vMin;
    fs["v_max"] >> vMax;
    fs.release();

    std::cout << "HSV ayarlari yuklendi: H[" << hMin << "-" << hMax
              << "] S[" << sMin << "-" << sMax
              << "] V[" << vMin << "-" << vMax << "]" << std::endl;

    return true;
}

bool saveHsvSettings(
    const std::string& filename,
    int hMin, int hMax,
    int sMin, int sMax,
    int vMin, int vMax)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    if(!fs.isOpened())
    {
        std::cout << "[UYARI] " << filename << " olusturulamadi, HSV ayarlari kaydedilemedi." << std::endl;
        return false;
    }

    fs << "h_min" << hMin;
    fs << "h_max" << hMax;
    fs << "s_min" << sMin;
    fs << "s_max" << sMax;
    fs << "v_min" << vMin;
    fs << "v_max" << vMax;
    fs.release();

    std::cout << "HSV ayarlari kaydedildi: " << filename << std::endl;
    return true;
}

//--------------------------------------------------
// HSV Kalibrasyon Araci (manuel, 'i'/'o'/'r' ile)
//--------------------------------------------------

struct HsvCalibState
{
    cv::Mat hsvFrame;
    bool collectingInside = false;
    bool collectingOutside = false;
    std::vector<cv::Vec3b> insideSamples;
    std::vector<cv::Vec3b> outsideSamples;
};

void hsvCalibMouseCallback(int event, int x, int y, int, void* userdata)
{
    if(event != cv::EVENT_LBUTTONDOWN)
        return;

    HsvCalibState* state = static_cast<HsvCalibState*>(userdata);

    if(state->hsvFrame.empty() ||
       x < 0 || y < 0 || x >= state->hsvFrame.cols || y >= state->hsvFrame.rows)
        return;

    cv::Vec3b p = state->hsvFrame.at<cv::Vec3b>(y, x);

    if(state->collectingInside)
    {
        state->insideSamples.push_back(p);
        std::cout << "[ICERI  #" << state->insideSamples.size() << "] "
                  << "H=" << (int)p[0] << " S=" << (int)p[1] << " V=" << (int)p[2] << std::endl;
    }
    else if(state->collectingOutside)
    {
        state->outsideSamples.push_back(p);
        std::cout << "[DISARI #" << state->outsideSamples.size() << "] "
                  << "H=" << (int)p[0] << " S=" << (int)p[1] << " V=" << (int)p[2] << std::endl;
    }
    else
    {
        std::cout << "[BILGI] Once 'i' (icerisi) veya 'o' (disarisi) tusuyla "
                     "ornek toplama modunu acin." << std::endl;
    }
}

void printSuggestedHsvRange(const HsvCalibState& state)
{
    if(state.insideSamples.empty())
    {
        std::cout << "[UYARI] Hic 'ICERI' ornegi toplanmadi, oneri hesaplanamiyor." << std::endl;
        return;
    }

    int hMin = 255, hMax = 0, sMin = 255, sMax = 0, vMin = 255, vMax = 0;

    for(const auto& p : state.insideSamples)
    {
        hMin = std::min(hMin, (int)p[0]); hMax = std::max(hMax, (int)p[0]);
        sMin = std::min(sMin, (int)p[1]); sMax = std::max(sMax, (int)p[1]);
        vMin = std::min(vMin, (int)p[2]); vMax = std::max(vMax, (int)p[2]);
    }

    const int hMargin = 5;
    const int svMargin = 25;

    int lowerH = std::max(0,   hMin - hMargin);
    int upperH = std::min(179, hMax + hMargin);
    int lowerS = std::max(0,   sMin - svMargin);
    int upperS = std::min(255, sMax + svMargin);
    int lowerV = std::max(0,   vMin - svMargin);
    int upperV = std::min(255, vMax + svMargin);

    std::cout << "\n===== ONERILEN ARALIK (ICERI orneklerinden, " << state.insideSamples.size()
              << " nokta) =====\n";
    std::cout << "H: " << lowerH << " - " << upperH << std::endl;
    std::cout << "S: " << lowerS << " - " << upperS << std::endl;
    std::cout << "V: " << lowerV << " - " << upperV << std::endl;
    std::cout << "(Bu degerleri 'Mask (debug)' penceresindeki trackbar'lara girin)" << std::endl;

    if(!state.outsideSamples.empty())
    {
        int falsePositives = 0;

        for(const auto& p : state.outsideSamples)
        {
            bool inRange =
                p[0] >= lowerH && p[0] <= upperH &&
                p[1] >= lowerS && p[1] <= upperS &&
                p[2] >= lowerV && p[2] <= upperV;

            if(inRange) falsePositives++;
        }

        std::cout << falsePositives << " / " << state.outsideSamples.size()
                  << " DISARI ornegi bu aralikla YANLISLIKLA eslesiyor";

        if(falsePositives > 0)
            std::cout << " -> aralik hala COK GENIS, ICERI orneklerini patternin "
                         "kenarindan degil ORTASINA yakin noktalardan toplayip "
                         "tekrar deneyin.";

        std::cout << std::endl;
    }
    else
    {
        std::cout << "[BILGI] Karsilastirma icin 'o' ile birkac DISARI (arka plan) "
                     "ornegi de toplarsaniz oneri daha guvenilir olur." << std::endl;
    }

    std::cout << "=====================================\n" << std::endl;
}

cv::Mat cleanMask(const cv::Mat& mask)
{
    cv::Mat opened;
    cv::Mat closed;

    cv::Mat openKernel =
        cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(5,5));

    cv::Mat closeKernel =
        cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(7,7));

    cv::morphologyEx(
        mask,
        opened,
        cv::MORPH_OPEN,
        openKernel);

    cv::morphologyEx(
        opened,
        closed,
        cv::MORPH_CLOSE,
        closeKernel);

    return closed;
}

//--------------------------------------------------
// Find Largest Contour
//--------------------------------------------------

int findLargestContour(
    const cv::Mat& binary,
    std::vector<std::vector<cv::Point>>& contours,
    std::vector<cv::Vec4i>& hierarchy)
{
    cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxArea = 0.0;
    int maxIndex = -1;

    for(size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);

        if(area > 500 && area > maxArea)
        {
            maxArea = area;
            maxIndex = static_cast<int>(i);
        }
    }

    return maxIndex;
}

//--------------------------------------------------
// Order Corners
//--------------------------------------------------

std::vector<cv::Point2f> orderCorners(const std::vector<cv::Point2f>& pts)
{
    if(pts.size() != 4)
        return pts;

    std::vector<cv::Point2f> ordered(4);

    auto sumCmp = [](const cv::Point2f& a, const cv::Point2f& b)
    {
        return (a.x + a.y) < (b.x + b.y);
    };

    auto diffCmp = [](const cv::Point2f& a, const cv::Point2f& b)
    {
        return (a.x - a.y) < (b.x - b.y);
    };

    ordered[0] = *std::min_element(pts.begin(), pts.end(), sumCmp);   // sol üst
    ordered[2] = *std::max_element(pts.begin(), pts.end(), sumCmp);   // sağ alt
    ordered[1] = *std::max_element(pts.begin(), pts.end(), diffCmp);  // sağ üst
    ordered[3] = *std::min_element(pts.begin(), pts.end(), diffCmp);  // sol alt

    return ordered;
}

//--------------------------------------------------
// Refine Corners To Subpixel Accuracy
//--------------------------------------------------

std::vector<cv::Point2f> refineCornersSubPixel(
    const cv::Mat& grayFrame,
    const cv::Point2f rawCorners[4])
{
    std::vector<cv::Point2f> refined(rawCorners, rawCorners + 4);

    if(grayFrame.empty())
        return refined;

    for(const auto& p : refined)
    {
        if(p.x < 0 ||
           p.y < 0 ||
           p.x >= grayFrame.cols ||
           p.y >= grayFrame.rows)
        {
            return refined;
        }
    }

    cv::cornerSubPix(
        grayFrame,
        refined,
        cv::Size(5,5),
        cv::Size(-1,-1),
        cv::TermCriteria(
            cv::TermCriteria::EPS +
            cv::TermCriteria::COUNT,
            30,
            0.01));

    return refined;
}
//==================================================
// Line Intersection
//==================================================

bool lineIntersection(
    const cv::Vec4f& line1,
    const cv::Vec4f& line2,
    cv::Point2f& outPoint)
{
    float vx1 = line1[0];
    float vy1 = line1[1];
    float x1  = line1[2];
    float y1  = line1[3];

    float vx2 = line2[0];
    float vy2 = line2[1];
    float x2  = line2[2];
    float y2  = line2[3];

    float det = vx1 * vy2 - vy1 * vx2;

    if(std::abs(det) < 1e-6f)
        return false;

    float t =
        ((x2 - x1) * vy2 -
         (y2 - y1) * vx2) / det;

    outPoint.x = x1 + t * vx1;
    outPoint.y = y1 + t * vy1;

    return true;
}
//==================================================
// Fit Line From Points
//==================================================
// NOT: DIST_HUBER denenmisti ama gercek olcumlerde (hem width hem height
// buyudu, %0.07/%0.71 -> %0.77/%2.8) DIST_L2'den daha KOTU sonuc verdi -
// bu yuzden klasik DIST_L2'ye (en kucuk kareler) geri donuldu. Kisa/uzun
// kenar asimetrisi icin asil cozum, mask kalitesini (Otsu tabani, S/V
// ayari) iyilestirmek oldu - bkz. detectColorRegionAdaptive() ve vMinFloor.

bool fitLineFromPoints(
    const std::vector<cv::Point2f>& points,
    cv::Vec4f& line)
{
    if(points.size() < 2)
        return false;

    cv::fitLine(
        points,
        line,
        cv::DIST_L2,
        0,
        0.01,
        0.01);

    return true;
}
//--------------------------------------------------
// Split Contour Into 4 Sides (RotatedRect Axis)
//--------------------------------------------------

void splitContourSides(
    const std::vector<cv::Point>& contour,
    const cv::RotatedRect& rect,
    std::vector<cv::Point2f>& top,
    std::vector<cv::Point2f>& right,
    std::vector<cv::Point2f>& bottom,
    std::vector<cv::Point2f>& left)
{
    top.clear();
    right.clear();
    bottom.clear();
    left.clear();

    float angle = rect.angle;

    if(rect.size.width < rect.size.height)
        angle += 90.0f;

    float theta = angle * CV_PI / 180.0f;

    cv::Point2f ux(std::cos(theta), std::sin(theta));
    cv::Point2f uy(-std::sin(theta), std::cos(theta));

    cv::Point2f c = rect.center;

    float halfW = rect.size.width  * 0.5f;
    float halfH = rect.size.height * 0.5f;

    const float margin = 8.0f;

    for(const auto& p : contour)
    {
        cv::Point2f v = cv::Point2f(p) - c;

        float x = v.dot(ux);
        float y = v.dot(uy);

        if(std::abs(y + halfH) < margin)
            top.push_back(p);

        if(std::abs(y - halfH) < margin)
            bottom.push_back(p);

        if(std::abs(x + halfW) < margin)
            left.push_back(p);

        if(std::abs(x - halfW) < margin)
            right.push_back(p);
    }
}
//==================================================
// Extract Precise Corners
//==================================================

bool extractPreciseCorners(
    const std::vector<cv::Point>& contour,
    const cv::RotatedRect& rect,
    std::vector<cv::Point2f>& corners)
{
    corners.clear();

    if(contour.size() < 20)
        return false;

    std::vector<cv::Point2f> top;
    std::vector<cv::Point2f> right;
    std::vector<cv::Point2f> bottom;
    std::vector<cv::Point2f> left;

    splitContourSides(
        contour,
        rect,
        top,
        right,
        bottom,
        left);

    cv::Vec4f topLine;
    cv::Vec4f rightLine;
    cv::Vec4f bottomLine;
    cv::Vec4f leftLine;

    if(!fitLineFromPoints(top, topLine))
        return false;

    if(!fitLineFromPoints(right, rightLine))
        return false;

    if(!fitLineFromPoints(bottom, bottomLine))
        return false;

    if(!fitLineFromPoints(left, leftLine))
        return false;

    cv::Point2f p0;
    cv::Point2f p1;
    cv::Point2f p2;
    cv::Point2f p3;

    if(!lineIntersection(topLine, leftLine, p0))
        return false;

    if(!lineIntersection(topLine, rightLine, p1))
        return false;

    if(!lineIntersection(bottomLine, rightLine, p2))
        return false;

    if(!lineIntersection(bottomLine, leftLine, p3))
        return false;

    corners = {p0, p1, p2, p3};

    corners = orderCorners(corners);

    return true;
}


//--------------------------------------------------
// Is Rectangular Contour
//--------------------------------------------------

bool isRectangularContour(
    const std::vector<cv::Point>& contour,
    double contourArea,
    const cv::RotatedRect& rotated,
    double minFillRatio = 0.85)
{
    double rectArea = static_cast<double>(rotated.size.width) * rotated.size.height;

    if(rectArea < 1.0)
        return false;

    double fillRatio = contourArea / rectArea;

    if(fillRatio < minFillRatio)
        return false;

    double peri = cv::arcLength(contour, true);
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, 0.03 * peri, true);

    if(approx.size() < 4 || approx.size() > 6)
        return false;

    return true;
}

//--------------------------------------------------
// Estimate Pose
//--------------------------------------------------

bool estimatePose(
    const std::vector<cv::Point2f>& orderedImageCorners,
    float realWidth,
    float realHeight,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    double& pitchDeg,
    double& rollDeg)
{
    if(cameraMatrix.empty() || distCoeffs.empty())
        return false;

    if(orderedImageCorners.size() != 4)
        return false;

    std::vector<cv::Point3f> objectPoints = {
        cv::Point3f(0.0f, 0.0f, 0.0f),
        cv::Point3f(realWidth, 0.0f, 0.0f),
        cv::Point3f(realWidth, realHeight, 0.0f),
        cv::Point3f(0.0f, realHeight, 0.0f)
    };

    cv::Mat rvec, tvec;

    bool ok = cv::solvePnP(
        objectPoints,
        orderedImageCorners,
        cameraMatrix,
        distCoeffs,
        rvec,
        tvec);

    if(!ok)
        return false;

    cv::Mat R;
    cv::Rodrigues(rvec, R);

    double sy = std::sqrt(
        R.at<double>(0,0) * R.at<double>(0,0) +
        R.at<double>(1,0) * R.at<double>(1,0));

    bool singular = sy < 1e-6;

    double xRot, yRot;

    if(!singular)
    {
        xRot = std::atan2(R.at<double>(2,1), R.at<double>(2,2));
        yRot = std::atan2(-R.at<double>(2,0), sy);
    }
    else
    {
        xRot = std::atan2(-R.at<double>(1,2), R.at<double>(1,1));
        yRot = std::atan2(-R.at<double>(2,0), sy);
    }

    rollDeg  = xRot * 180.0 / CV_PI;
    pitchDeg = yRot * 180.0 / CV_PI;

    return true;
}

//--------------------------------------------------
// Measure Object
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
    const DLPModel& selectedModel)
{
    MeasurementData measurement;

    measurement.valid     = false;
    measurement.poseValid = false;
    measurement.mmValid   = false;
    measurement.sizeSuspicious = false;

    measurement.widthMM  = 0.0;
    measurement.heightMM = 0.0;

    measurement.centerMM = cv::Point2d(0.0, 0.0);

    measurement.pitch = 0.0;
    measurement.roll  = 0.0;

    measurement.diagonalMM          = 0.0;
    measurement.perspectiveErrorPct = 0.0;
    measurement.rotationDeg         = 0.0;

    if(maxIndex == -1)
        return measurement;

    static int s_warnThrottleFrames = 0;
    const int WARN_THROTTLE_INTERVAL = 20;

    auto printThrottled = [&](const std::string& msg)
    {
        if(s_warnThrottleFrames == 0)
        {
            std::cout << "\n[UYARI] " << msg << std::endl;
            s_warnThrottleFrames = WARN_THROTTLE_INTERVAL;
        }
        else
        {
            s_warnThrottleFrames--;
        }
    };

    cv::RotatedRect rotated = cv::minAreaRect(contours[maxIndex]);

    double area = cv::contourArea(contours[maxIndex]);

    if(!grayFrame.empty())
    {
        cv::Rect boundingBox = cv::boundingRect(contours[maxIndex]);

        double frameArea = static_cast<double>(grayFrame.cols) * grayFrame.rows;
        double boxArea    = static_cast<double>(boundingBox.width) * boundingBox.height;

        bool touchesAllEdges =
            boundingBox.x <= 1 &&
            boundingBox.y <= 1 &&
            (boundingBox.x + boundingBox.width)  >= (grayFrame.cols - 1) &&
            (boundingBox.y + boundingBox.height) >= (grayFrame.rows - 1);

        bool coversFrame = (frameArea > 0.0) && ((boxArea / frameArea) > 0.90);

        if(coversFrame || touchesAllEdges)
        {
            printThrottled(
                "Tespit edilen parlak bolge kameranin goruntu cercevesinin neredeyse "
                "TAMAMINI kapliyor - bu gercek bir DLP deseni degil, asiri pozlama / "
                "cok dusuk threshold / fazla ortam isigi belirtisidir. Olcum atlandi. "
                "'Mask (debug)' penceresindeki Threshold kaydiricisini YUKSELTIN veya "
                "ortam isigini azaltin.");
            return measurement;
        }
    }

    if(!isRectangularContour(contours[maxIndex], area, rotated))
    {
        printThrottled("Bulunan kontur dikdortgen degil, olcum atlandi.");
        return measurement;
    }

    s_warnThrottleFrames = 0;

    measurement.rect  = rotated;
    measurement.valid = true;

    std::vector<cv::Point2f> imageCorners;

    if(!extractPreciseCorners(
            contours[maxIndex],
            rotated,
            imageCorners))
    {
        return measurement;
    }

    measurement.imageCorners = imageCorners;
    std::vector<cv::Point2f> worldPoints;

    cv::Point2f centerPxF = rotated.center;

    double width  = rotated.size.width;
    double height = rotated.size.height;

    double angle = rotated.angle;

    if(g_verboseLog)
    {
        std::cout << "\n=====================\n";
        std::cout << "Area : " << area << std::endl;
        std::cout << "Width (pixel) : " << width << std::endl;
        std::cout << "Height (pixel) : " << height << std::endl;
    }

    {
        cv::Point2f topEdge = imageCorners[1] - imageCorners[0];
        double rotationRad = std::atan2(topEdge.y, topEdge.x);
        measurement.rotationDeg = rotationRad * 180.0 / CV_PI;
    }

    if(homographyLoaded && !homographyMatrix.empty())
    {
        cv::perspectiveTransform(imageCorners, worldPoints, homographyMatrix);

        if(worldPoints.size() == 4)
        {
            // Projeksiyonun donusunu kamera goruntusune gore degil,
// cihaz/homography koordinat sistemine gore hesapla.
cv::Point2f topEdgeWorld = worldPoints[1] - worldPoints[0];

double rotationRadWorld =
    std::atan2(topEdgeWorld.y, topEdgeWorld.x);

measurement.rotationDeg =
    rotationRadWorld * 180.0 / CV_PI;

            double d01 = cv::norm(worldPoints[0] - worldPoints[1]);
            double d12 = cv::norm(worldPoints[1] - worldPoints[2]);
            double d23 = cv::norm(worldPoints[2] - worldPoints[3]);
            double d30 = cv::norm(worldPoints[3] - worldPoints[0]);

            double sideA = (d01 + d23) / 2.0;
            double sideB = (d12 + d30) / 2.0;

            measurement.widthMM  = std::max(sideA, sideB);
            measurement.heightMM = std::min(sideA, sideB);

            double diag02 = cv::norm(worldPoints[0] - worldPoints[2]);
            double diag13 = cv::norm(worldPoints[1] - worldPoints[3]);
            measurement.diagonalMM = (diag02 + diag13) / 2.0;

            double horizKeystonePct = (sideA > 1e-6)
                ? (std::abs(d01 - d23) / sideA) * 100.0
                : 0.0;

            double vertKeystonePct = (sideB > 1e-6)
                ? (std::abs(d12 - d30) / sideB) * 100.0
                : 0.0;

            measurement.perspectiveErrorPct = std::max(horizKeystonePct, vertKeystonePct);
        }

        std::vector<cv::Point2f> centerPx = { centerPxF };
        std::vector<cv::Point2f> centerWorld;
        cv::perspectiveTransform(centerPx, centerWorld, homographyMatrix);

        if(!centerWorld.empty())
        {
            measurement.centerMM = cv::Point2d(centerWorld[0].x, centerWorld[0].y);
            measurement.mmValid  = true;
        }

        if(measurement.mmValid && selectedModel.widthMM > 1e-3f && selectedModel.heightMM > 1e-3f)
        {
            double wDiffPct = std::abs(measurement.widthMM  - selectedModel.widthMM)  / selectedModel.widthMM  * 100.0;
            double hDiffPct = std::abs(measurement.heightMM - selectedModel.heightMM) / selectedModel.heightMM * 100.0;

            if(wDiffPct > SIZE_SUSPICIOUS_PCT || hDiffPct > SIZE_SUSPICIOUS_PCT)
            {
                measurement.sizeSuspicious = true;

                printThrottled(
                    "Olculen boyut (" + std::to_string(measurement.widthMM) + " x " +
                    std::to_string(measurement.heightMM) + " mm), secilen DLP modelinin "
                    "nominal boyutundan (" + selectedModel.name + ") %35'ten fazla sapiyor. "
                    "Olasi nedenler: (1) Homography programinda A4 kagidi, DLP olcum "
                    "yuzeyiyle AYNI fiziksel duzlemde degildi, (2) yanlis parlak bolge "
                    "tespit edildi. Homografiyi DLP yuzeyinin TAM UZERINDE yeniden "
                    "olusturun (./Homography).");
            }
        }

        if(g_verboseLog)
        {
            std::cout << "Width (mm) : "  << measurement.widthMM  << std::endl;
            std::cout << "Height (mm) : " << measurement.heightMM << std::endl;
            std::cout << "Center (mm) : (" << measurement.centerMM.x << ", "
                       << measurement.centerMM.y << ")" << std::endl;
            std::cout << "Diagonal (mm) : " << measurement.diagonalMM << std::endl;
            std::cout << "Perspective error (%) : " << measurement.perspectiveErrorPct << std::endl;
            std::cout << "Rotation (deg) : " << measurement.rotationDeg << std::endl;
        }
    }
    else
    {
        std::cout << "[UYARI] Homografi yuklenmedi, mm degerleri hesaplanamadi." << std::endl;
    }

    if(g_verboseLog)
        std::cout << "Angle (minAreaRect) : " << angle << std::endl;

    if(calibrationLoaded)
    {
        double pitchDeg = 0.0;
        double rollDeg  = 0.0;

        float poseWidth  = selectedModel.widthMM;
        float poseHeight = selectedModel.heightMM;

        bool poseOk = estimatePose(
            imageCorners,
            poseWidth,
            poseHeight,
            cameraMatrix,
            distCoeffs,
            pitchDeg,
            rollDeg);

        if(poseOk)
        {
            measurement.pitch     = pitchDeg;
            measurement.roll      = rollDeg;
            measurement.poseValid = true;

            if(g_verboseLog)
            {
                std::cout << "Pitch : " << pitchDeg << " derece" << std::endl;
                std::cout << "Roll  : " << rollDeg  << " derece" << std::endl;
            }
        }
        else
        {
            std::cout << "[UYARI] Pose (pitch/roll) hesaplanamadi." << std::endl;
        }
    }
    else
    {
        std::cout << "[UYARI] Kalibrasyon yok, pitch/roll hesaplanamiyor." << std::endl;
    }

    return measurement;
}

//--------------------------------------------------
// Update Alignment Status
//--------------------------------------------------

AlignmentStatus updateAlignmentStatus(
    const MeasurementData& measurement,
    const DLPModel& selectedModel)
{
    if(!measurement.valid || !measurement.mmValid || !measurement.poseValid)
        return AlignmentStatus::NO_DATA;

    if(measurement.sizeSuspicious)
        return AlignmentStatus::SIZE_MISMATCH;

   // 4 referans vidanin olusturdugu fiziksel alan
constexpr double SCREW_FRAME_WIDTH_MM  = 222.5;
constexpr double SCREW_FRAME_HEIGHT_MM = 150.0;

// Gercek hedef: 4 vidanin geometrik merkezi
const double targetCenterX = SCREW_FRAME_WIDTH_MM  / 2.0; // 111.25 mm
const double targetCenterY = SCREW_FRAME_HEIGHT_MM / 2.0; // 75.00 mm

// Projeksiyon merkezinin hedef merkezden sapmasi
const double dx = measurement.centerMM.x - targetCenterX;
const double dy = measurement.centerMM.y - targetCenterY;

const bool xOutside = std::abs(dx) > CENTER_TOLERANCE_MM;
const bool yOutside = std::abs(dy) > CENTER_TOLERANCE_MM;

// Iki eksen de tolerans disindaysa,
// once daha buyuk sapmayi duzelt
if(xOutside || yOutside)
{
    if(xOutside && (!yOutside || std::abs(dx) >= std::abs(dy)))
    {
        if(dx > 0.0)
            return AlignmentStatus::MOVE_LEFT;
        else
            return AlignmentStatus::MOVE_RIGHT;
    }
    else
    {
        if(dy > 0.0)
            return AlignmentStatus::MOVE_UP;
        else
            return AlignmentStatus::MOVE_DOWN;
    }
}

    double nominalAvg  = (selectedModel.widthMM + selectedModel.heightMM) / 2.0;
    double measuredAvg = (measurement.widthMM   + measurement.heightMM)   / 2.0;
    double sizeDiffPct = (nominalAvg > 1e-6)
        ? ((measuredAvg - nominalAvg) / nominalAvg) * 100.0
        : 0.0;

    if(sizeDiffPct < -SIZE_TOLERANCE_PCT)
    {
        return AlignmentStatus::MOVE_FORWARD;
    }
    else if(sizeDiffPct > SIZE_TOLERANCE_PCT)
    {
        return AlignmentStatus::MOVE_BACKWARD;
    }
    else if(measurement.rotationDeg > ANGLE_TOLERANCE_DEG)
{
    return AlignmentStatus::ROTATE_CCW;
}
else if(measurement.rotationDeg < -ANGLE_TOLERANCE_DEG)
{
    return AlignmentStatus::ROTATE_CW;
}

    return AlignmentStatus::OK;
}

//--------------------------------------------------
// Alignment Detail Text
//--------------------------------------------------

std::string formatAlignmentDetail(
    AlignmentStatus status,
    const MeasurementData& measurement,
    const DLPModel& selectedModel)
{
    char buf[128];

// Vida referans koordinat sisteminin fiziksel merkezi
// TL=(0,0), TR=(222.5,0), BR=(222.5,150), BL=(0,150)

constexpr double SCREW_FRAME_WIDTH_MM  = 222.5;
constexpr double SCREW_FRAME_HEIGHT_MM = 150.0;

double targetCenterX = SCREW_FRAME_WIDTH_MM  / 2.0;  // 111.25 mm
double targetCenterY = SCREW_FRAME_HEIGHT_MM / 2.0;  // 75.00 mm

    double dx = measurement.centerMM.x - targetCenterX;
    double dy = measurement.centerMM.y - targetCenterY;

    double nominalAvg  = (selectedModel.widthMM + selectedModel.heightMM) / 2.0;
    double measuredAvg = (measurement.widthMM   + measurement.heightMM)   / 2.0;
    double sizeDiffPct = (nominalAvg > 1e-6)
        ? ((measuredAvg - nominalAvg) / nominalAvg) * 100.0
        : 0.0;

    switch(status)
    {
        case AlignmentStatus::MOVE_LEFT:
        case AlignmentStatus::MOVE_RIGHT:
        case AlignmentStatus::MOVE_UP:
        case AlignmentStatus::MOVE_DOWN:
            snprintf(
                buf,
                sizeof(buf),
                "X:%+.1f mm  Y:%+.1f mm",
                dx,
                dy
            );
            return std::string(buf);

    case AlignmentStatus::MOVE_FORWARD:
    case AlignmentStatus::MOVE_BACKWARD:
        snprintf(buf, sizeof(buf), "Distance Offset : %+.1f %%", sizeDiffPct);
        return std::string(buf);

    case AlignmentStatus::ROTATE_CW:
    case AlignmentStatus::ROTATE_CCW:
    snprintf(buf, sizeof(buf), "Angle Offset : %+.1f deg", measurement.rotationDeg);
        return std::string(buf);

    case AlignmentStatus::SIZE_MISMATCH:
        snprintf(buf, sizeof(buf), "Beklenen: %.1fx%.1f mm, Olculen: %.1fx%.1f mm",
                 selectedModel.widthMM, selectedModel.heightMM,
                 measurement.widthMM, measurement.heightMM);
        return std::string(buf);

    case AlignmentStatus::OK:
        return "Tum eksenler tolerans icinde";

    default:
        return "";
    }
}

//--------------------------------------------------
// Measurement History
//--------------------------------------------------

//==================================================
// ALIGNMENT STATUS -> TURKCE YONLENDIRME
//==================================================

std::string alignmentStatusToTurkish(AlignmentStatus status)
{
    switch(status)
    {
        case AlignmentStatus::OK:
            return "HIZALAMA TAMAM";

        case AlignmentStatus::MOVE_LEFT:
            return "PROJEKSIYONU SOLA HAREKET ETTIR";

        case AlignmentStatus::MOVE_RIGHT:
            return "PROJEKSIYONU SAGA HAREKET ETTIR";

        case AlignmentStatus::MOVE_UP:
            return "PROJEKSIYONU YUKARI HAREKET ETTIR";

        case AlignmentStatus::MOVE_DOWN:
            return "PROJEKSIYONU ASAGI HAREKET ETTIR";

        case AlignmentStatus::MOVE_FORWARD:
            return "PROJEKSIYONU ILERI HAREKET ETTIR";

        case AlignmentStatus::MOVE_BACKWARD:
            return "PROJEKSIYONU GERI HAREKET ETTIR";

        case AlignmentStatus::ROTATE_CW:
            return "PROJEKSIYONU SAAT YONUNDE DONDUR";

        case AlignmentStatus::ROTATE_CCW:
            return "PROJEKSIYONU SAAT YONUNUN TERSINE DONDUR";

        case AlignmentStatus::STABILIZING:
            return "OLCUM SABITLENIYOR...";

        case AlignmentStatus::SIZE_MISMATCH:
            return "PROJEKSIYON BOYUTU UYUSMUYOR";

        case AlignmentStatus::NO_DATA:
        default:
            return "PROJEKSIYON VERISI BEKLENIYOR";
    }
}
cv::Mat detectColorRegionAdaptive(
    const cv::Mat& hsv,
    int hMin, int hMax,
    int sMin,
    int vMinFloor,          // YENI: elle dogrulanmis guvenli minimum V - Otsu bunun ALTINA dusemez
    int& outUsedVThresh)
{
    cv::Mat blurred;
    cv::GaussianBlur(hsv, blurred, cv::Size(5, 5), 0);

    std::vector<cv::Mat> channels;
    cv::split(blurred, channels);
    const cv::Mat& hueChan = channels[0];
    const cv::Mat& satChan = channels[1];
    const cv::Mat& valChan = channels[2];

    cv::Mat hueMask;
    cv::inRange(hueChan, hMin, hMax, hueMask);

    cv::Mat satMask;
    cv::threshold(satChan, satMask, sMin, 255, cv::THRESH_BINARY);

    // FIX (regresyon duzeltmesi): Otsu istatistiksel olarak "en iyi ikiye
    // bolen" esigi bulur, ama bu her zaman "en dogru olcumu veren" esik
    // degildir. Kullanicinin elle deneme-hatayla bulup dogruladigi bir
    // minimum V degeri varsa (g_vMin trackbar'i / kaydedilmis hsv_settings.yml),
    // Otsu bu degerin ALTINA asla dusmesin - sadece bu minimumun UZERINDE
    // ince ayar yapsin. Boylece Otsu, bilinen iyi bir noktadan daha
    // "gevsek" (halo'yu da iceren) bir esik secmez.
    cv::Mat valMask;
    double otsuThresh = cv::threshold(valChan, valMask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    double effectiveThresh = std::max(otsuThresh, static_cast<double>(vMinFloor));
    cv::threshold(valChan, valMask, effectiveThresh, 255, cv::THRESH_BINARY);

    outUsedVThresh = static_cast<int>(std::round(effectiveThresh));

    cv::Mat mask;
    cv::bitwise_and(hueMask, satMask, mask);
    cv::bitwise_and(mask, valMask, mask);

    return mask;
}
