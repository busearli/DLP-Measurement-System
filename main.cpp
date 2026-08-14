#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <vector>
#include <deque>
#include <algorithm>
#include <limits>
#include <cmath>
#include <numeric>
//--------------------------------------------------
// AUTOMATIC SCREW REFERENCE SETTINGS
//--------------------------------------------------

const float SCREW_WIDTH_MM  = 222.5f;
const float SCREW_HEIGHT_MM = 150.0f;

const double MAX_ACCEPTABLE_REPROJ_MM = 0.3;

const int STABILITY_FRAMES = 12;
const double STABILITY_TOL_PX = 2.0;

const int MAX_MISSED_FRAMES = 5;

const char* SCREW_NAMES[4] = {
    "TL",
    "TR",
    "BR",
    "BL"
};
//--------------------------------------------------
// Debug / Verbose Logging
//--------------------------------------------------

bool g_verboseLog = false;

//--------------------------------------------------
// Timestamp Utility
//--------------------------------------------------

std::string nowTimestamp(const char* fmt)
{
    std::time_t t = std::time(nullptr);
    std::tm tmBuf{};

#if defined(_WIN32)
    localtime_s(&tmBuf, &t);
#else
    localtime_r(&t, &tmBuf);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tmBuf, fmt);
    return oss.str();
}

//--------------------------------------------------
// Measurement Data
//--------------------------------------------------

struct MeasurementData
{
    double widthMM;
    double heightMM;

    cv::Point2d centerMM;

    double pitch;
    double roll;

    double diagonalMM;
    double perspectiveErrorPct;
    double rotationDeg;

    cv::RotatedRect rect;

    bool valid;

    bool poseValid;
    bool mmValid;

    bool sizeSuspicious;
};

//--------------------------------------------------
// DLP Model Listesi
//--------------------------------------------------

struct DLPModel
{
    std::string name;
    float widthMM;
    float heightMM;
};

std::vector<DLPModel> g_dlpModels = {
    {"57.6 x 32.4 mm", 57.6f, 32.4f},
    {"96 x 54 mm",     96.0f, 54.0f},
    {"124.8 x 70.2 mm",124.8f, 70.2f},
    {"134.4 x 75.6 mm",134.4f, 75.6f},
    {"149.8 x 84.2 mm",149.8f, 84.2f},
    {"192 x 108 mm",   192.0f, 108.0f}
};

DLPModel selectDLPModel()
{
    std::cout << "\n=== DLP Modeli Secin ===" << std::endl;

    for(size_t i = 0; i < g_dlpModels.size(); i++)
    {
        std::cout << i + 1 << ") " << g_dlpModels[i].name << std::endl;
    }

    int choice = 0;
    std::cout << "Secim (1-" << g_dlpModels.size() << "): ";
    std::cin >> choice;

    if(std::cin.fail() || choice < 1 || choice > static_cast<int>(g_dlpModels.size()))
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Gecersiz secim, varsayilan olarak ilk model kullanilacak." << std::endl;
        choice = 1;
    }

    return g_dlpModels[choice - 1];
}

//--------------------------------------------------
// Test Deseni Secimi
//--------------------------------------------------

enum class TestPatternType
{
    WHITE_RECTANGLE,
    FOUR_CORNER_MARKERS,
    CROSS_LINES,
    CALIBRATION_GRID
};

TestPatternType selectTestPattern()
{
    std::cout << "\n=== Test Deseni Secin ===" << std::endl;
    std::cout << "1) Tam beyaz dikdortgen (uygulanmis)" << std::endl;
    std::cout << "2) Dort kose markeri (henuz uygulanmadi)" << std::endl;
    std::cout << "3) Capraz cizgiler (henuz uygulanmadi)" << std::endl;
    std::cout << "4) Kalibrasyon gridi (henuz uygulanmadi)" << std::endl;

    int choice = 0;
    std::cout << "Secim (1-4): ";
    std::cin >> choice;

    if(std::cin.fail())
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        choice = 1;
    }

    switch(choice)
    {
    case 2: return TestPatternType::FOUR_CORNER_MARKERS;
    case 3: return TestPatternType::CROSS_LINES;
    case 4: return TestPatternType::CALIBRATION_GRID;
    default: return TestPatternType::WHITE_RECTANGLE;
    }
}

//--------------------------------------------------
// Alignment Tolerances
//--------------------------------------------------

const double CENTER_TOLERANCE_MM = 2.0;
const double ANGLE_TOLERANCE_DEG = 1.0;
const double SIZE_TOLERANCE_PCT  = 3.0;

// FIX: olcum, secili DLP modelinin nominal boyutundan bu oranin
// UZERINDE sapiyorsa artik "supheli/guvenilmez" sayilir (bkz.
// asagidaki SIZE_SUSPICIOUS_PCT ve measureObject icindeki kontrol).
const double SIZE_SUSPICIOUS_PCT = 35.0;

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
    NO_DATA,
    STABILIZING,
    SIZE_MISMATCH
};

AlignmentStatus g_status = AlignmentStatus::NO_DATA;

//--------------------------------------------------
// Reference Plane (A4)
//--------------------------------------------------
// NOT: Bu iki sabit main.cpp icinde artik DOGRUDAN hesaplamada kullanilmiyor
// (bkz. updateAlignmentStatus() duzeltmesi) - burada sadece dunya (mm)
// koordinat sisteminin ORIJININI belgelemek icin tutuluyor: (0,0) noktasi,
// Homography.cpp'de tiklanan A4 kagidinin SOL-UST kosesidir. Gercek
// homografi hesabi homography.cpp icinde A4_WIDTH/A4_HEIGHT ile yapilir.

const float A4_WIDTH  = 210.0f;
const float A4_HEIGHT = 297.0f;

//--------------------------------------------------
// Alignment Status -> Metin / Renk
//--------------------------------------------------

std::string alignmentStatusToString(AlignmentStatus status)
{
    switch(status)
    {
    case AlignmentStatus::OK:            return "ALIGNMENT OK";
    case AlignmentStatus::MOVE_LEFT:     return "MOVE PROJECTOR LEFT";
    case AlignmentStatus::MOVE_RIGHT:    return "MOVE PROJECTOR RIGHT";
    case AlignmentStatus::MOVE_UP:       return "MOVE PROJECTOR UP";
    case AlignmentStatus::MOVE_DOWN:     return "MOVE PROJECTOR DOWN";
    case AlignmentStatus::MOVE_FORWARD:  return "MOVE PROJECTOR FORWARD";
    case AlignmentStatus::MOVE_BACKWARD: return "MOVE PROJECTOR BACKWARD";
    case AlignmentStatus::ROTATE_CW:     return "ROTATE CW";
    case AlignmentStatus::ROTATE_CCW:    return "ROTATE CCW";
    case AlignmentStatus::STABILIZING:   return "STABILIZING...";
    case AlignmentStatus::SIZE_MISMATCH: return "SIZE MISMATCH - CHECK SETUP";
    case AlignmentStatus::NO_DATA:
    default:                             return "NO DATA";
    }
}

cv::Scalar alignmentStatusToColor(AlignmentStatus status)
{
    switch(status)
    {
    case AlignmentStatus::OK:            return cv::Scalar(0,255,0);
    case AlignmentStatus::MOVE_LEFT:
    case AlignmentStatus::MOVE_RIGHT:
    case AlignmentStatus::MOVE_UP:
    case AlignmentStatus::MOVE_DOWN:     return cv::Scalar(0,255,255);
    case AlignmentStatus::MOVE_FORWARD:
    case AlignmentStatus::MOVE_BACKWARD: return cv::Scalar(255,0,255);
    case AlignmentStatus::ROTATE_CW:
    case AlignmentStatus::ROTATE_CCW:    return cv::Scalar(0,165,255);
    case AlignmentStatus::STABILIZING:   return cv::Scalar(200,200,0);
    case AlignmentStatus::SIZE_MISMATCH: return cv::Scalar(0,0,255);
    case AlignmentStatus::NO_DATA:
    default:                             return cv::Scalar(128,128,128);
    }
}

//--------------------------------------------------
// Capture Frame
//--------------------------------------------------

cv::Mat captureFrame(cv::VideoCapture& cap)
{
    cv::Mat frame;
    cap >> frame;
    return frame;
}

//--------------------------------------------------
// Detect Blue/Color Region (HSV) - Saha testinde eklendi
//--------------------------------------------------

int g_hMin = 100, g_hMax = 140;
int g_sMin = 40,  g_sMax = 255;
int g_vMin = 60,  g_vMax = 255;

// YENI: Otsu tabanli otomatik V esigi kullanildiginda, hangi esigin secildigini
// ekranda gostermek/loglamak icin. Trackbar'daki g_vMin/g_vMax adaptif moddayken
// KULLANILMAZ (sadece Sabit HSV modunda anlamlidir), ama arayuzde gormek icin
// tutuluyor.
int g_lastOtsuVThresh = 0;

enum class DetectionMode
{
    BRIGHTNESS_THRESHOLD,     // eski yontem: tam beyaz/parlak dikdortgen
    HSV_COLOR_RANGE,          // sabit HSV araligi (trackbar'lardan)
    HSV_ADAPTIVE              // YENI: Hue sabit/manuel, V esigi HER KAREDE Otsu ile otomatik
};

// Varsayilan artik adaptif mod: ortam isigi degistiginde (gunduz/gece,
// farkli aydinlatma) elle trackbar oynatma ihtiyacini azaltir.
DetectionMode g_detectionMode = DetectionMode::HSV_ADAPTIVE;

cv::Mat detectColorRegion(const cv::Mat& hsv, int hMin, int hMax, int sMin, int sMax, int vMin, int vMax)
{
    cv::Mat blurred;
    cv::GaussianBlur(hsv, blurred, cv::Size(5, 5), 0);

    cv::Mat mask;
    cv::inRange(blurred, cv::Scalar(hMin, sMin, vMin), cv::Scalar(hMax, sMax, vMax), mask);
    return mask;
}

// YENI: Adaptif tespit - Hue bandi (manuel/otomatik kalibre edilmis) sabit
// kabul edilir, ama V (parlaklik) esigi HER KAREDE o karenin kendi
// histogramindan Otsu yontemiyle otomatik hesaplanir. Bu, ortam isigi
// degistikce (gunduz/gece, farkli lamba) trackbar'a dokunmadan uyum
// saglanmasini saglar - cunku Otsu, "koyu arka plan + parlak desen" gibi
// iki-tepeli (bimodal) bir dagilimda esigi otomatik ortaya bulur.
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

// YENI: Program acilisinda (veya istendiginde 'a' tusuyla) sahnedeki en
// parlak bolgenin (Otsu esiginin ustunde kalan pikseller) Hue degerlerini
// orneklyip otomatik bir Hue araligi onerir. Boylece H trackbar'ini elle
// ayarlama ihtiyaci da buyuk olcude ortadan kalkar - sadece isik/pattern
// ekranda goruntudeyken cagirmak yeterlidir.
bool autoCalibrateHueBand(const cv::Mat& hsvFrame, int& outHMin, int& outHMax, int marginDeg = 6)
{
    std::vector<cv::Mat> ch;
    cv::split(hsvFrame, ch);

    cv::Mat brightMask;
    cv::threshold(ch[2], brightMask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    std::vector<uchar> hueSamples;
    hueSamples.reserve(5000);

    for(int y = 0; y < hsvFrame.rows; y += 3)
    {
        for(int x = 0; x < hsvFrame.cols; x += 3)
        {
            if(brightMask.at<uchar>(y, x) > 0)
                hueSamples.push_back(ch[0].at<uchar>(y, x));
        }
    }

    if(hueSamples.size() < 50)
    {
        std::cout << "[UYARI] Otomatik Hue kalibrasyonu icin yeterli parlak piksel bulunamadi "
                     "(" << hueSamples.size() << " ornek). Isik/pattern ekranda gorunur oldugundan "
                     "emin olun ve tekrar deneyin." << std::endl;
        return false;
    }

    std::sort(hueSamples.begin(), hueSamples.end());
    size_t n = hueSamples.size();

    int p10 = hueSamples[static_cast<size_t>(n * 0.10)];
    int p90 = hueSamples[static_cast<size_t>(n * 0.90)];

    outHMin = std::max(0,   p10 - marginDeg);
    outHMax = std::min(179, p90 + marginDeg);

    std::cout << "[BILGI] Otomatik Hue kalibrasyonu: H[" << outHMin << " - " << outHMax
              << "] (" << hueSamples.size() << " ornekten, p10=" << p10 << " p90=" << p90 << ")" << std::endl;

    return true;
}

// Eski (brightness) yontem - 'm' tusuyla geri gecis icin korunuyor.
int g_thresholdValue = 150;

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

        result.widthMM  = medianOf([](const MeasurementData& m){ return m.widthMM;  });
        result.heightMM = medianOf([](const MeasurementData& m){ return m.heightMM; });
        result.pitch    = medianOf([](const MeasurementData& m){ return m.pitch;    });
        result.roll     = medianOf([](const MeasurementData& m){ return m.roll;     });
        result.diagonalMM          = medianOf([](const MeasurementData& m){ return m.diagonalMM; });
        result.perspectiveErrorPct = medianOf([](const MeasurementData& m){ return m.perspectiveErrorPct; });
        result.rotationDeg         = medianOf([](const MeasurementData& m){ return m.rotationDeg; });

        double cx = medianOf([](const MeasurementData& m){ return m.centerMM.x; });
        double cy = medianOf([](const MeasurementData& m){ return m.centerMM.y; });
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
// Olcum Kayit Altyapisi
//--------------------------------------------------

struct LogEntry
{
    std::string timestamp;
    double widthMM;
    double heightMM;
    double centerX;
    double centerY;
    double pitch;
    double roll;
    double rotationDeg;
    double diagonalMM;
    double perspectiveErrorPct;
    std::string status;
};

LogEntry makeLogEntry(const MeasurementData& m, const std::string& statusText)
{
    LogEntry entry;

    entry.timestamp           = nowTimestamp("%Y-%m-%d %H:%M:%S");
    entry.widthMM             = m.widthMM;
    entry.heightMM            = m.heightMM;
    entry.centerX             = m.centerMM.x;
    entry.centerY             = m.centerMM.y;
    entry.pitch                = m.pitch;
    entry.roll                 = m.roll;
    entry.rotationDeg          = m.rotationDeg;
    entry.diagonalMM           = m.diagonalMM;
    entry.perspectiveErrorPct  = m.perspectiveErrorPct;
    entry.status                = statusText;

    return entry;
}

bool exportLogToCSV(const std::vector<LogEntry>& entries, const std::string& filename)
{
    std::ofstream file(filename);

    if(!file.is_open())
        return false;

    file << "Timestamp,Width_mm,Height_mm,Center_X_mm,Center_Y_mm,"
            "Pitch_deg,Roll_deg,Rotation_deg,Diagonal_mm,Perspective_Error_pct,Status\n";

    for(const auto& e : entries)
    {
        file << e.timestamp << ","
             << e.widthMM << ","
             << e.heightMM << ","
             << e.centerX << ","
             << e.centerY << ","
             << e.pitch << ","
             << e.roll << ","
             << e.rotationDeg << ","
             << e.diagonalMM << ","
             << e.perspectiveErrorPct << ","
             << e.status << "\n";
    }

    file.close();
    return true;
}

//--------------------------------------------------
// Compute Target Overlay
//--------------------------------------------------

bool computeTargetOverlayPixels(
    const DLPModel& model,
    const cv::Mat& invHomography,
    bool homographyLoaded,
    std::vector<cv::Point2f>& outCornersPx,
    cv::Point2f& outCenterPx)
{
    if(!homographyLoaded || invHomography.empty())
        return false;

    // 4 referans vidanin olusturdugu fiziksel alan
    constexpr float SCREW_FRAME_WIDTH_MM  = 222.5f;
    constexpr float SCREW_FRAME_HEIGHT_MM = 150.0f;

    // Vida referans alaninin merkezi
    const float centerX = SCREW_FRAME_WIDTH_MM  / 2.0f;  // 111.25 mm
    const float centerY = SCREW_FRAME_HEIGHT_MM / 2.0f;  // 75.00 mm

    // Secilen projeksiyon modelinin yari boyutlari
    const float halfW = model.widthMM  / 2.0f;
    const float halfH = model.heightMM / 2.0f;

    // Projeksiyon hedefini vida merkezinin etrafina yerlestir
    std::vector<cv::Point2f> worldCorners = {
        cv::Point2f(centerX - halfW, centerY - halfH), // TL
        cv::Point2f(centerX + halfW, centerY - halfH), // TR
        cv::Point2f(centerX + halfW, centerY + halfH), // BR
        cv::Point2f(centerX - halfW, centerY + halfH)  // BL
    };

    cv::perspectiveTransform(
        worldCorners,
        outCornersPx,
        invHomography
    );

    if(outCornersPx.size() != 4)
        return false;

    // Hedef merkez = 4 vidanin geometrik merkezi
    std::vector<cv::Point2f> worldCenter = {
        cv::Point2f(centerX, centerY)
    };

    std::vector<cv::Point2f> centerPx;

    cv::perspectiveTransform(
        worldCenter,
        centerPx,
        invHomography
    );

    if(centerPx.empty())
        return false;

    outCenterPx = centerPx[0];

    return true;
}
//--------------------------------------------------
// Draw Result
//--------------------------------------------------

cv::Mat drawResult(
    const cv::Mat& image,
    const std::vector<std::vector<cv::Point>>& contours,
    int maxIndex,
    const DLPModel& selectedModel,
    const MeasurementData& measurement,
    AlignmentStatus status,
    const cv::Mat& invHomography,
    bool homographyLoaded,
    bool cameraConnected,
    bool calibrationLoaded)
{
    int panelWidth = 340;

    cv::Mat result(
        image.rows,
        image.cols + panelWidth,
        CV_8UC3,
        cv::Scalar(35,35,35));

    image.copyTo(
        result(
            cv::Rect(
                0,
                0,
                image.cols,
                image.rows)));

    std::vector<cv::Point2f> targetCornersPx;
    cv::Point2f targetCenterPxF;
    bool targetOverlayValid = computeTargetOverlayPixels(
        selectedModel, invHomography, homographyLoaded, targetCornersPx, targetCenterPxF);

    cv::Point targetCenter;

    if(targetOverlayValid)
    {
        for(int i = 0; i < 4; i++)
        {
            cv::line(
                result,
                targetCornersPx[i],
                targetCornersPx[(i + 1) % 4],
                cv::Scalar(0,255,0),
                2);
        }

        targetCenter = cv::Point(
            static_cast<int>(std::round(targetCenterPxF.x)),
            static_cast<int>(std::round(targetCenterPxF.y)));
    }
    else
    {
        int targetWidth = 400;
        int targetHeight = 225;

        cv::Rect placeholderRect(
            image.cols / 2 - targetWidth / 2,
            image.rows / 2 - targetHeight / 2,
            targetWidth,
            targetHeight);

        cv::rectangle(result, placeholderRect, cv::Scalar(0,140,255), 2);

        cv::putText(
            result,
            "OLCEK DISI (homografi yok)",
            cv::Point(placeholderRect.x, placeholderRect.y - 10),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0,140,255),
            1);

        targetCenter = cv::Point(
            placeholderRect.x + placeholderRect.width / 2,
            placeholderRect.y + placeholderRect.height / 2);
    }

    int panelX = image.cols + 20;

    cv::putText(
        result,
        "DLP MEASUREMENT",
        cv::Point(panelX,32),
        cv::FONT_HERSHEY_SIMPLEX,
        0.8,
        cv::Scalar(255,255,255),
        2);

    cv::line(
        result,
        cv::Point(image.cols,0),
        cv::Point(image.cols,image.rows),
        cv::Scalar(255,255,255),
        2);

    auto drawChecklistLine = [&](int y, const std::string& label, bool ok)
    {
        std::string text = (ok ? std::string("[OK] ") : std::string("[--] ")) + label;
        cv::Scalar color = ok ? cv::Scalar(0,220,0) : cv::Scalar(0,0,255);

        cv::putText(
            result,
            text,
            cv::Point(panelX, y),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2);
    };

    drawChecklistLine(58,  "Camera Connected",    cameraConnected);
    drawChecklistLine(80,  "Calibration Loaded",  calibrationLoaded);
    drawChecklistLine(102, "Homography Loaded",   homographyLoaded);
    drawChecklistLine(124, "Projection Detected", measurement.valid);

    cv::line(
        result,
        cv::Point(panelX, 138),
        cv::Point(image.cols + panelWidth - 20, 138),
        cv::Scalar(90,90,90),
        1);

    cv::putText(
        result,
        "Selected Model",
        cv::Point(panelX,168),
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        cv::Scalar(255,255,0),
        2);

    cv::putText(
        result,
        selectedModel.name,
        cv::Point(panelX,196),
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        cv::Scalar(255,255,255),
        2);

    cv::putText(
        result,
        "Measurements",
        cv::Point(panelX,232),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(0,255,255),
        2);

    char buf[128];

    std::string widthStr = measurement.mmValid
        ? (snprintf(buf, sizeof(buf), "Width  : %.2f mm", measurement.widthMM), std::string(buf))
        : "Width  : N/A";

    std::string heightStr = measurement.mmValid
        ? (snprintf(buf, sizeof(buf), "Height : %.2f mm", measurement.heightMM), std::string(buf))
        : "Height : N/A";

    std::string centerStr = measurement.mmValid
        ? (snprintf(buf, sizeof(buf), "Center : (%.1f, %.1f)", measurement.centerMM.x, measurement.centerMM.y), std::string(buf))
        : "Center : N/A";

    std::string pitchStr = measurement.poseValid
        ? (snprintf(buf, sizeof(buf), "Pitch  : %.2f deg", measurement.pitch), std::string(buf))
        : "Pitch  : N/A";

    std::string rollStr = measurement.poseValid
        ? (snprintf(buf, sizeof(buf), "Roll   : %.2f deg", measurement.roll), std::string(buf))
        : "Roll   : N/A";

    std::string rotationStr = measurement.valid
        ? (snprintf(buf, sizeof(buf), "Rotation : %.2f deg", measurement.rotationDeg), std::string(buf))
        : "Rotation : N/A";

    std::string diagonalStr = measurement.mmValid
        ? (snprintf(buf, sizeof(buf), "Diagonal : %.2f mm", measurement.diagonalMM), std::string(buf))
        : "Diagonal : N/A";

    std::string perspectiveStr = measurement.mmValid
        ? (snprintf(buf, sizeof(buf), "Perspektif : %.2f %%", measurement.perspectiveErrorPct), std::string(buf))
        : "Perspektif : N/A";

    cv::putText(result, widthStr,      cv::Point(panelX,264), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 2);
    cv::putText(result, heightStr,     cv::Point(panelX,291), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 2);
    cv::putText(result, centerStr,     cv::Point(panelX,318), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 2);
    cv::putText(result, pitchStr,      cv::Point(panelX,345), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 2);
    cv::putText(result, rollStr,       cv::Point(panelX,372), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 2);
    cv::putText(result, rotationStr,   cv::Point(panelX,399), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 2);
    cv::putText(result, diagonalStr,   cv::Point(panelX,426), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 2);
    cv::putText(result, perspectiveStr,cv::Point(panelX,453), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 2);

    cv::putText(
        result,
        "Status :",
        cv::Point(panelX,489),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(0,255,255),
        2);

    std::string statusText  = alignmentStatusToString(status);
    cv::Scalar  statusColor = alignmentStatusToColor(status);

    cv::putText(
        result,
        statusText,
        cv::Point(panelX,522),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        statusColor,
        2);

    std::string detailText = formatAlignmentDetail(status, measurement, selectedModel);

    if(!detailText.empty())
    {
        cv::putText(
            result,
            detailText,
            cv::Point(panelX,550),
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            cv::Scalar(200,200,200),
            2);
    }

    if(maxIndex == -1)
        return result;

    cv::drawContours(result, contours, maxIndex, cv::Scalar(0,255,0), 3);

    cv::Rect box = cv::boundingRect(contours[maxIndex]);
    cv::rectangle(result, box, cv::Scalar(255,0,0), 2);

    cv::RotatedRect rotated = cv::minAreaRect(contours[maxIndex]);
    cv::Point2f vertices[4];
    rotated.points(vertices);

    for(int i = 0; i < 4; i++)
    {
        cv::line(result, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,255), 2);
    }

    cv::Point center(
        static_cast<int>(std::round(rotated.center.x)),
        static_cast<int>(std::round(rotated.center.y)));

    cv::circle(result, targetCenter, 5, cv::Scalar(0,255,0), -1);

    cv::circle(result, center, 6, cv::Scalar(0,0,255), -1);

    cv::line(result, center, targetCenter, cv::Scalar(255,0,255), 2);

    return result;
}

//--------------------------------------------------
// Load Calibration
//--------------------------------------------------

bool loadCalibration(
    const std::string& filename,
    cv::Mat& cameraMatrix,
    cv::Mat& distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if(!fs.isOpened())
    {
        std::cout << "Calibration file bulunamadi!" << std::endl;
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    if(cameraMatrix.empty() || distCoeffs.empty())
    {
        std::cout << "Calibration dosyasi bozuk veya eksik!" << std::endl;
        return false;
    }

    std::cout << "Calibration yuklendi." << std::endl;
    return true;
}

//--------------------------------------------------
// Load Homography
//--------------------------------------------------

bool loadHomography(
    const std::string& filename,
    cv::Mat& homographyMatrix)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if(!fs.isOpened())
    {
        std::cout << "Homography file bulunamadi!" << std::endl;
        return false;
    }

    fs["homography_matrix"] >> homographyMatrix;
    fs.release();

    if(homographyMatrix.empty() || homographyMatrix.rows != 3 || homographyMatrix.cols != 3)
    {
        std::cout << "Homography dosyasi bozuk veya eksik!" << std::endl;
        return false;
    }

    std::cout << "Homography yuklendi." << std::endl;
    return true;
}

//--------------------------------------------------
// Undistort Frame
//--------------------------------------------------

cv::Mat undistortFrame(
    const cv::Mat& image,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs)
{
    cv::Mat corrected;
    cv::undistort(image, corrected, cameraMatrix, distCoeffs);
    return corrected;
}

//--------------------------------------------------
// MAIN
//--------------------------------------------------


//==================================================
// AUTOMATIC SCREW REFERENCE DETECTION
//==================================================

std::vector<cv::Point2f> detectScrewCandidates(
    const cv::Mat& grayUndistorted,
    cv::Mat& debugDisplay)
{
    std::vector<cv::Point2f> candidates;

    if(grayUndistorted.empty())
        return candidates;

    cv::Mat blurred;

    cv::GaussianBlur(
        grayUndistorted,
        blurred,
        cv::Size(7, 7),
        1.5
    );

    std::vector<cv::Vec3f> circles;

    cv::HoughCircles(
        blurred,
        circles,
        cv::HOUGH_GRADIENT,
        1.2,
        25,
        100,
        22,
        5,
        25
    );

    for(const auto& circle : circles)
    {
        cv::Point2f center(circle[0], circle[1]);
        float radius = circle[2];

        int x = cvRound(center.x);
        int y = cvRound(center.y);

        if(x < 0 || x >= grayUndistorted.cols ||
           y < 0 || y >= grayUndistorted.rows)
        {
            continue;
        }

        //--------------------------------------------------
        // BOYANMIS VIDA MERKEZI KONTROLU
        //--------------------------------------------------
        //
        // Tek piksele bakmiyoruz.
        // Merkezin etrafindaki kucuk dairesel alanin
        // ortalama parlakligini hesapliyoruz.
        //--------------------------------------------------

        int sampleRadius = std::max(
            3,
            cvRound(radius * 0.30f)
        );

        cv::Mat centerMask = cv::Mat::zeros(
            grayUndistorted.size(),
            CV_8UC1
        );

        cv::circle(
            centerMask,
            cv::Point(x, y),
            sampleRadius,
            cv::Scalar(255),
            -1
        );

        double meanIntensity =
            cv::mean(grayUndistorted, centerMask)[0];

 //--------------------------------------------------
// VIDA GORUNUM FILTRESI
//--------------------------------------------------

// Tek bir sert kosul yerine puanlama kullaniyoruz.
// Isik degisimlerinde gercek vidanin bir karede kaybolmasini azaltir.

const double DARK_CENTER_STRONG = 100.0;
const double DARK_CENTER_WEAK   = 135.0;

const double RING_MIN_STRONG = 80.0;
const double RING_MIN_WEAK   = 60.0;

const double CONTRAST_STRONG = 20.0;
const double CONTRAST_WEAK   = 8.0;


//--------------------------------------------------
// HALKA HESABI
//--------------------------------------------------

int innerRadius =
    std::max(3, cvRound(radius * 0.45f));

int outerRadius =
    std::max(innerRadius + 2, cvRound(radius * 0.90f));

cv::Mat outerMask = cv::Mat::zeros(
    grayUndistorted.size(),
    CV_8UC1
);

cv::Mat innerMask = cv::Mat::zeros(
    grayUndistorted.size(),
    CV_8UC1
);

cv::circle(
    outerMask,
    cv::Point(x, y),
    outerRadius,
    cv::Scalar(255),
    -1
);

cv::circle(
    innerMask,
    cv::Point(x, y),
    innerRadius,
    cv::Scalar(255),
    -1
);

cv::subtract(
    outerMask,
    innerMask,
    outerMask
);

double ringMean =
    cv::mean(grayUndistorted, outerMask)[0];

double contrast =
    ringMean - meanIntensity;


//--------------------------------------------------
// PUANLAMA
//--------------------------------------------------

int appearanceScore = 0;

// Merkez ne kadar koyu?
if(meanIntensity < DARK_CENTER_STRONG)
    appearanceScore += 2;
else if(meanIntensity < DARK_CENTER_WEAK)
    appearanceScore += 1;

// Metal halka yeterince parlak mi?
if(ringMean > RING_MIN_STRONG)
    appearanceScore += 2;
else if(ringMean > RING_MIN_WEAK)
    appearanceScore += 1;

// Merkez ile halka arasinda kontrast var mi?
if(contrast > CONTRAST_STRONG)
    appearanceScore += 2;
else if(contrast > CONTRAST_WEAK)
    appearanceScore += 1;


//--------------------------------------------------
// SON KARAR
//--------------------------------------------------

// Maksimum puan = 6.
// 3 ve uzeri vida adayi olarak kabul edilir.
bool accepted = appearanceScore >= 3;


//--------------------------------------------------
// DEBUG
//--------------------------------------------------

std::cout
    << "[VIDA ADAY]"
    << " x=" << x
    << " y=" << y
    << " r=" << radius
    << " center=" << meanIntensity
    << " ring=" << ringMean
    << " contrast=" << contrast
    << " score=" << appearanceScore
    << (accepted ? " -> KABUL" : " -> RED")
    << std::endl;

//--------------------------------------------------
// DEBUG
//--------------------------------------------------

std::cout
    << "[VIDA ADAY]"
    << " x=" << x
    << " y=" << y
    << " r=" << radius
    << " center=" << meanIntensity
    << " ring=" << ringMean
    << " contrast=" << contrast
    << (accepted ? " -> KABUL" : " -> RED")
    << std::endl;


if(!accepted)
    continue;

candidates.push_back(center);

       
        //--------------------------------------------------
        // Kabul edilen adaylari sari ciz
        //--------------------------------------------------

        cv::circle(
            debugDisplay,
            center,
            cvRound(radius),
            cv::Scalar(0, 255, 255),
            2
        );

        // Gercek Hough merkezi
        cv::circle(
            debugDisplay,
            center,
            3,
            cv::Scalar(0, 0, 255),
            -1
        );

        // Koyuluk olctugumuz alan
        cv::circle(
            debugDisplay,
            center,
            sampleRadius,
            cv::Scalar(255, 0, 255),
            1
        );
    }

    return candidates;
}
//--------------------------------------------------
// Adaylar arasindan en uygun 4 referans vidasini sec
//--------------------------------------------------

bool selectBestScrewQuad(
    const std::vector<cv::Point2f>& candidates,
    std::vector<cv::Point2f>& bestQuad)
{
    bestQuad.clear();

    if(candidates.size() < 4)
        return false;

    const double expectedRatio =
        static_cast<double>(SCREW_WIDTH_MM) /
        static_cast<double>(SCREW_HEIGHT_MM);

    double bestScore = std::numeric_limits<double>::max();

    //--------------------------------------------------
    // Tum 4'lu kombinasyonlari dene
    //--------------------------------------------------

    for(size_t a = 0; a < candidates.size() - 3; a++)
    {
        for(size_t b = a + 1; b < candidates.size() - 2; b++)
        {
            for(size_t c = b + 1; c < candidates.size() - 1; c++)
            {
                for(size_t d = c + 1; d < candidates.size(); d++)
                {
                    std::vector<cv::Point2f> pts = {
                        candidates[a],
                        candidates[b],
                        candidates[c],
                        candidates[d]
                    };

                    //--------------------------------------------------
                    // Noktalari TL, TR, BR, BL olarak sirala
                    //--------------------------------------------------

                    cv::Point2f center(0.0f, 0.0f);

                    for(const auto& p : pts)
                        center += p;

                    center *= 0.25f;

                    std::vector<cv::Point2f> top;
                    std::vector<cv::Point2f> bottom;

                    for(const auto& p : pts)
                    {
                        if(p.y < center.y)
                            top.push_back(p);
                        else
                            bottom.push_back(p);
                    }

                    // 2 ust + 2 alt nokta bekliyoruz
                    if(top.size() != 2 || bottom.size() != 2)
                        continue;

                    std::sort(
                        top.begin(),
                        top.end(),
                        [](const cv::Point2f& p1,
                           const cv::Point2f& p2)
                        {
                            return p1.x < p2.x;
                        }
                    );

                    std::sort(
                        bottom.begin(),
                        bottom.end(),
                        [](const cv::Point2f& p1,
                           const cv::Point2f& p2)
                        {
                            return p1.x < p2.x;
                        }
                    );

                    std::vector<cv::Point2f> ordered = {
                        top[0],       // TL
                        top[1],       // TR
                        bottom[1],    // BR
                        bottom[0]     // BL
                    };

                    //--------------------------------------------------
                    // Kenar uzunluklari
                    //--------------------------------------------------

                    double topWidth =
                        cv::norm(ordered[0] - ordered[1]);

                    double rightHeight =
                        cv::norm(ordered[1] - ordered[2]);

                    double bottomWidth =
                        cv::norm(ordered[2] - ordered[3]);

                    double leftHeight =
                        cv::norm(ordered[3] - ordered[0]);

                    if(topWidth < 1.0 ||
                       bottomWidth < 1.0 ||
                       rightHeight < 1.0 ||
                       leftHeight < 1.0)
                    {
                        continue;
                    }

                    double avgWidth =
                        (topWidth + bottomWidth) / 2.0;

                    double avgHeight =
                        (leftHeight + rightHeight) / 2.0;

                    double measuredRatio =
                        avgWidth / avgHeight;

                    //--------------------------------------------------
                    // 1) Fiziksel en/boy oranina yakinlik
                    //--------------------------------------------------

                    double ratioError =
                        std::abs(measuredRatio - expectedRatio) /
                        expectedRatio;

                    //--------------------------------------------------
                    // 2) Ust ve alt kenar birbirine benzemeli
                    //--------------------------------------------------

                    double widthAsym =
                        std::abs(topWidth - bottomWidth) /
                        std::max(topWidth, bottomWidth);

                    //--------------------------------------------------
                    // 3) Sol ve sag kenar birbirine benzemeli
                    //--------------------------------------------------

                    double heightAsym =
                        std::abs(leftHeight - rightHeight) /
                        std::max(leftHeight, rightHeight);

                    //--------------------------------------------------
                    // 4) Kosegenler birbirine yakin olmali
                    //--------------------------------------------------

                    double diag1 =
                        cv::norm(ordered[0] - ordered[2]);

                    double diag2 =
                        cv::norm(ordered[1] - ordered[3]);

                    double diagAsym =
                        std::abs(diag1 - diag2) /
                        std::max(diag1, diag2);

                    //--------------------------------------------------
                    // Cok bozuk dortgenleri direkt ele
                    //--------------------------------------------------

                    if(ratioError > 0.30)
                        continue;

                    if(widthAsym > 0.20)
                        continue;

                    if(heightAsym > 0.20)
                        continue;

                    if(diagAsym > 0.20)
                        continue;

                    //--------------------------------------------------
                    // Toplam skor
                    // Dusuk skor = daha iyi referans dortgeni
                    //--------------------------------------------------

                    double score =
                        ratioError  * 4.0 +
                        widthAsym   * 2.0 +
                        heightAsym  * 2.0 +
                        diagAsym    * 1.5;

                    //--------------------------------------------------
                    // Daha buyuk dortgeni hafif tercih et
                    //
                    // Boylece birbirine yakin 4 kucuk vida yerine
                    // kasanin 4 kosesindeki referans vidalarinin
                    // secilme ihtimali artar.
                    //--------------------------------------------------

                    double area =
                        std::abs(
                            cv::contourArea(
                                std::vector<cv::Point2f>{
                                    ordered[0],
                                    ordered[1],
                                    ordered[2],
                                    ordered[3]
                                }
                            )
                        );

                    if(area > 1.0)
                        score -= std::min(area / 1000000.0, 0.25);

                    if(score < bestScore)
                    {
                        bestScore = score;
                        bestQuad = ordered;
                    }
                }
            }
        }
    }

    if(bestQuad.size() != 4)
        return false;

    std::cout
        << "[AUTO] En iyi vida dortlusu bulundu."
        << " Score = "
        << bestScore
        << std::endl;

    return true;
}

//--------------------------------------------------
// Dortgen gecerlilik kontrolu (yanlis vida secimine karsi)
//--------------------------------------------------
bool sanityCheckQuad(const std::vector<cv::Point2f>& ordered, std::string& reason)
{
    double d01 = cv::norm(ordered[0] - ordered[1]);
    double d12 = cv::norm(ordered[1] - ordered[2]);
    double d23 = cv::norm(ordered[2] - ordered[3]);
    double d30 = cv::norm(ordered[3] - ordered[0]);

    double diag02 = cv::norm(ordered[0] - ordered[2]);
    double diag13 = cv::norm(ordered[1] - ordered[3]);

    double widthAsym  = std::abs(d01 - d23) / std::max(d01, d23);
    double heightAsym = std::abs(d12 - d30) / std::max(d12, d30);
    double diagAsym   = std::abs(diag02 - diag13) / std::max(diag02, diag13);

    double expectedRatio = SCREW_WIDTH_MM / SCREW_HEIGHT_MM;
    double measuredRatio = ((d01 + d23) / 2.0) / ((d12 + d30) / 2.0);
    double ratioErrPct = std::abs(measuredRatio - expectedRatio) / expectedRatio * 100.0;

    if(widthAsym > 0.15 || heightAsym > 0.15)
    {
        reason = "Karsit kenarlar arasinda buyuk asimetri - bir ROI yanlis vidaya kaymis olabilir.";
        return false;
    }
    if(diagAsym > 0.15)
    {
        reason = "Kosegenler arasinda buyuk fark - dortgen carpik.";
        return false;
    }
    if(ratioErrPct > 20.0)
    {
        reason = "Olculen en/boy orani beklenenden cok sapiyor (%" + std::to_string(int(ratioErrPct)) + ").";
        return false;
    }
    return true;
}
//--------------------------------------------------
// AUTO SCREW HOMOGRAPHY
//--------------------------------------------------

bool autoCalibrateScrewHomography(
    cv::VideoCapture& cap,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    bool calibrationLoaded,
    cv::Mat& outHomography)
{
    std::cout << "\n=== OTOMATIK VIDA REFERANSI ===\n"
              << "4 referans vida kameradan araniyor...\n";

    const int REQUIRED_GOOD_FRAMES = 8;
    const double MAX_POINT_JUMP_PX = 15.0;

    int goodFrames = 0;

    // Tek bir kotu kare tum stabiliteyi bozmasin.
    // Arka arkaya en fazla 3 kotu kareyi tolere ediyoruz.
    int missedFrames = 0;
    const int MAX_MISSED_FRAMES_AUTO = 3;
    
    std::vector<cv::Point2f> previousQuad;
    std::vector<cv::Point2f> accumulated(4, cv::Point2f(0.0f, 0.0f));

    cv::namedWindow("Auto Screw Reference", cv::WINDOW_NORMAL);

    while(goodFrames < REQUIRED_GOOD_FRAMES)
    {
        cv::Mat frame;
        cap >> frame;

        if(frame.empty())
        {
            std::cerr << "[HATA] Vida referansi icin kamera karesi alinamadi.\n";
            cv::destroyWindow("Auto Screw Reference");
            return false;
        }

        //--------------------------------------------------
        // ScrewDetection ile ayni koordinat sistemi
        //--------------------------------------------------

        if(calibrationLoaded)
        {
            cv::Mat corrected;
            cv::undistort(frame, corrected, cameraMatrix, distCoeffs);
            frame = corrected;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Mat display = frame.clone();

        //--------------------------------------------------
        // Vida adaylarini bul
        //--------------------------------------------------

        std::vector<cv::Point2f> candidates =
            detectScrewCandidates(gray, display);

        std::vector<cv::Point2f> quad;

        bool found =
            selectBestScrewQuad(candidates, quad);

        bool accepted = found && quad.size() == 4;

        //--------------------------------------------------
        // Geometrik sanity check
        //--------------------------------------------------

        if(accepted)
        {
            std::string reason;

            if(!sanityCheckQuad(quad, reason))
            {
                accepted = false;
            }
        }

        //--------------------------------------------------
        // Bir onceki kareyle ayni dortgen mi?
        //--------------------------------------------------

        if(accepted && !previousQuad.empty())
        {
            for(int i = 0; i < 4; i++)
            {
                double jump =
                    cv::norm(quad[i] - previousQuad[i]);

                if(jump > MAX_POINT_JUMP_PX)
                {
                    accepted = false;
                    break;
                }
            }
        }

        //--------------------------------------------------
        // Kabul edilen kare
        //--------------------------------------------------

        if(accepted)
        {
            missedFrames = 0;
        
            if(goodFrames == 0)
            {
                for(int i = 0; i < 4; i++)
                    accumulated[i] = cv::Point2f(0.0f, 0.0f);
            }

            for(int i = 0; i < 4; i++)
                accumulated[i] += quad[i];

            previousQuad = quad;

            goodFrames++;

            //--------------------------------------------------
            // Ekranda referans dortgenini goster
            //--------------------------------------------------

            for(int i = 0; i < 4; i++)
            {
                cv::circle(
                    display,
                    quad[i],
                    8,
                    cv::Scalar(0,255,0),
                    -1
                );

                cv::putText(
                    display,
                    SCREW_NAMES[i],
                    quad[i] + cv::Point2f(10,-10),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.65,
                    cv::Scalar(0,255,0),
                    2
                );

                cv::line(
                    display,
                    quad[i],
                    quad[(i+1)%4],
                    cv::Scalar(0,255,0),
                    2
                );
            }
        }
        else
{
    //--------------------------------------------------
    // Bu karede vida dortlusu bulunamadi.
    //
    // Tek bir kotu frame geldiginde onceki iyi
    // tespitleri atmiyoruz. Bir kac kotu frame'i
    // tolere ediyoruz.
    //--------------------------------------------------

    missedFrames++;

    std::cout
        << "[AUTO] Gecici vida kaybi: "
        << missedFrames
        << "/"
        << MAX_MISSED_FRAMES_AUTO
        << std::endl;

    //--------------------------------------------------
    // Ancak uzun sure bulunamazsa stabiliteyi sifirla.
    //--------------------------------------------------

    if(missedFrames > MAX_MISSED_FRAMES_AUTO)
    {
        std::cout
            << "[AUTO] Vida referansi uzun sure kayip. "
            << "Stabilite yeniden baslatiliyor."
            << std::endl;

        goodFrames = 0;
        missedFrames = 0;

        previousQuad.clear();

        for(int i = 0; i < 4; i++)
            accumulated[i] = cv::Point2f(0.0f, 0.0f);
    }
}

        //--------------------------------------------------
        // Durum bilgisi
        //--------------------------------------------------

        std::string status =
            "Vida referansi: " +
            std::to_string(goodFrames) +
            "/" +
            std::to_string(REQUIRED_GOOD_FRAMES);

        cv::putText(
            display,
            status,
            cv::Point(20,35),
            cv::FONT_HERSHEY_SIMPLEX,
            0.75,
            accepted
                ? cv::Scalar(0,255,0)
                : cv::Scalar(0,0,255),
            2
        );

        cv::imshow("Auto Screw Reference", display);

        int key = cv::waitKey(30);

        if(key == 27)
        {
            cv::destroyWindow("Auto Screw Reference");
            return false;
        }
    }

    //--------------------------------------------------
    // 8 karenin ortalama vida merkezleri
    //--------------------------------------------------

    std::vector<cv::Point2f> screwPoints(4);

    for(int i = 0; i < 4; i++)
        screwPoints[i] =
            accumulated[i] / static_cast<float>(REQUIRED_GOOD_FRAMES);

    //--------------------------------------------------
    // Fiziksel vida koordinat sistemi
    //
    // TL = 0,0
    // TR = 222.5,0
    // BR = 222.5,150
    // BL = 0,150
    //--------------------------------------------------

    std::vector<cv::Point2f> worldPoints = {
        {0.0f,             0.0f},
        {SCREW_WIDTH_MM,   0.0f},
        {SCREW_WIDTH_MM,   SCREW_HEIGHT_MM},
        {0.0f,             SCREW_HEIGHT_MM}
    };

    outHomography =
        cv::findHomography(screwPoints, worldPoints);

    if(outHomography.empty())
    {
        std::cerr
            << "[HATA] Otomatik vida homografisi hesaplanamadi.\n";

        cv::destroyWindow("Auto Screw Reference");
        return false;
    }

    //--------------------------------------------------
    // Son kontrol: bulunan vidalar gercek dunya
    // koordinatlarina geri projekte ediliyor.
    //--------------------------------------------------

    std::vector<cv::Point2f> projected;

    cv::perspectiveTransform(
        screwPoints,
        projected,
        outHomography
    );

    double error = 0.0;

    for(int i = 0; i < 4; i++)
        error += cv::norm(projected[i] - worldPoints[i]);

    error /= 4.0;

    std::cout
        << "[AUTO] Vida homography olusturuldu.\n"
        << "[AUTO] Ortalama reprojeksiyon hatasi: "
        << error
        << " mm\n";

    //--------------------------------------------------
    // Referans merkezini de yazdir
    //--------------------------------------------------

    cv::Point2f screwCenterPx(0.0f, 0.0f);

    for(const auto& p : screwPoints)
        screwCenterPx += p;

    screwCenterPx *= 0.25f;

    std::cout
        << "[AUTO] Vida merkezi PIXEL = ("
        << screwCenterPx.x << ", "
        << screwCenterPx.y << ")\n"
        << "[AUTO] Vida merkezi MM = ("
        << SCREW_WIDTH_MM / 2.0f << ", "
        << SCREW_HEIGHT_MM / 2.0f << ")\n";

    cv::destroyWindow("Auto Screw Reference");

    return true;
}

//==================================================
// END AUTOMATIC SCREW REFERENCE DETECTION
//==================================================

int main()
{
    cv::VideoCapture cap(0);
    std::cout << "Camera WIDTH  = "
    << cap.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;

std::cout << "Camera HEIGHT = "
    << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;

    if(!cap.isOpened())
    {
        std::cout << "Kamera acilamadi!" << std::endl;
        return -1;
    }

    // FIX (optimizasyon): Otomatik pozlama/beyaz dengesi acik oldugu surece
    // kamera her karede kendi V/S seviyelerini kaydirabilir - bu da HSV
    // esiklemesinin (sabit ya da adaptif) gun be gun/an be an farkli sonuc
    // vermesine yol acar. Sabit exposure/WB, adaptif Otsu esiginin daha
    // KARARLI calismasini saglar. NOT: bu ayarlar platforma/backend'e
    // (V4L2, AVFoundation, DirectShow) gore farkli davranabilir veya
    // yoksayilabilir; deger uygulanamazsa sessizce yoksayilir.
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);   // bazi backend'lerde 0=manuel, 1=otomatik
    cap.set(cv::CAP_PROP_AUTO_WB, 0);

    
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs   = cv::Mat::zeros(5, 1, CV_64F);

    bool calibrationLoaded =
        loadCalibration("calibration.yml", cameraMatrix, distCoeffs);

    if(!calibrationLoaded)
    {
        std::cout << "[UYARI] Kalibrasyon yok, distorsiyon duzeltmesi ve pose hesaplamasi yapilmayacak." << std::endl;
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        distCoeffs   = cv::Mat::zeros(5, 1, CV_64F);
    }

   //--------------------------------------------------
// HOMOGRAPHY
// Once kameradan 4 referans vidayi otomatik bul.
// Basarisiz olursa eski screw_homography.yml kullan.
//--------------------------------------------------

cv::Mat homographyMatrix;

bool homographyLoaded = false;

std::cout
    << "\n[BILGI] 4 referans vida otomatik araniyor..."
    << std::endl;

bool autoHomographyOK =
    autoCalibrateScrewHomography(
        cap,
        cameraMatrix,
        distCoeffs,
        calibrationLoaded,
        homographyMatrix
    );

if(autoHomographyOK)
{
    homographyLoaded = true;

    std::cout
        << "[BILGI] Homography CANLI vida tespitinden olusturuldu."
        << std::endl;
}
else
{
 
    //--------------------------------------------------
    // ASAMA 1 BASARISIZ.
    // Eski homography ile devam ETME.
    // Vida referansi bulunmadan olcum asamasina gecilmez.
    //--------------------------------------------------

    std::cerr
        << "\n[HATA] 4 referans vida stabil olarak tespit edilemedi."
        << "\n[HATA] ASAMA 2 BASLATILMAYACAK."
        << "\n       Program sonlandiriliyor."
        << std::endl;

    cap.release();
    cv::destroyAllWindows();
    return -1;
}
    cv::Mat invHomographyMatrix;

    if(homographyLoaded && !homographyMatrix.empty())
    {
        bool inverted = cv::invert(homographyMatrix, invHomographyMatrix);

        if(!inverted)
        {
            std::cout << "[UYARI] Homografi tersinir degil, target rectangle cizilemeyecek." << std::endl;
            homographyLoaded = false;
        }
    }

    // FIX (YENI): daha once bulunmus HSV ayarlarini dosyadan yukle -
    // program her acildiginda trackbar'lari sifirdan ayarlama ihtiyacini
    // ortadan kaldirir. Dosya yoksa (ilk calistirma) varsayilan degerler
    // (g_hMin=100 vb.) kullanilir.
    loadHsvSettings("hsv_settings.yml", g_hMin, g_hMax, g_sMin, g_sMax, g_vMin, g_vMax);

   //--------------------------------------------------
// SABIT DLP MODELI
// Sistem su an yalnizca 134.4 x 75.6 mm
// beyaz dikdortgen ile otomatik calisiyor.
//--------------------------------------------------

DLPModel selectedModel = g_dlpModels[3];

TestPatternType selectedPattern =
    TestPatternType::WHITE_RECTANGLE;

std::cout
    << "\n[BILGI] Sabit DLP modeli otomatik secildi: "
    << selectedModel.name
    << std::endl;

std::cout
    << "[BILGI] Test deseni: Tam beyaz dikdortgen"
    << std::endl;

    std::cout << "\nSecilen model: " << selectedModel.name << std::endl;
    std::cout << "Baslatiliyor...\n"
              << "  'v' detayli log   's' screenshot   'e' CSV export   ESC cikis\n"
              << "  'm' tespit modunu degistir (Adaptif HSV / Sabit HSV / Parlaklik Esigi)\n"
              << "  'i' ICERI (pattern) HSV ornegi topla   'o' DISARI (arka plan) HSV ornegi topla\n"
              << "  'r' toplanan orneklerden onerilen HSV araligini yazdir\n"
              << "  'a' pattern'den otomatik Hue bandi kalibre et (isik ekranda gorunur olmali)\n"
              << "  'p' guncel HSV ayarlarini kaydet (hsv_settings.yml) - cikiste otomatik da kaydedilir\n" << std::endl;

    if(homographyLoaded)
    {
        cv::Mat probeFrame = captureFrame(cap);

        if(!probeFrame.empty())
        {
            DLPModel largestModel = selectedModel;

            std::vector<cv::Point2f> probeCorners;
            cv::Point2f probeCenter;

            bool ok = computeTargetOverlayPixels(
                largestModel, invHomographyMatrix, homographyLoaded, probeCorners, probeCenter);

            if(ok)
            {
                bool allInside = true;

                for(const auto& pt : probeCorners)
                {
                    if(pt.x < 0 || pt.y < 0 ||
                       pt.x > probeFrame.cols || pt.y > probeFrame.rows)
                    {
                        allInside = false;
                        break;
                    }
                }

                if(!allInside)
                {
                    std::cout << "\n[UYARI] KAMERA KONUM KONTROLU: En buyuk desteklenen DLP modeli "
                              << "(" << largestModel.name << ") kamera goruntu cercevesine sigmiyor.\n"
                              << "        Kamera yanlis konumlanmis olabilir - mesafeyi/acisini kontrol edin "
                              << "ve Homography programini yeniden calistirin.\n" << std::endl;
                }
                else
                {
                    std::cout << "[BILGI] Kamera konum kontrolu OK: en buyuk model calisma alanina sigiyor." << std::endl;
                }
            }
        }
    }

    cv::namedWindow("Mask (debug)");
    cv::createTrackbar("Threshold (Brightness)", "Mask (debug)", &g_thresholdValue, 255);
    cv::createTrackbar("H min", "Mask (debug)", &g_hMin, 179);
    cv::createTrackbar("H max", "Mask (debug)", &g_hMax, 179);
    cv::createTrackbar("S min", "Mask (debug)", &g_sMin, 255);
    cv::createTrackbar("S max", "Mask (debug)", &g_sMax, 255);
    cv::createTrackbar("V min (sabit modda / Otsu tabani)", "Mask (debug)", &g_vMin, 255);
    cv::createTrackbar("V max (manuel modda)", "Mask (debug)", &g_vMax, 255);
    cv::moveWindow("Mask (debug)", 900, 50);

    HsvCalibState calibState;
    cv::namedWindow("Machine Vision");
    cv::setMouseCallback("Machine Vision", hsvCalibMouseCallback, &calibState);

    MeasurementHistory history(/*maxSize=*/7, /*minSamples=*/3);

    std::vector<LogEntry> logBuffer;
    auto lastLogTime = std::chrono::steady_clock::now() - std::chrono::seconds(5);

    while(true)
    {
        cv::Mat frame = captureFrame(cap);
 

        if(frame.empty())
            break;

        bool cameraConnected = true;

        if(calibrationLoaded)
        {
            frame = undistortFrame(frame, cameraMatrix, distCoeffs);
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Mat mask;

        if(g_detectionMode == DetectionMode::HSV_ADAPTIVE)
        {
            cv::cvtColor(frame, calibState.hsvFrame, cv::COLOR_BGR2HSV);
            // g_vMin buradaki tabani (Otsu'nun ALTINA dusemeyecegi minimum) belirliyor -
            // trackbar'dan elle dogruladiginiz iyi degeri (once bulunan ~110-140 gibi)
            // burada girerseniz Otsu artik daha gevsek bir esik secemez.
            mask = detectColorRegionAdaptive(calibState.hsvFrame, g_hMin, g_hMax, g_sMin, g_vMin, g_lastOtsuVThresh);
        }
        else if(g_detectionMode == DetectionMode::HSV_COLOR_RANGE)
        {
            cv::cvtColor(frame, calibState.hsvFrame, cv::COLOR_BGR2HSV);
            mask = detectColorRegion(calibState.hsvFrame, g_hMin, g_hMax, g_sMin, g_sMax, g_vMin, g_vMax);
        }
        else
        {
            mask = detectBrightRegion(gray, g_thresholdValue);
        }

        cv::Mat clean = cleanMask(mask);

        cv::imshow("Mask (debug)", clean);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        int maxIndex = findLargestContour(clean, contours, hierarchy);

        MeasurementData rawMeasurement = measureObject(
            contours,
            maxIndex,
            gray,
            homographyMatrix,
            homographyLoaded,
            cameraMatrix,
            distCoeffs,
            calibrationLoaded,
            selectedModel);

        history.push(rawMeasurement);

        MeasurementData displayMeasurement;
        AlignmentStatus status;

        if(!rawMeasurement.valid || history.empty())
        {
            displayMeasurement = rawMeasurement;
            status = AlignmentStatus::NO_DATA;
        }
        else if(!history.ready())
        {
            displayMeasurement = rawMeasurement;
            status = AlignmentStatus::STABILIZING;
        }
        else
        {
            displayMeasurement = history.median();
            status = updateAlignmentStatus(displayMeasurement, selectedModel);
        }

        g_status = status;

        if(status != AlignmentStatus::NO_DATA && status != AlignmentStatus::STABILIZING &&
           status != AlignmentStatus::SIZE_MISMATCH &&
           displayMeasurement.valid && displayMeasurement.mmValid && displayMeasurement.poseValid)
        {
            auto now = std::chrono::steady_clock::now();

            if(std::chrono::duration_cast<std::chrono::milliseconds>(now - lastLogTime).count() >= 1000)
            {
                logBuffer.push_back(makeLogEntry(displayMeasurement, alignmentStatusToString(status)));
                lastLogTime = now;
            }
        }

        cv::Mat result =
            drawResult(
                frame,
                contours,
                maxIndex,
                selectedModel,
                displayMeasurement,
                status,
                invHomographyMatrix,
                homographyLoaded,
                cameraConnected,
                calibrationLoaded);

        std::string modeText;
        switch(g_detectionMode)
        {
        case DetectionMode::HSV_ADAPTIVE:
            modeText = "Mode: Adaptif HSV (Otsu V esigi: " + std::to_string(g_lastOtsuVThresh) +
                       ")  (m: degistir, a: Hue kalibre et, i/o/r: manuel kalibrasyon)";
            break;
        case DetectionMode::HSV_COLOR_RANGE:
            modeText = "Mode: Sabit HSV Renk Araligi (m: degistir, i/o/r: kalibrasyon)";
            break;
        default:
            modeText = "Mode: Parlaklik Esigi (m: degistir)";
            break;
        }
        cv::putText(result, modeText, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0,255,255), 2);

        cv::imshow("Machine Vision", result);

        int key = cv::waitKey(30);

        if(key == 27)
        {
            break;
        }
        else if(key == 'v' || key == 'V')
        {
            g_verboseLog = !g_verboseLog;
            std::cout << "[BILGI] Detayli log " << (g_verboseLog ? "ACIK" : "KAPALI") << std::endl;
        }
        else if(key == 'm' || key == 'M')
        {
            // Adaptif -> Sabit HSV -> Parlaklik Esigi -> (basa don)
            if(g_detectionMode == DetectionMode::HSV_ADAPTIVE)
                g_detectionMode = DetectionMode::HSV_COLOR_RANGE;
            else if(g_detectionMode == DetectionMode::HSV_COLOR_RANGE)
                g_detectionMode = DetectionMode::BRIGHTNESS_THRESHOLD;
            else
                g_detectionMode = DetectionMode::HSV_ADAPTIVE;

            std::string modeName =
                (g_detectionMode == DetectionMode::HSV_ADAPTIVE)     ? "Adaptif HSV (Otsu)" :
                (g_detectionMode == DetectionMode::HSV_COLOR_RANGE)  ? "Sabit HSV Renk Araligi" :
                                                                        "Parlaklik Esigi";

            std::cout << "[BILGI] Tespit modu: " << modeName << std::endl;
        }
        else if(key == 'a' || key == 'A')
        {
            // YENI: pattern ekrandayken Hue bandini otomatik kalibre et.
            cv::Mat hsvNow;
            cv::cvtColor(frame, hsvNow, cv::COLOR_BGR2HSV);

            int newHMin = g_hMin;
            int newHMax = g_hMax;

            if(autoCalibrateHueBand(hsvNow, newHMin, newHMax))
            {
                g_hMin = newHMin;
                g_hMax = newHMax;

                cv::setTrackbarPos("H min", "Mask (debug)", g_hMin);
                cv::setTrackbarPos("H max", "Mask (debug)", g_hMax);

                std::cout << "[BILGI] Hue bandi otomatik guncellendi: H[" << g_hMin << " - " << g_hMax << "]" << std::endl;
            }
        }
        else if(key == 'i' || key == 'I')
        {
            calibState.collectingInside = !calibState.collectingInside;
            calibState.collectingOutside = false;
            std::cout << "[KALIBRASYON] ICERI ornek toplama "
                      << (calibState.collectingInside ? "ACIK - patternin ustune tiklayin" : "KAPALI")
                      << std::endl;
        }
        else if(key == 'o' || key == 'O')
        {
            calibState.collectingOutside = !calibState.collectingOutside;
            calibState.collectingInside = false;
            std::cout << "[KALIBRASYON] DISARI ornek toplama "
                      << (calibState.collectingOutside ? "ACIK - arka plana/metale tiklayin" : "KAPALI")
                      << std::endl;
        }
        else if(key == 'r' || key == 'R')
        {
            printSuggestedHsvRange(calibState);
            calibState.insideSamples.clear();
            calibState.outsideSamples.clear();
            calibState.collectingInside = false;
            calibState.collectingOutside = false;
        }
        else if(key == 'p' || key == 'P')
        {
            // YENI: guncel HSV trackbar/otomatik degerlerini anlik kaydet.
            saveHsvSettings("hsv_settings.yml", g_hMin, g_hMax, g_sMin, g_sMax, g_vMin, g_vMax);
        }
        else if(key == 's' || key == 'S')
        {
            std::string filename = "Result_" + nowTimestamp("%Y%m%d_%H%M%S") + ".png";

            if(cv::imwrite(filename, result))
                std::cout << "[BILGI] Ekran goruntusu kaydedildi: " << filename << std::endl;
            else
                std::cout << "[UYARI] Ekran goruntusu kaydedilemedi." << std::endl;
        }
        else if(key == 'e' || key == 'E')
        {
            if(logBuffer.empty())
            {
                std::cout << "[BILGI] Kayitli olcum yok, CSV olusturulmadi." << std::endl;
            }
            else
            {
                std::string filename = "measurements_log_" + nowTimestamp("%Y%m%d_%H%M%S") + ".csv";

                if(exportLogToCSV(logBuffer, filename))
                    std::cout << "[BILGI] " << logBuffer.size()
                              << " kayit CSV'ye yazildi: " << filename << std::endl;
                else
                    std::cout << "[UYARI] CSV dosyasi olusturulamadi." << std::endl;
            }
        }
    }

    cap.release();
    cv::destroyAllWindows();

    // FIX (YENI): cikiste guncel HSV ayarlarini otomatik kaydet - bir
    // sonraki acilista elle trackbar cekmeye gerek kalmasin. 'p' ile
    // manuel kaydetmeyi unutsaniz da program kapanirken kaydeder.
    saveHsvSettings("hsv_settings.yml", g_hMin, g_hMax, g_sMin, g_sMax, g_vMin, g_vMax);

    if(!logBuffer.empty())
    {
        std::string filename = "measurements_log_" + nowTimestamp("%Y%m%d_%H%M%S") + "_auto.csv";

        if(exportLogToCSV(logBuffer, filename))
            std::cout << "\n[BILGI] Cikiste " << logBuffer.size()
                      << " olcum kaydi otomatik CSV'ye yazildi: " << filename << std::endl;
    }

    return 0;
}