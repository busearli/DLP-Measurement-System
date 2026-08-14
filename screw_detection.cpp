#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>
#include <deque>
#include <algorithm>
#include <numeric>

const float SCREW_WIDTH_MM  = 222.5f;
const float SCREW_HEIGHT_MM = 150.0f;

const double MAX_ACCEPTABLE_REPROJ_MM = 0.3;

// FIX (tam otomasyon): kararlilik icin, bir vida merkezinin son N karede
// ne kadar oynadigini kontrol ediyoruz. Bu kadar kare boyunca standart
// sapma bu esigin ALTINDAYSA "stabil" kabul edilir ve otomatik kaydedilir.
const int STABILITY_FRAMES = 12;
const double STABILITY_TOL_PX = 2.0;

// Arka arkaya birkac kotu kareye izin ver.
// Boylece tek karelik Hough/aydinlatma hatasi kilidi bozmaz.
const int MAX_MISSED_FRAMES = 5;

const char* ROI_FILENAME = "screw_rois.yml";
const char* names[4] = {"TL", "TR", "BR", "BL"};

//--------------------------------------------------
// ROI tabanli otomatik vida merkezi tespiti
//--------------------------------------------------
// Kullanicinin bir kere elle cizdigi (kabaca) ROI kutusu icinde, o ROI'ye
// EN GUCLU eslesen daireyi buluyoruz. ROI zaten tek bir vidayi kapsayacak
// sekilde cizildigi icin komsu vida karisikligi soz konusu degil.
//--------------------------------------------------
bool detectScrewInROI(const cv::Mat& grayUndistorted, const cv::Rect& roi, cv::Point2f& outCenter)
{
    cv::Rect safeROI = roi & cv::Rect(0, 0, grayUndistorted.cols, grayUndistorted.rows);
    if(safeROI.width < 6 || safeROI.height < 6)
        return false;

    cv::Mat patch = grayUndistorted(safeROI);

    cv::Mat blurred;
    cv::GaussianBlur(patch, blurred, cv::Size(5, 5), 1.2);

    int shortSide = std::min(safeROI.width, safeROI.height);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(
        blurred, circles, cv::HOUGH_GRADIENT,
        1.0,
        shortSide * 2.0,                   // minDist: ROI'de tek daire beklenir
        100,                                // Canny ust esik
        18,                                 // akumulator esigi
        std::max(3, (int)(shortSide * 0.15)), // min yaricap
        (int)(shortSide * 0.55)             // max yaricap
    );

    if(!circles.empty())
    {
        // HoughCircles sonuclari akumulator gucune gore SIRALI dondurur -
        // ilk eleman en guclu/guvenilir tespit.
        outCenter = cv::Point2f(safeROI.x + circles[0][0], safeROI.y + circles[0][1]);
        return true;
    }

    // Fallback: Otsu + moment agirlik merkezi
    cv::Mat bin;
    cv::threshold(blurred, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::Moments m = cv::moments(bin, true);
    if(m.m00 < 1.0)
        return false;

    outCenter = cv::Point2f(safeROI.x + (float)(m.m10 / m.m00),
                             safeROI.y + (float)(m.m01 / m.m00));
    return true;
}
//--------------------------------------------------
// Tum goruntuden otomatik vida adaylarini bul
//--------------------------------------------------
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

bool loadCalibration(const std::string& filename, cv::Mat& cameraMatrix, cv::Mat& distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened()) { std::cerr << "[HATA] " << filename << " acilamadi!" << std::endl; return false; }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();
    return !cameraMatrix.empty() && !distCoeffs.empty();
}

cv::Mat undistortFrame(const cv::Mat& image, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
{
    cv::Mat corrected;
    cv::undistort(image, corrected, cameraMatrix, distCoeffs);
    return corrected;
}

bool validateHomography(const cv::Mat& H, const std::vector<cv::Point2f>& imagePoints,
                         const std::vector<cv::Point2f>& worldPoints, double& outErrMM)
{
    outErrMM = 0.0;
    if(H.empty() || H.rows != 3 || H.cols != 3) return false;

    std::vector<cv::Point2f> reprojected;
    cv::perspectiveTransform(imagePoints, reprojected, H);
    if(reprojected.size() != worldPoints.size()) return false;

    double sumErr = 0.0;
    for(size_t i = 0; i < reprojected.size(); i++)
        sumErr += cv::norm(reprojected[i] - worldPoints[i]);

    outErrMM = sumErr / reprojected.size();
    return true;
}

//--------------------------------------------------
// ROI Kaydet / Yukle
//--------------------------------------------------
bool saveScrewROIs(const std::string& filename, const cv::Rect rois[4])
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened()) return false;
    for(int i = 0; i < 4; i++)
    {
        fs << (std::string("roi_") + names[i]) << rois[i];
    }
    fs.release();
    return true;
}

bool loadScrewROIs(const std::string& filename, cv::Rect rois[4])
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened()) return false;
    for(int i = 0; i < 4; i++)
    {
        fs[(std::string("roi_") + names[i])] >> rois[i];
        if(rois[i].width <= 0 || rois[i].height <= 0) { fs.release(); return false; }
    }
    fs.release();
    return true;
}

//--------------------------------------------------
// ROI Kalibrasyon Modu (sadece rig kuruldugunda / kamera oynadiginda
// BIR KERE calistirilir - fareyle her kosenin etrafina kabaca bir
// dikdortgen suruklenir).
//--------------------------------------------------
struct RoiCalibState
{
    bool dragging = false;
    cv::Point start;
    cv::Rect current;
    std::vector<cv::Rect> finished;
};

void roiMouseCallback(int event, int x, int y, int, void* userdata)
{
    RoiCalibState* st = static_cast<RoiCalibState*>(userdata);
    if(st->finished.size() >= 4) return;

    if(event == cv::EVENT_LBUTTONDOWN)
    {
        st->dragging = true;
        st->start = cv::Point(x, y);
        st->current = cv::Rect(x, y, 0, 0);
    }
    else if(event == cv::EVENT_MOUSEMOVE && st->dragging)
    {
        int x0 = std::min(st->start.x, x), y0 = std::min(st->start.y, y);
        int x1 = std::max(st->start.x, x), y1 = std::max(st->start.y, y);
        st->current = cv::Rect(x0, y0, x1 - x0, y1 - y0);
    }
    else if(event == cv::EVENT_LBUTTONUP && st->dragging)
    {
        st->dragging = false;
        if(st->current.width > 8 && st->current.height > 8)
        {
            st->finished.push_back(st->current);
            std::cout << "ROI " << st->finished.size() << "/4 ("
                      << names[st->finished.size()-1] << ") tanimlandi: "
                      << st->current << std::endl;
        }
        st->current = cv::Rect();
    }
}

bool runRoiCalibration(cv::VideoCapture& cap, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                        cv::Rect outRois[4])
{
    cv::namedWindow("ROI Kalibrasyon");
    RoiCalibState st;
    cv::setMouseCallback("ROI Kalibrasyon", roiMouseCallback, &st);

    std::cout << "\n=== ROI KALIBRASYON (SADECE BIR KEZ) ===\n"
              << "Fareyle her KOSE VIDASININ etrafina kabaca bir dikdortgen surukleyin.\n"
              << "Sira: 1=TL(sol-ust) 2=TR(sag-ust) 3=BR(sag-alt) 4=BL(sol-alt)\n"
              << "Kutuyu vida etrafinda BIRAZ BOSLUKLU cizin (vida her zaman kutunun\n"
              << "icinde kalsin, ama komsu vidayi icine ALMASIN).\n"
              << "'z' = son ROI'yi sil   ESC = iptal\n";

    while(st.finished.size() < 4)
    {
        cv::Mat frame;
        cap >> frame;
        if(frame.empty()) break;
        frame = undistortFrame(frame, cameraMatrix, distCoeffs);

        cv::Mat display = frame.clone();
        for(size_t i = 0; i < st.finished.size(); i++)
        {
            cv::rectangle(display, st.finished[i], cv::Scalar(0,255,0), 2);
            cv::putText(display, names[i], st.finished[i].tl() + cv::Point(0,-6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
        }
        if(st.dragging)
            cv::rectangle(display, st.current, cv::Scalar(0,180,255), 2);

        cv::putText(display, "ROI: " + std::to_string(st.finished.size()) + "/4  -  Sira: " +
                    (st.finished.size() < 4 ? names[st.finished.size()] : ""),
                    cv::Point(20,35), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,255,0), 2);

        cv::imshow("ROI Kalibrasyon", display);

        int key = cv::waitKey(30);
        if(key == 27) { cv::destroyWindow("ROI Kalibrasyon"); return false; }
        if((key == 'z' || key == 'Z') && !st.finished.empty())
        {
            st.finished.pop_back();
            std::cout << "[BILGI] Son ROI silindi. Kalan: " << st.finished.size() << "/4" << std::endl;
        }
    }

    for(int i = 0; i < 4; i++) outRois[i] = st.finished[i];
    cv::destroyWindow("ROI Kalibrasyon");
    saveScrewROIs(ROI_FILENAME, outRois);
    std::cout << "[BILGI] " << ROI_FILENAME << " kaydedildi. Bir sonraki calistirmada "
              << "bu ROI'ler otomatik kullanilacak (tekrar cizmenize gerek yok)." << std::endl;
    return true;
}

double stddevOf(const std::deque<cv::Point2f>& buf, bool useX)
{
    if(buf.size() < 2) return std::numeric_limits<double>::max();
    std::vector<double> v;
    for(auto& p : buf) v.push_back(useX ? p.x : p.y);
    double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    double sq = 0.0;
    for(double d : v) sq += (d - mean) * (d - mean);
    return std::sqrt(sq / v.size());
}

cv::Point2f meanOf(const std::deque<cv::Point2f>& buf)
{
    double sx = 0, sy = 0;
    for(auto& p : buf) { sx += p.x; sy += p.y; }
    return cv::Point2f((float)(sx / buf.size()), (float)(sy / buf.size()));
}

int main()
{
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) { std::cerr << "Kamera acilamadi!" << std::endl; return -1; }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    cv::Mat cameraMatrix, distCoeffs;
    if(!loadCalibration("calibration.yml", cameraMatrix, distCoeffs))
    {
        std::cerr << "[HATA] calibration.yml yuklenemedi." << std::endl;
        cap.release();
        return -1;
    }
    std::cout << "[BILGI] Calibration yuklendi. UNDISTORT aktif." << std::endl;

    /*cv::Rect screwROIs[4];
    bool roisLoaded = loadScrewROIs(ROI_FILENAME, screwROIs);

    if(!roisLoaded)
    {
        std::cout << "[BILGI] " << ROI_FILENAME << " bulunamadi - ilk kurulum icin "
                  << "ROI kalibrasyonu baslatiliyor (bu SADECE bir kez gerekli)." << std::endl;
        if(!runRoiCalibration(cap, cameraMatrix, distCoeffs, screwROIs))
        {
            std::cerr << "ROI kalibrasyonu iptal edildi." << std::endl;
            cap.release();
            return -1;
        }
    }
    else
    {
        std::cout << "[BILGI] " << ROI_FILENAME << " yuklendi - vidalar TAMAMEN OTOMATIK "
                  << "tespit edilecek, tiklama gerekmiyor.\n"
                  << "        (Kamera fiziksel olarak oynadiysa 'c' tusuyla ROI'leri "
                  << "yeniden kalibre edebilirsiniz.)" << std::endl;
    }*/

    cv::namedWindow("Screw Reference");

    std::deque<cv::Point2f> history[4]; // TL,TR,BR,BL stabilite tamponu

    std::cout << "\n=== OTOMATIK VIDA TESPITI ===\n"
              << "Sistemi sabit tutun, " << STABILITY_FRAMES << " kare boyunca stabil "
              << "olcum bekleniyor...\n"
              << "'c' = ROI'leri yeniden kalibre et   'r' = stabilite arabellegini sifirla   ESC = cikis\n";

    std::vector<cv::Point2f> finalScrewPoints;
    bool captured = false;
    int missedFrames = 0;
    while(!captured)
    {
        cv::Mat frame;
        cap >> frame;
        if(frame.empty()) break;
        frame = undistortFrame(frame, cameraMatrix, distCoeffs);

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Mat display = frame.clone();
//--------------------------------------------------
// AUTO SCREW DETECTION TEST
//--------------------------------------------------

std::vector<cv::Point2f> autoCandidates =
    detectScrewCandidates(gray, display);

std::vector<cv::Point2f> autoQuad;

bool autoFound =
    selectBestScrewQuad(autoCandidates, autoQuad);

if(autoFound && autoQuad.size() == 4)
{
    const char* names[4] = {
        "TL",
        "TR",
        "BR",
        "BL"
    };

    // Secilen 4 referans vidayi goster
    for(int i = 0; i < 4; i++)
    {
        cv::circle(
            display,
            autoQuad[i],
            8,
            cv::Scalar(0, 255, 0),
            -1
        );

        cv::putText(
            display,
            names[i],
            autoQuad[i] + cv::Point2f(10, -10),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 0),
            2
        );
    }

    // Secilen dortgeni ciz
    for(int i = 0; i < 4; i++)
    {
        cv::line(
            display,
            autoQuad[i],
            autoQuad[(i + 1) % 4],
            cv::Scalar(0, 255, 0),
            2
        );
    }

    cv::putText(
        display,
        "AUTO: 4 REFERANS VIDA BULUNDU",
        cv::Point(30, 80),
        cv::FONT_HERSHEY_SIMPLEX,
        0.8,
        cv::Scalar(0, 255, 0),
        2
    );
}
else
{
    std::string text =
        "AUTO ADAY: " +
        std::to_string(autoCandidates.size());

    cv::putText(
        display,
        text,
        cv::Point(30, 80),
        cv::FONT_HERSHEY_SIMPLEX,
        0.8,
        cv::Scalar(0, 0, 255),
        2
    );
}
bool allDetected = autoFound && autoQuad.size() == 4;

cv::Point2f detected[4];
if(allDetected)
{
    //--------------------------------------------------
    // Onceki tespitle ayni dikdortgen mi kontrol et
    //--------------------------------------------------

    bool consistent = true;
    const double MAX_FRAME_JUMP_PX = 12.0;

    // History varsa yeni dortgen onceki dortgenden
    // cok fazla sicramamis olmali.
    if(!history[0].empty())
    {
        for(int i = 0; i < 4; i++)
        {
            cv::Point2f previous = history[i].back();

            double jump =
                cv::norm(autoQuad[i] - previous);

            if(jump > MAX_FRAME_JUMP_PX)
            {
                consistent = false;
                break;
            }
        }
    }

    if(consistent)
    {
        //--------------------------------------------------
        // Dogru ve tutarli tespit
        //--------------------------------------------------

        missedFrames = 0;

        for(int i = 0; i < 4; i++)
        {
            detected[i] = autoQuad[i];

            history[i].push_back(detected[i]);

            if((int)history[i].size() > STABILITY_FRAMES)
                history[i].pop_front();
        }
    }
    else
    {
        //--------------------------------------------------
        // Dortgen baska bir yere sicradi.
        // History'ye EKLEME.
        //--------------------------------------------------

        allDetected = false;
        missedFrames++;

        std::cout
            << "[AUTO] Dortgen ani sicrama yapti - kare yok sayildi."
            << std::endl;
    }
}
else
{
    //--------------------------------------------------
    // Bu karede 4 uygun vida bulunamadi.
    // History'yi hemen silmiyoruz.
    //--------------------------------------------------

    missedFrames++;
}


//--------------------------------------------------
// Uzun sure referans bulunamazsa history'yi sifirla
//--------------------------------------------------

if(missedFrames > MAX_MISSED_FRAMES)
{
    for(int i = 0; i < 4; i++)
        history[i].clear();

    missedFrames = 0;

    std::cout
        << "[AUTO] Referans uzun sure kayboldu. "
        << "Stabilite sifirlandi."
        << std::endl;
}


//--------------------------------------------------
// STABILITE KONTROLU
//--------------------------------------------------

bool stable = true;
int minBufSize = STABILITY_FRAMES;

for(int i = 0; i < 4; i++)
{
    minBufSize =
        std::min(
            minBufSize,
            (int)history[i].size()
        );

    if((int)history[i].size() < STABILITY_FRAMES)
    {
        stable = false;
        continue;
    }

    double stdX =
        stddevOf(history[i], true);

    double stdY =
        stddevOf(history[i], false);

    if(stdX > STABILITY_TOL_PX ||
       stdY > STABILITY_TOL_PX)
    {
        stable = false;
    }
}

        std::string statusMsg = allDetected
    ? ("AUTO Stabilite: " +
       std::to_string(minBufSize) +
       "/" +
       std::to_string(STABILITY_FRAMES))
    : ("AUTO: uygun 4 referans vida bulunamadi - aday: " +
       std::to_string(autoCandidates.size()));
        cv::putText(display, statusMsg, cv::Point(20,35), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    allDetected ? cv::Scalar(0,255,255) : cv::Scalar(0,0,255), 2);

        cv::imshow("Screw Reference", display);

        if(stable)
        {
            std::vector<cv::Point2f> ordered = {
                meanOf(history[0]), meanOf(history[1]), meanOf(history[2]), meanOf(history[3])
            };

            std::string reason;
            if(!sanityCheckQuad(ordered, reason))
            {
                std::cerr << "\n[HATA] Otomatik tespit edilen vidalar gecersiz dortgen "
                          << "olusturuyor: " << reason
                          << "\n       ROI'leri kontrol edin ('c' ile yeniden kalibre edin)." << std::endl;
                for(int i = 0; i < 4; i++) history[i].clear();
            }
            else
            {
                finalScrewPoints = ordered;
                captured = true;
                break;
            }
        }

        int key = cv::waitKey(30);
        if(key == 27) { cap.release(); cv::destroyAllWindows(); return 0; }
        if(key == 'r' || key == 'R')
        {
            for(int i = 0; i < 4; i++) history[i].clear();
            std::cout << "[BILGI] Stabilite arabellegi sifirlandi." << std::endl;
        }
        
    }

    if(!captured) { cap.release(); cv::destroyAllWindows(); return -1; }

    std::vector<cv::Point2f> worldPoints = {
        {0.0f, 0.0f},
        {SCREW_WIDTH_MM, 0.0f},
        {SCREW_WIDTH_MM, SCREW_HEIGHT_MM},
        {0.0f, SCREW_HEIGHT_MM}
    };

    cv::Mat H = cv::findHomography(finalScrewPoints, worldPoints);
    if(H.empty()) { std::cerr << "Homography hesaplanamadi!" << std::endl; return -1; }

    double reprojErrMM = 0.0;
    bool valid = validateHomography(H, finalScrewPoints, worldPoints, reprojErrMM);

    std::cout << "\nVida Homography Matrix:\n" << H << std::endl;
    std::cout << "Reprojeksiyon hatasi: " << reprojErrMM << " mm" << std::endl;

    if(!valid || reprojErrMM > MAX_ACCEPTABLE_REPROJ_MM)
    {
        std::cerr << "[HATA] Reprojeksiyon hatasi (" << reprojErrMM << " mm) hedef "
                  << MAX_ACCEPTABLE_REPROJ_MM << " mm'nin uzerinde. KAYDEDILMEDI.\n"
                  << "       ROI'leri daha dar/net vida etrafina cizip 'c' ile yeniden kalibre edin."
                  << std::endl;
        cap.release();
        cv::destroyAllWindows();
        return -1;
    }

    cv::FileStorage fs("screw_homography.yml", cv::FileStorage::WRITE);
    if(fs.isOpened())
    {
        fs << "homography_matrix" << H;
        fs << "reference_width_mm"  << SCREW_WIDTH_MM;
        fs << "reference_height_mm" << SCREW_HEIGHT_MM;
        fs << "reprojection_error_mm" << reprojErrMM;
        fs.release();
        std::cout << "\nscrew_homography.yml OTOMATIK olarak kaydedildi (tiklama olmadan)." << std::endl;
    }

    while(true)
    {
        cv::Mat frame;
        cap >> frame;
        if(frame.empty()) break;

        frame = undistortFrame(frame, cameraMatrix, distCoeffs);
        cv::Mat display = frame.clone();

        for(int i = 0; i < 4; i++)
        {
            cv::circle(display, finalScrewPoints[i], 7, cv::Scalar(0,255,0), -1);
            cv::putText(display, names[i], finalScrewPoints[i] + cv::Point2f(10,-10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
            cv::line(display, finalScrewPoints[i], finalScrewPoints[(i+1)%4], cv::Scalar(255,0,0), 2);
        }

        cv::putText(display, "REFERENCE: 222.5 x 150.0 mm (OTOMATIK)", cv::Point(20,35),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);

        cv::imshow("Screw Reference", display);
        if(cv::waitKey(30) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}