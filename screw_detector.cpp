#include "screw_detector.hpp"
#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>
#include <algorithm>
#include <numeric>

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
//
// Alt bolgede aydinlatma farkli oldugu icin
// metal halka her zaman merkezden daha parlak
// gorunmeyebilir.
//
// Bu nedenle:
// 1) Normal appearanceScore filtresi korunur.
// 2) Merkezi belirgin sekilde koyu olan,
//    Hough tarafindan guclu sekilde bulunan
//    daireleri de kaybetmeyiz.
//--------------------------------------------------

bool darkCenterCandidate =
    meanIntensity < 75.0 &&
    radius >= 10.0f &&
    radius <= 25.0f;

bool accepted =
    appearanceScore >= 3 ||
    darkCenterCandidate;

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
// REFERANS DORTGEN BOYUT KONTROLU
//--------------------------------------------------
//
// Gercek 4 referans vida kasanin genis bir bolgesini
// kaplar. Kucuk/ic dortgenleri daha burada eliyoruz.
//--------------------------------------------------

double quadWidth =
    (topWidth + bottomWidth) / 2.0;

double quadHeight =
    (leftHeight + rightHeight) / 2.0;

if(quadWidth < 700.0 || quadHeight < 350.0)
    continue;

//--------------------------------------------------
// ALAN
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

// Buyuk referans dortgenini belirgin sekilde tercih et.
score -= std::min(area / 500000.0, 0.60);

                     std::cout
    << "[QUAD] area=" << area
    << " ratioErr=" << ratioError
    << " widthAsym=" << widthAsym
    << " heightAsym=" << heightAsym
    << " diagAsym=" << diagAsym
    << " score=" << score
    << std::endl;

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
        << "[QUAD] "
        << "TL=(" << bestQuad[0].x << "," << bestQuad[0].y << ") "
        << "TR=(" << bestQuad[1].x << "," << bestQuad[1].y << ") "
        << "BR=(" << bestQuad[2].x << "," << bestQuad[2].y << ") "
        << "BL=(" << bestQuad[3].x << "," << bestQuad[3].y << ")"
        << std::endl;

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
