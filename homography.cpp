#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

//--------------------------------------------------
// Reference Size (A4)
//--------------------------------------------------

const float A4_WIDTH = 210.0f;
const float A4_HEIGHT = 297.0f;

//--------------------------------------------------
// Mouse ile tiklanan noktalar
//--------------------------------------------------
// Tiklama sirasi: sol-ust, sag-ust, sag-alt, sol-alt

std::vector<cv::Point2f> g_clickedPoints;

void onMouse(int event, int x, int y, int, void*)
{
    if(event == cv::EVENT_LBUTTONDOWN && g_clickedPoints.size() < 4)
    {
        g_clickedPoints.emplace_back(static_cast<float>(x), static_cast<float>(y));
        std::cout << "Nokta " << g_clickedPoints.size() << "/4 alindi: ("
                  << x << ", " << y << ")" << std::endl;
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
// Load Calibration
//--------------------------------------------------
// FIX (KRITIK TUTARLILIK HATASI): main.cpp, homography.yml'i UNDISTORT
// EDILMIS piksel koordinatlarina uyguluyor (calibrationLoaded ise her
// karede undistortFrame() cagriliyor). Ama bu programda (Homography)
// eskiden calibration.yml HIC yuklenmiyordu ve kullanici HAM (distorsiyonlu)
// goruntu uzerinde tikliyordu. Sonuc: homografi distorsiyonlu piksellerden
// hesaplaniyor, runtime'da ise undistort edilmis piksellere uygulaniyordu -
// bu da ozellikle goruntu kenarlarina yakin bolgelerde sistematik mm hatasi
// yaratiyordu. COZUM: bu program da calibration.yml'i yukluyor ve
// kullaniciya gosterdigi/tiklattigi goruntuyu undistort ediyor - boylece
// iki program da AYNI (undistorted) koordinat uzayinda calisiyor.

bool loadCalibration(
    const std::string& filename,
    cv::Mat& cameraMatrix,
    cv::Mat& distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if(!fs.isOpened())
        return false;

    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    return !cameraMatrix.empty() && !distCoeffs.empty();
}

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
// Compute Homography (A4 koseleri -> gercek mm)
//--------------------------------------------------

cv::Mat computeHomography(const std::vector<cv::Point2f>& imagePoints)
{
    std::vector<cv::Point2f> worldPoints = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(A4_WIDTH, 0.0f),
        cv::Point2f(A4_WIDTH, A4_HEIGHT),
        cv::Point2f(0.0f, A4_HEIGHT)
    };

    return cv::findHomography(imagePoints, worldPoints);
}

//--------------------------------------------------
// Validate Homography (FIX: eskiden hic dogrulanmiyordu)
//--------------------------------------------------
// findHomography, 4 nokta neredeyse dogrusal (dejenere) ise bos veya
// anlamsiz bir matris donebilir. Ayrica hesaplanan H'yi tekrar
// imagePoints uzerine uygulayip worldPoints ile karsilastirarak bir
// reprojeksiyon hatasi (mm) raporluyoruz - bu, operatorun tiklama
// hassasiyetini (SRS madde 11: "olcum tekrarlanabilirligi") dogrudan
// gosteren basit bir gostergedir.

bool validateHomography(
    const cv::Mat& H,
    const std::vector<cv::Point2f>& imagePoints,
    double& outReprojectionErrorMM)
{
    outReprojectionErrorMM = 0.0;

    if(H.empty() || H.rows != 3 || H.cols != 3)
        return false;

    std::vector<cv::Point2f> worldPoints = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(A4_WIDTH, 0.0f),
        cv::Point2f(A4_WIDTH, A4_HEIGHT),
        cv::Point2f(0.0f, A4_HEIGHT)
    };

    std::vector<cv::Point2f> reprojected;
    cv::perspectiveTransform(imagePoints, reprojected, H);

    if(reprojected.size() != worldPoints.size())
        return false;

    double sumErr = 0.0;

    for(size_t i = 0; i < reprojected.size(); i++)
        sumErr += cv::norm(reprojected[i] - worldPoints[i]);

    outReprojectionErrorMM = sumErr / reprojected.size();

    return true;
}

//--------------------------------------------------
// Save Homography
//--------------------------------------------------

void saveHomography(const cv::Mat& H)
{
    cv::FileStorage fs("homography.yml", cv::FileStorage::WRITE);

    if(!fs.isOpened())
    {
        std::cout << "homography.yml olusturulamadi!" << std::endl;
        return;
    }

    fs << "homography_matrix" << H;
    fs.release();

    std::cout << "Homography kaydedildi." << std::endl;
}

//--------------------------------------------------
// MAIN
//--------------------------------------------------

int main()
{
    std::cout << "Homography Program (A4 kagit)" << std::endl;
    std::cout << "A4 kagidi (210x297mm) olcum yuzeyine yerlestirin." << std::endl;
    std::cout << "Kose sirasi: sol-ust, sag-ust, sag-alt, sol-alt." << std::endl;
    std::cout << "Her koseye fare ile tiklayin. Iptal icin ESC." << std::endl;

    cv::VideoCapture cap(0);

    if(!cap.isOpened())
    {
        std::cout << "Kamera acilamadi!" << std::endl;
        return -1;
    }

    // FIX: calibration.yml varsa yukle ve goruntuyu undistort et - bkz.
    // loadCalibration() yorumu (kritik tutarlilik duzeltmesi).
    cv::Mat cameraMatrix, distCoeffs;
    bool calibrationLoaded = loadCalibration("calibration.yml", cameraMatrix, distCoeffs);

    if(calibrationLoaded)
    {
        std::cout << "Calibration yuklendi, goruntu undistort edilecek." << std::endl;
    }
    else
    {
        std::cout << "[UYARI] calibration.yml bulunamadi/eksik. Once Calibration programini "
                  << "calistirmaniz onerilir; bu haliyle homografi HAM (distorsiyonlu) goruntu "
                  << "uzerinden hesaplanacak ve main.cpp'nin undistort edilmis goruntusuyle "
                  << "TUTARSIZ olabilir." << std::endl;
    }

    cv::namedWindow("Homography");
    cv::setMouseCallback("Homography", onMouse);

    while(static_cast<int>(g_clickedPoints.size()) < 4)
    {
        cv::Mat frame = captureFrame(cap);

        if(frame.empty())
            break;

        if(calibrationLoaded)
        {
            frame = undistortFrame(frame, cameraMatrix, distCoeffs);
        }

        for(size_t i = 0; i < g_clickedPoints.size(); i++)
        {
            cv::circle(frame, g_clickedPoints[i], 6, cv::Scalar(0, 0, 255), -1);
            cv::putText(frame, std::to_string(i + 1),
                        g_clickedPoints[i] + cv::Point2f(10, 0),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }

        cv::putText(frame, "Nokta: " + std::to_string(g_clickedPoints.size()) + "/4",
                    cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        if(!calibrationLoaded)
        {
            cv::putText(frame, "UYARI: kalibrasyon yok (distorsiyon duzeltilmedi)",
                        cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 140, 255), 2);
        }

        cv::imshow("Homography", frame);

        if(cv::waitKey(30) == 27)
        {
            std::cout << "Iptal edildi." << std::endl;
            cap.release();
            cv::destroyAllWindows();
            return 0;
        }
    }

    cv::Mat H = computeHomography(g_clickedPoints);

    double reprojErrMM = 0.0;
    bool valid = validateHomography(H, g_clickedPoints, reprojErrMM);

    if(!valid)
    {
        // FIX: eskiden bu durum hic kontrol edilmiyor, bozuk/bos matris
        // sessizce kaydediliyordu.
        std::cout << "[HATA] Homografi hesaplanamadi (noktalar dejenere/dogrusal olabilir). "
                  << "Kaydedilmedi. Lutfen programi tekrar calistirip 4 koseyi daha "
                  << "belirgin/farkli konumlarda tiklayin." << std::endl;
        cap.release();
        cv::destroyAllWindows();
        return -1;
    }

    std::cout << "\nHomography Matrix:\n" << H << std::endl;
    std::cout << "Reprojeksiyon hatasi (kose basina ortalama): " << reprojErrMM << " mm" << std::endl;

    if(reprojErrMM > 2.0)
    {
        std::cout << "[UYARI] Reprojeksiyon hatasi yuksek (>2mm). Tiklama hassasiyeti dusuk "
                  << "olabilir; A4 kagidinin koselerini daha net gorunur hale getirip "
                  << "(iyi aydinlatma, dik acidan bakis) tekrar deneyin." << std::endl;
    }

    saveHomography(H);

    cap.release();
    cv::destroyAllWindows();

    return 0;
}