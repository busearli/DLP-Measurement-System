#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

//--------------------------------------------------
// Ayarlar
//--------------------------------------------------
static const cv::Size BOARD_SIZE(9, 6);   // ic kose sayisi (satranc tahtasi 10x7 karelik ise 9x6 kose)
static const float SQUARE_SIZE = 20.0f;   // mm cinsinden bir karenin kenar uzunlugu
static const int REQUIRED_FRAMES = 15;    // kalibrasyon icin toplanacak gecerli kare sayisi

//--------------------------------------------------
// Tek bir karede satranc tahtasi kosesi ara
//--------------------------------------------------
bool findBoardCorners(const cv::Mat& frame, std::vector<cv::Point2f>& corners)
{
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    bool found = cv::findChessboardCorners(
        gray,
        BOARD_SIZE,
        corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if(found)
    {
        cv::cornerSubPix(
            gray,
            corners,
            cv::Size(11, 11),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    }

    return found;
}

//--------------------------------------------------
// Tahtanin dunya koordinatlarindaki (3D) nokta seti
//--------------------------------------------------
std::vector<cv::Point3f> makeObjectPoints()
{
    std::vector<cv::Point3f> objectPoints;

    for(int i = 0; i < BOARD_SIZE.height; i++)
    {
        for(int j = 0; j < BOARD_SIZE.width; j++)
        {
            objectPoints.emplace_back(j * SQUARE_SIZE, i * SQUARE_SIZE, 0.0f);
        }
    }

    return objectPoints;
}

//--------------------------------------------------
// Kalibrasyonu calistir
//--------------------------------------------------
double runCalibration(
    const std::vector<std::vector<cv::Point2f>>& imagePoints,
    const cv::Size& imageSize,
    cv::Mat& cameraMatrix,
    cv::Mat& distCoeffs)
{
    std::vector<std::vector<cv::Point3f>> objectPoints(
        imagePoints.size(),
        makeObjectPoints());

    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(
        objectPoints,
        imagePoints,
        imageSize,
        cameraMatrix,
        distCoeffs,
        rvecs,
        tvecs);

    return rms;
}

//--------------------------------------------------
// Sonuclari dosyaya kaydet
//--------------------------------------------------
void saveCalibration(
    const std::string& path,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    const cv::Size& imageSize,
    double rms)
{
    cv::FileStorage fs(path, cv::FileStorage::WRITE);

    if(!fs.isOpened())
    {
        std::cout << "Kalibrasyon dosyasi acilamadi: " << path << std::endl;
        return;
    }

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "reprojection_error" << rms;

    fs.release();

    std::cout << "Kalibrasyon kaydedildi: " << path << std::endl;
}

//--------------------------------------------------
// MAIN
//--------------------------------------------------
int main()
{
    std::cout << "Calibration Program" << std::endl;
    std::cout << "Tahtayi kameraya farkli acilardan gosterin. "
              << REQUIRED_FRAMES << " gecerli kare toplanacak." << std::endl;
    std::cout << "Gecerli bir kose bulundugunda otomatik kaydedilir. "
              << "Cikmak icin ESC tusuna basin." << std::endl;

    cv::VideoCapture cap(0);

    if(!cap.isOpened())
    {
        std::cout << "Kamera acilamadi!" << std::endl;
        return -1;
    }

    std::vector<std::vector<cv::Point2f>> imagePoints;
    cv::Size imageSize;

    while(static_cast<int>(imagePoints.size()) < REQUIRED_FRAMES)
    {
        cv::Mat frame;
        cap >> frame;

        if(frame.empty())
            break;

        imageSize = frame.size();

        std::vector<cv::Point2f> corners;
        bool found = findBoardCorners(frame, corners);

        cv::Mat display = frame.clone();

        if(found)
        {
            cv::drawChessboardCorners(display, BOARD_SIZE, corners, found);
        }

        std::string status =
            "Toplanan: " + std::to_string(imagePoints.size()) + "/" + std::to_string(REQUIRED_FRAMES);

        cv::putText(
            display,
            status,
            cv::Point(20, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            0.8,
            cv::Scalar(0, 255, 0),
            2);

        cv::imshow("Calibration", display);

        int key = cv::waitKey(30);

        if(key == 27)
        {
            break;
        }
        else if(key == 32 && found) // SPACE: bu kareyi kaydet
        {
            imagePoints.push_back(corners);
            std::cout << "Kare eklendi: " << imagePoints.size() << "/" << REQUIRED_FRAMES << std::endl;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    if(imagePoints.size() < 4)
    {
        std::cout << "Yeterli kare toplanamadi, kalibrasyon iptal edildi." << std::endl;
        return -1;
    }

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    double rms = runCalibration(imagePoints, imageSize, cameraMatrix, distCoeffs);

    std::cout << "\n=====================\n";
    std::cout << "RMS Reprojection Error: " << rms << std::endl;
    std::cout << "Camera Matrix:\n" << cameraMatrix << std::endl;
    std::cout << "Distortion Coefficients:\n" << distCoeffs << std::endl;

    saveCalibration("calibration.yml", cameraMatrix, distCoeffs, imageSize, rms);

    return 0;
}
