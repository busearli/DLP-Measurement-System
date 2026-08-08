#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <vector>
#include <cmath>

//--------------------------------------------------
// DLP Model Listesi (SRS madde 8)
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

    if(choice < 1 || choice > static_cast<int>(g_dlpModels.size()))
    {
        std::cout << "Gecersiz secim, varsayilan olarak ilk model kullanilacak." << std::endl;
        choice = 1;
    }

    return g_dlpModels[choice - 1];
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
// Detect Bright Region (DLP projeksiyon deseni icin)
//--------------------------------------------------
// Renk yerine parlaklik kontrasti kullanir. Gri seviyeye cevirir,
// gurultuyu azaltmak icin hafif blur uygular, sonra trackbar ile
// ayarlanan esik degerine gore parlak bolgeyi ayirir.

int g_thresholdValue = 150; // trackbar ile canli ayarlanacak

cv::Mat detectBrightRegion(const cv::Mat& image)
{
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

    cv::Mat mask;
    cv::threshold(
        blurred,
        mask,
        g_thresholdValue,
        255,
        cv::THRESH_BINARY);

    return mask;
}

//--------------------------------------------------
// Clean Mask
//--------------------------------------------------

cv::Mat cleanMask(const cv::Mat& mask)
{
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9,9));
    cv::Mat opened, closed;

    cv::morphologyEx(mask, opened, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(opened, closed, cv::MORPH_CLOSE, kernel);

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
// Estimate Pose (Pitch / Roll) via solvePnP
//--------------------------------------------------
// imageCorners sirasi: rotated.points() ciktisi (4 nokta)
// realWidth/realHeight: secilen DLP modelinin gercek boyutu (mm)
// NOT: Bu ilk-versiyon; gercek DLP deseni entegre edildiginde
// objectPoints sirasi ile imageCorners sirasinin eslesmesi
// yeniden dogrulanmali (kose siralama tutarliligi kritik).

bool estimatePose(
    const cv::Point2f imageCorners[4],
    float realWidth,
    float realHeight,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    double& pitchDeg,
    double& rollDeg)
{
    if(cameraMatrix.empty() || distCoeffs.empty())
        return false;

    std::vector<cv::Point3f> objectPoints = {
        cv::Point3f(0.0f, 0.0f, 0.0f),
        cv::Point3f(realWidth, 0.0f, 0.0f),
        cv::Point3f(realWidth, realHeight, 0.0f),
        cv::Point3f(0.0f, realHeight, 0.0f)
    };

    std::vector<cv::Point2f> imgPts(imageCorners, imageCorners + 4);

    cv::Mat rvec, tvec;

    bool ok = cv::solvePnP(
        objectPoints,
        imgPts,
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

void measureObject(
    const std::vector<std::vector<cv::Point>>& contours,
    int maxIndex,
    const cv::Mat& homographyMatrix,
    bool homographyLoaded,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    bool calibrationLoaded,
    const DLPModel& selectedModel)
{
    if(maxIndex == -1)
        return;

    cv::Rect box = cv::boundingRect(contours[maxIndex]);
    cv::RotatedRect rotated = cv::minAreaRect(contours[maxIndex]);

    cv::Point2f imageCorners[4];
    rotated.points(imageCorners);

    std::vector<cv::Point2f> imagePoints(imageCorners, imageCorners + 4);
    std::vector<cv::Point2f> worldPoints;

    int centerX = box.x + box.width / 2;
    int centerY = box.y + box.height / 2;

    double width  = rotated.size.width;
    double height = rotated.size.height;

    double widthMM  = 0.0;
    double heightMM = 0.0;
    cv::Point2f centerMM(0.0f, 0.0f);

    if(homographyLoaded && !homographyMatrix.empty())
    {
        cv::perspectiveTransform(imagePoints, worldPoints, homographyMatrix);

        if(worldPoints.size() == 4)
        {
            double d01 = cv::norm(worldPoints[0] - worldPoints[1]);
            double d12 = cv::norm(worldPoints[1] - worldPoints[2]);
            double d23 = cv::norm(worldPoints[2] - worldPoints[3]);
            double d30 = cv::norm(worldPoints[3] - worldPoints[0]);

            double sideA = (d01 + d23) / 2.0;
            double sideB = (d12 + d30) / 2.0;

            widthMM  = std::max(sideA, sideB);
            heightMM = std::min(sideA, sideB);
        }

        std::vector<cv::Point2f> centerPx = { cv::Point2f((float)centerX, (float)centerY) };
        std::vector<cv::Point2f> centerWorld;
        cv::perspectiveTransform(centerPx, centerWorld, homographyMatrix);

        if(!centerWorld.empty())
            centerMM = centerWorld[0];
    }

    double angle = rotated.angle;
    double area  = cv::contourArea(contours[maxIndex]);

    std::cout << "\n=====================\n";
    std::cout << "Area : " << area << std::endl;
    std::cout << "Width (pixel) : " << width << std::endl;
    std::cout << "Height (pixel) : " << height << std::endl;

    if(homographyLoaded && !homographyMatrix.empty())
    {
        std::cout << "Width (mm) : " << widthMM << std::endl;
        std::cout << "Height (mm) : " << heightMM << std::endl;
        std::cout << "Center (mm) : (" << centerMM.x << ", " << centerMM.y << ")" << std::endl;
    }
    else
    {
        std::cout << "[UYARI] Homografi yuklenmedi, mm degerleri hesaplanamadi." << std::endl;
    }

    std::cout << "Angle : " << angle << std::endl;

    //------------------------------------------
    // Pitch / Roll (solvePnP)
    //------------------------------------------

    if(calibrationLoaded)
    {
        double pitchDeg = 0.0;
        double rollDeg  = 0.0;

        bool poseOk = estimatePose(
            imageCorners,
            selectedModel.widthMM,
            selectedModel.heightMM,
            cameraMatrix,
            distCoeffs,
            pitchDeg,
            rollDeg);

        if(poseOk)
        {
            std::cout << "Pitch : " << pitchDeg << " derece" << std::endl;
            std::cout << "Roll  : " << rollDeg  << " derece" << std::endl;
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
}

//--------------------------------------------------
// Draw Result
//--------------------------------------------------

cv::Mat drawResult(
    const cv::Mat& image,
    const std::vector<std::vector<cv::Point>>& contours,
    int maxIndex)
{
    cv::Mat result = image.clone();

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

    cv::Point center(box.x + box.width / 2, box.y + box.height / 2);
    cv::circle(result, center, 6, cv::Scalar(0,0,255), -1);

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

int main()
{
    cv::VideoCapture cap(0);

    if(!cap.isOpened())
    {
        std::cout << "Kamera acilamadi!" << std::endl;
        return -1;
    }

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

    cv::Mat homographyMatrix;
    bool homographyLoaded =
        loadHomography("homography.yml", homographyMatrix);

    if(!homographyLoaded)
    {
        std::cout << "[UYARI] Homografi yok, mm olcumu yapilamayacak. "
                  << "Once ./Homography programini calistirin." << std::endl;
    }

    DLPModel selectedModel = selectDLPModel();

    std::cout << "\nSecilen model: " << selectedModel.name << std::endl;
    std::cout << "Baslatiliyor...\n" << std::endl;

    cv::namedWindow("Mask (debug)");
    cv::createTrackbar("Threshold", "Mask (debug)", &g_thresholdValue, 255);
    cv::moveWindow("Mask (debug)", 900, 50);

    while(true)
    {
        cv::Mat frame = captureFrame(cap);

        if(frame.empty())
            break;

        frame = undistortFrame(frame, cameraMatrix, distCoeffs);

        cv::Mat mask = detectBrightRegion(frame);
        cv::Mat clean = cleanMask(mask);

        cv::imshow("Mask (debug)", clean);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        int maxIndex = findLargestContour(clean, contours, hierarchy);

        measureObject(
            contours,
            maxIndex,
            homographyMatrix,
            homographyLoaded,
            cameraMatrix,
            distCoeffs,
            calibrationLoaded,
            selectedModel);

        cv::Mat result = drawResult(frame, contours, maxIndex);

        cv::imshow("Machine Vision", result);

        if(cv::waitKey(30) == 27)
            break;
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}

