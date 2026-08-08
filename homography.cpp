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

    cv::namedWindow("Homography");
    cv::setMouseCallback("Homography", onMouse);

    while(static_cast<int>(g_clickedPoints.size()) < 4)
    {
        cv::Mat frame = captureFrame(cap);

        if(frame.empty())
            break;

        for(size_t i = 0; i < g_clickedPoints.size(); i++)
        {
            cv::circle(frame, g_clickedPoints[i], 6, cv::Scalar(0, 0, 255), -1);
            cv::putText(frame, std::to_string(i + 1),
                        g_clickedPoints[i] + cv::Point2f(10, 0),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }

        cv::putText(frame, "Nokta: " + std::to_string(g_clickedPoints.size()) + "/4",
                    cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

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

    std::cout << "\nHomography Matrix:\n" << H << std::endl;

    saveHomography(H);

    cap.release();
    cv::destroyAllWindows();

    return 0;
}