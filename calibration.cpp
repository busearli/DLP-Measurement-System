#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

//--------------------------------------------------
// Ayarlar
//--------------------------------------------------
static const cv::Size BOARD_SIZE(7, 7);   // ic kose sayisi (ahsap satranc tahtasi: 8x8 kare -> 7x7 ic kose)
static const float SQUARE_SIZE = 32.0f;   // mm cinsinden bir karenin kenar uzunlugu (olculdu: 3.2cm)
static const int REQUIRED_FRAMES = 15;    // kalibrasyon icin toplanacak gecerli kare sayisi

//--------------------------------------------------
// Tek bir karede satranc tahtasi kosesi ara
//--------------------------------------------------
// FIX (KRITIK - gercek tahtada koseler bulunamiyor sorunu): cv::findChessboardCorners
// (klasik yontem) sentetik/dijital olarak uretilmis, keskin kenarli desenlerde iyi
// calisir ama GERCEK, fiziksel bir tahtada (ahsap dokusu, cizikler, parlama, hafif
// bulaniklik) genellikle basarisiz olur - tam olarak burada gozlenen davranis budur.
//
// OpenCV 4.0+ ile gelen cv::findChessboardCornersSB ("SB" = "sector based"),
// Duda ve digerlerinin makalesine dayanan cok daha guclu bir algoritma kullanir ve
// gercek dunya kosullarina (degisken aydinlatma, hafif bulaniklik, dokulu yuzeyler)
// karsi klasik yontemden BELIRGIN SEKILDE daha dayaniklidir. Ayrica kose noktalarini
// zaten subpixel hassasiyetinde dondurur - ayri bir cornerSubPix cagrisina ihtiyac
// duymaz.
//
// STRATEJI: once SB yontemini dene (cogu gercek tahta icin yeterli olacaktir).
// Basarisiz olursa, eski klasik yontem + cornerSubPix'e GERI DUS - boylece iki
// yontemden biri calisirsa tespit basarili olur.

bool findBoardCorners(const cv::Mat& frame, std::vector<cv::Point2f>& corners)
{
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 1) Modern yontem (SB) - gercek tahtalarda ilk tercih
    bool foundSB = cv::findChessboardCornersSB(
        gray,
        BOARD_SIZE,
        corners,
        cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ACCURACY);

    if(foundSB)
        return true;   // SB zaten subpixel hassasiyetinde donduruyor, ek islem gerekmez

    // 2) Fallback: klasik yontem + cornerSubPix
    bool foundClassic = cv::findChessboardCorners(
        gray,
        BOARD_SIZE,
        corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if(foundClassic)
    {
        cv::cornerSubPix(
            gray,
            corners,
            cv::Size(11, 11),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    }

    return foundClassic;
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

    // FIX (KRITIK - "kamera acilinca direk kapaniyor" sorunu): programin
    // sessizce/aniden kapanmasinin iki olasi nedeni var:
    //   1) cv::VideoCapture bazi backend'lerde (ozellikle macOS AVFoundation /
    //      Continuity Camera) ACILDIKTAN SONRA ilk bir kac karede BOS Mat
    //      donebilir. Eski kod, frame.empty() gorunce HEMEN break ediyordu;
    //      bu da dongunun 0 kare toplamadan (imagePoints.size() < 4) hemen
    //      "Yeterli kare toplanamadi" ile sonlanmasina, yani pencerenin
    //      acilir acilmaz kapanmasina yol aciyordu.
    //   2) findChessboardCornersSB (veya baska bir OpenCV cagrisi) bir
    //      cv::Exception firlatirsa, try/catch olmadigi icin program
    //      ANINDA ve HICBIR MESAJ VERMEDEN sonlanir - pencere de beraber kapanir.
    // Asagidaki try/catch + warm-up + "ardisik bos kare" tolerans mekanizmasi
    // bu iki senaryoyu da ele alip, sessiz kapanma yerine ACIKCA NE OLDUGUNU
    // konsola yazdiriyor.

    try
    {
        cv::VideoCapture cap(0);

        if(!cap.isOpened())
        {
            std::cout << "Kamera acilamadi!" << std::endl;
            return -1;
        }

        // Kamera "isinma" bekleyisi: bazi backend'ler ilk karelerde bos Mat
        // dondurur. Hemen basarisiz saymadan birkac deneme hakki taniyoruz.
        cv::Mat warmupFrame;
        int warmupAttempts = 0;
        const int MAX_WARMUP_ATTEMPTS = 60;   // ~60 * 30ms = ~1.8s

        while(warmupFrame.empty() && warmupAttempts < MAX_WARMUP_ATTEMPTS)
        {
            cap >> warmupFrame;
            warmupAttempts++;

            if(warmupFrame.empty())
                cv::waitKey(30);
        }

        if(warmupFrame.empty())
        {
            std::cout << "[HATA] Kameradan " << MAX_WARMUP_ATTEMPTS
                      << " denemede goruntu alinamadi. Kamera baska bir uygulama "
                      << "tarafindan kullaniliyor olabilir, yanlis kamera secilmis "
                      << "olabilir (VideoCapture(0) makinenizdeki ilk kamerayi acar - "
                      << "Continuity Camera/iPhone kamerasi ise bu index degisebilir), "
                      << "veya sistem kamera izni verilmemis olabilir "
                      << "(Sistem Ayarlari > Gizlilik ve Guvenlik > Kamera)." << std::endl;
            return -1;
        }

        std::cout << "[BILGI] Kameradan goruntu alinmaya basladi ("
                  << warmupAttempts << ". denemede, cozunurluk: "
                  << warmupFrame.cols << "x" << warmupFrame.rows << ")." << std::endl;

        std::vector<std::vector<cv::Point2f>> imagePoints;
        cv::Size imageSize;

        int consecutiveEmptyFrames = 0;
        const int MAX_CONSECUTIVE_EMPTY = 60;   // ~1.8s ardisik bos kareye tolerans

        while(static_cast<int>(imagePoints.size()) < REQUIRED_FRAMES)
        {
            cv::Mat frame;
            cap >> frame;

            if(frame.empty())
            {
                // FIX: eskiden burada HEMEN break ediliyordu. Artik gecici
                // (transient) bos kareler tolere ediliyor; sadece UZUN SURELI
                // (MAX_CONSECUTIVE_EMPTY kareyi asan) bir kesinti gercek bir
                // kopma/hata sayiliyor.
                consecutiveEmptyFrames++;

                if(consecutiveEmptyFrames > MAX_CONSECUTIVE_EMPTY)
                {
                    std::cout << "[HATA] Kameradan ardisik " << MAX_CONSECUTIVE_EMPTY
                              << " karede goruntu alinamadi, kamera baglantisi "
                              << "kesilmis olabilir. Program sonlandiriliyor." << std::endl;
                    break;
                }

                cv::waitKey(30);
                continue;
            }

            consecutiveEmptyFrames = 0;

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

            cv::putText(
                display,
                found ? "Kose durumu: BULUNDU (SPACE ile kaydet)" : "Kose durumu: bulunamadi",
                cv::Point(20, 60),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
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
    catch(const cv::Exception& e)
    {
        // FIX: eskiden yakalanmayan bir cv::Exception programi SESSIZCE
        // sonlandirip pencereyi kapatiyordu - operator hicbir sey gormeden
        // "direk kapaniyor" hissi yasiyordu. Artik en azindan hata mesaji
        // konsola yaziliyor.
        std::cout << "\n[HATA] OpenCV istisnasi yakalandi: " << e.what() << std::endl;
        return -1;
    }
    catch(const std::exception& e)
    {
        std::cout << "\n[HATA] Beklenmeyen istisna: " << e.what() << std::endl;
        return -1;
    }
}