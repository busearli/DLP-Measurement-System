#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFrame>
#include <QTimer>
#include <QImage>
#include <QPixmap>
#include <opencv2/opencv.hpp>
#include "screw_detector.hpp"
#include "measurement_engine.hpp"


//==================================================
// KAMERA KALIBRASYONU YUKLE
//==================================================

bool loadCalibrationGUI(
    const std::string& filename,
    cv::Mat& cameraMatrix,
    cv::Mat& distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if(!fs.isOpened())
    {
        std::cout << "[GUI] calibration.yml bulunamadi." << std::endl;
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    if(cameraMatrix.empty() || distCoeffs.empty())
    {
        std::cout << "[GUI] Kalibrasyon dosyasi eksik veya bozuk." << std::endl;
        return false;
    }

    std::cout << "[GUI] Kamera kalibrasyonu yuklendi." << std::endl;
    return true;
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QMainWindow window;
    window.setWindowTitle("DLP Projeksiyon Olcum ve Kalibrasyon Sistemi");
    window.resize(1200, 750);

    QWidget *central = new QWidget;
    QVBoxLayout *mainLayout = new QVBoxLayout(central);

    // Baslik
    QLabel *title = new QLabel("DLP PROJEKSIYON OLCUM VE KALIBRASYON SISTEMI");
    title->setAlignment(Qt::AlignCenter);
    title->setStyleSheet(
        "font-size: 24px;"
        "font-weight: bold;"
        "padding: 15px;"
    );

    mainLayout->addWidget(title);

    // Orta kisim
    QHBoxLayout *contentLayout = new QHBoxLayout;

    // Kamera alani
    QLabel *cameraView = new QLabel("Kamera goruntusu burada gosterilecek");
    cameraView->setAlignment(Qt::AlignCenter);
    cameraView->setMinimumSize(800, 500);
    cameraView->setStyleSheet(
        "background-color: #202020;"
        "color: white;"
        "font-size: 18px;"
        "border-radius: 8px;"
    );

    contentLayout->addWidget(cameraView, 3);

    // Sag panel
    QFrame *sidePanel = new QFrame;
    sidePanel->setMinimumWidth(300);

    QVBoxLayout *sideLayout = new QVBoxLayout(sidePanel);

    QLabel *statusTitle = new QLabel("SISTEM DURUMU");
    statusTitle->setStyleSheet(
        "font-size: 20px;"
        "font-weight: bold;"
    );

    QLabel *cameraStatus = new QLabel("○ Kamera");
    QLabel *calibrationStatus = new QLabel("○ Kamera Kalibrasyonu");
    QLabel *screwStatus = new QLabel("○ Vida Referansi");
    QLabel *projectionStatus = new QLabel("○ Projeksiyon Tespiti");

    QLabel *measurementTitle = new QLabel("\nOLCUMLER");
    measurementTitle->setStyleSheet(
        "font-size: 20px;"
        "font-weight: bold;"
    );

    QLabel *widthLabel = new QLabel("Genislik : -- mm");
    QLabel *heightLabel = new QLabel("Yukseklik : -- mm");
    QLabel *angleLabel = new QLabel("Aci : -- derece");

    sideLayout->addWidget(statusTitle);
    sideLayout->addWidget(cameraStatus);
    sideLayout->addWidget(calibrationStatus);
    sideLayout->addWidget(screwStatus);
    sideLayout->addWidget(projectionStatus);

    sideLayout->addWidget(measurementTitle);
    sideLayout->addWidget(widthLabel);
    sideLayout->addWidget(heightLabel);
    sideLayout->addWidget(angleLabel);

    sideLayout->addStretch();

    contentLayout->addWidget(sidePanel, 1);

    mainLayout->addLayout(contentLayout);

    // Yonlendirme
    QLabel *direction = new QLabel("SISTEM BASLATILMADI");
    direction->setAlignment(Qt::AlignCenter);
    direction->setMinimumHeight(80);
    direction->setStyleSheet(
        "font-size: 24px;"
        "font-weight: bold;"
        "background-color: #eeeeee;"
        "border-radius: 8px;"
    );

    mainLayout->addWidget(direction);

    // Butonlar
    QHBoxLayout *buttonLayout = new QHBoxLayout;

    QPushButton *startButton = new QPushButton("SISTEMI BASLAT");
    QPushButton *stopButton = new QPushButton("DURDUR");

    startButton->setMinimumHeight(55);
    stopButton->setMinimumHeight(55);

    startButton->setStyleSheet(
        "font-size: 18px;"
        "font-weight: bold;"
    );

    stopButton->setStyleSheet(
        "font-size: 18px;"
    );

    buttonLayout->addWidget(startButton);
    buttonLayout->addWidget(stopButton);

    mainLayout->addLayout(buttonLayout);

    //==================================================
// CANLI KAMERA
//==================================================

cv::VideoCapture *cap = new cv::VideoCapture();
QTimer *cameraTimer = new QTimer(&window);

// Kamera kalibrasyonu
cv::Mat *cameraMatrix = new cv::Mat();
cv::Mat *distCoeffs = new cv::Mat();
bool *calibrationLoaded = new bool(false);

QObject::connect(startButton, &QPushButton::clicked, [=]() {

    // Kamera zaten aciksa tekrar acma
    if(cap->isOpened())
        return;

    direction->setText("KAMERA BASLATILIYOR...");
    cameraStatus->setText("○ Kamera kontrol ediliyor...");

    if(!cap->open(0))
    {
        cameraStatus->setText("✗ Kamera acilamadi");
        direction->setText("KAMERA BULUNAMADI");
        return;
    }

    cameraStatus->setText("✓ Kamera bagli");

    //--------------------------------------------------
    // Kamera kalibrasyonunu yukle
    //--------------------------------------------------

    *calibrationLoaded =
        loadCalibrationGUI(
            "calibration.yml",
            *cameraMatrix,
            *distCoeffs
        );

    if(*calibrationLoaded)
        calibrationStatus->setText("✓ Kamera Kalibrasyonu");
    else
        calibrationStatus->setText("✗ Kamera Kalibrasyonu");

    direction->setText("KAMERA AKTIF - VIDA REFERANSI BEKLENIYOR");

    cameraTimer->start(30);
});

int *goodScrewFrames = new int(0);

int *missedScrewFrames = new int(0);

std::vector<cv::Point2f> *previousScrewQuad =
    new std::vector<cv::Point2f>();

std::vector<cv::Point2f> *accumulatedScrewQuad =
    new std::vector<cv::Point2f>(
        4,
        cv::Point2f(0.0f, 0.0f)
    );

bool *screwReferenceLocked = new bool(false);

std::vector<cv::Point2f> *lockedScrewQuad =
    new std::vector<cv::Point2f>();

// main.cpp ile AYNI degerler
const int REQUIRED_SCREW_FRAMES = 8;
const double MAX_SCREW_JUMP_PX = 15.0;
const int MAX_MISSED_SCREW_FRAMES = 3;
//==================================================
// ASAMA 2 - HOMOGRAFI / PROJEKSIYON OLCUM DURUMU
//==================================================

cv::Mat *homographyMatrix = new cv::Mat();
bool *homographyReady = new bool(false);
bool *stage2Active = new bool(false);
MeasurementHistory *measurementHistory =
    new MeasurementHistory(7, 3);
    MeasurementData *lastValidMeasurement =
    new MeasurementData();

bool *hasLastValidMeasurement =
    new bool(false);

int *lostProjectionFrames =
    new int(0);

const int MAX_LOST_PROJECTION_FRAMES = 30;

//--------------------------------------------------
// YONLENDIRME KILIDI
//--------------------------------------------------

// Ekranda gosterilen komut
AlignmentStatus *lockedAlignment =
    new AlignmentStatus(AlignmentStatus::NO_DATA);

// Komut verildigi andaki projeksiyon konumu
cv::Point2d *alignmentStartCenter =
    new cv::Point2d(0.0, 0.0);

double *alignmentStartAngle =
    new double(0.0);

// Su anda bir komut kilitli mi?
bool *alignmentLocked =
    new bool(false);

// Kullanici projeksiyonu gercekten hareket ettirdi mi?
// 2 mm'den veya 0.5 dereceden buyuk degisim hareket sayilir.
const double COMMAND_MOVE_THRESHOLD_MM = 2.0;
const double COMMAND_ANGLE_THRESHOLD_DEG = 0.5;

QObject::connect(cameraTimer, &QTimer::timeout, [=]() {
    static int frameCounter = 0;
    std::cout << "[FRAME] " << ++frameCounter << std::endl;
    if(!cap->isOpened())
        return;

    cv::Mat frame;
    (*cap) >> frame;

    if(frame.empty())
        return;

    //--------------------------------------------------
    // 1) Gri goruntu
    //--------------------------------------------------

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    //--------------------------------------------------
    // 2) Vida adaylarini bul
    //--------------------------------------------------

    cv::Mat display = frame.clone();

    std::vector<cv::Point2f> candidates =
        detectScrewCandidates(gray, display);

    std::vector<cv::Point2f> quad;

    bool found =
        selectBestScrewQuad(candidates, quad);

    bool accepted =
        found && quad.size() == 4;
        std::cout
        << "[GUI VIDA] candidates=" << candidates.size()
        << " found=" << found
        << " quad=" << quad.size()
        << std::endl;

    //--------------------------------------------------
    // 3) Geometri kontrolu
    //--------------------------------------------------

    if(accepted)
    {
        std::string reason;

       if(!sanityCheckQuad(quad, reason))
{
    std::cout
        << "[GUI VIDA] sanityCheck RED: "
        << reason
        << std::endl;

    accepted = false;
}
else
{
    std::cout
        << "[GUI VIDA] sanityCheck OK"
        << std::endl;
}
    }

    //--------------------------------------------------
    // 4) Onceki kareye gore ani sicrama kontrolu
    //--------------------------------------------------

    if(accepted && !previousScrewQuad->empty())
    {
        for(int i = 0; i < 4; i++)
        {
            double jump =
                cv::norm(quad[i] - (*previousScrewQuad)[i]);

            if(jump > MAX_SCREW_JUMP_PX)
            {
                accepted = false;
                break;
            }
        }
    }

    //--------------------------------------------------
// 5) Stabil vida referansi - main.cpp ile ayni mantik
//--------------------------------------------------

if(accepted)
{
    // Bu kare basarili
    *missedScrewFrames = 0;

    // Yeni bir 8-karelik toplama basliyorsa
    // accumulator'u temizle
    if(*goodScrewFrames == 0)
    {
        for(int i = 0; i < 4; ++i)
        {
            (*accumulatedScrewQuad)[i] =
                cv::Point2f(0.0f, 0.0f);
        }
    }

    //--------------------------------------------------
    // Bu karenin 4 vida noktasini toplama ekle
    //--------------------------------------------------

    for(int i = 0; i < 4; ++i)
    {
        (*accumulatedScrewQuad)[i] += quad[i];
    }

    *previousScrewQuad = quad;

    if(*goodScrewFrames < REQUIRED_SCREW_FRAMES)
    {
        (*goodScrewFrames)++;
    }

    //--------------------------------------------------
    // TL / TR / BR / BL ciz
    //--------------------------------------------------

    const char* names[4] = {
        "TL", "TR", "BR", "BL"
    };

    for(int i = 0; i < 4; ++i)
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
            names[i],
            quad[i] + cv::Point2f(10,-10),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0,255,0),
            2
        );

        cv::line(
            display,
            quad[i],
            quad[(i + 1) % 4],
            cv::Scalar(0,255,0),
            2
        );
    }
}
else if(!*screwReferenceLocked)
{
    //--------------------------------------------------
    // main.cpp gibi gecici kotu kareleri tolere et
    //--------------------------------------------------

    (*missedScrewFrames)++;

    std::cout
        << "[GUI VIDA] Gecici vida kaybi: "
        << *missedScrewFrames
        << "/"
        << MAX_MISSED_SCREW_FRAMES
        << std::endl;

    //--------------------------------------------------
    // Ancak 3'ten fazla arka arkaya kotu kare gelirse
    // stabiliteyi bastan baslat
    //--------------------------------------------------

    if(*missedScrewFrames > MAX_MISSED_SCREW_FRAMES)
    {
        std::cout
            << "[GUI VIDA] Vida referansi uzun sure kayip. "
            << "Stabilite yeniden baslatiliyor."
            << std::endl;

        *goodScrewFrames = 0;
        *missedScrewFrames = 0;

        previousScrewQuad->clear();

        for(int i = 0; i < 4; ++i)
        {
            (*accumulatedScrewQuad)[i] =
                cv::Point2f(0.0f, 0.0f);
        }
    }
}


    //--------------------------------------------------
    // 6) GUI durumunu guncelle
    //--------------------------------------------------

    if(*goodScrewFrames >= REQUIRED_SCREW_FRAMES)
{
    if(!*screwReferenceLocked)
    {
        //--------------------------------------------------
// main.cpp ile ayni:
// 8 basarili karenin ortalama vida konumlarini kullan
//--------------------------------------------------

lockedScrewQuad->resize(4);

for(int i = 0; i < 4; ++i)
{
    (*lockedScrewQuad)[i] =
        (*accumulatedScrewQuad)[i] /
        static_cast<float>(REQUIRED_SCREW_FRAMES);
}

// Ortalama referans artik kesin olarak kilitlendi
*screwReferenceLocked = true;

std::cout
    << "[GUI] Vida referansi "
    << REQUIRED_SCREW_FRAMES
    << " karenin ortalamasi ile kilitlendi."
    << std::endl;
        //--------------------------------------------------
        // VIDA PIXEL -> GERCEK DUNYA (mm) HOMOGRAFISI
        //--------------------------------------------------

        if(lockedScrewQuad->size() == 4)
        {
            std::vector<cv::Point2f> worldPoints = {
                {0.0f, 0.0f},
                {222.5f, 0.0f},
                {222.5f, 150.0f},
                {0.0f, 150.0f}
            };

            *homographyMatrix =
                cv::findHomography(
                    *lockedScrewQuad,
                    worldPoints
                );

            if(!homographyMatrix->empty())
            {
                *homographyReady = true;
                *stage2Active = true;

                std::cout
                    << "[GUI] Homografi olusturuldu."
                    << std::endl;

                std::cout
                    << "[GUI] ASAMA 2 baslatildi."
                    << std::endl;
            }
            else
            {
                *homographyReady = false;
                *stage2Active = false;

                std::cerr
                    << "[GUI] Homografi hesaplanamadi."
                    << std::endl;
            }
        }
    }

    screwStatus->setText("✓ Vida Referansi");

    if(*stage2Active)
    {
        direction->setText(
            "ASAMA 2 - PROJEKSIYON ARANIYOR..."
        );
    }
    else
    {
        direction->setText(
            "VIDA REFERANSI BULUNDU - HOMOGRAFI HATASI"
        );
    }
}
    else
    {
        screwStatus->setText(
            QString("○ Vida Referansi %1/%2")
                .arg(*goodScrewFrames)
                .arg(REQUIRED_SCREW_FRAMES)
        );

        direction->setText(
            "REFERANS VIDALAR ARANIYOR..."
        );
    }


    //--------------------------------------------------
    // ASAMA 2 - PROJEKSIYON TESPITI VE OLCUM
    //--------------------------------------------------

    if(*stage2Active && *homographyReady)
    {
        // Sistemin sabit DLP modeli
        DLPModel selectedModel = defaultDLPModel();

        //--------------------------------------------------
        // 1) Parlak projeksiyon bolgesini bul
        //--------------------------------------------------

       //--------------------------------------------------
// main.cpp ile ayni Adaptive HSV projeksiyon tespiti
//--------------------------------------------------

cv::Mat hsvFrame;
cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

const int H_MIN = 100;
const int H_MAX = 140;
const int S_MIN = 40;
const int V_MIN = 60;

int usedVThresh = 0;

cv::Mat projectionMask =
    detectColorRegionAdaptive(
        hsvFrame,
        H_MIN,
        H_MAX,
        S_MIN,
        V_MIN,
        usedVThresh
    );

projectionMask = cleanMask(projectionMask);

        //--------------------------------------------------
        // 2) En buyuk konturu bul
        //--------------------------------------------------

        std::vector<std::vector<cv::Point>> projectionContours;
        std::vector<cv::Vec4i> projectionHierarchy;

        int projectionIndex =
            findLargestContour(
                projectionMask,
                projectionContours,
                projectionHierarchy
            );

        //--------------------------------------------------
        // 3) Projeksiyonu olc
        //--------------------------------------------------
        std::cout << "[DEBUG] measureObject ONCESI" << std::endl;
        MeasurementData rawMeasurement =
        measureObject(
            projectionContours,
            projectionIndex,
            gray,
            *homographyMatrix,
            *homographyReady,
            *cameraMatrix,
            *distCoeffs,
            *calibrationLoaded,
            selectedModel
        );
    
    // Ham olcumu gecmise ekle
   //--------------------------------------------------
// OLCUM STABILIZASYONU + KISA TESPIT KAYIPLARINI YOK SAY
//--------------------------------------------------

MeasurementData measurement = rawMeasurement;

if(rawMeasurement.valid && rawMeasurement.mmValid)
{
    // Projeksiyon bu karede bulundu.
    *lostProjectionFrames = 0;

    measurementHistory->push(rawMeasurement);

    if(measurementHistory->ready())
    {
        measurement = measurementHistory->median();
    }

    // Son guvenilir olcumu sakla.
    *lastValidMeasurement = measurement;
    *hasLastValidMeasurement = true;
}
else
{
    // Bu karede tespit kacirildi.
    (*lostProjectionFrames)++;

    // Kisa sureli kayiplarda GUI'yi bozma.
    if(*hasLastValidMeasurement &&
       *lostProjectionFrames <= MAX_LOST_PROJECTION_FRAMES)
    {
        measurement = *lastValidMeasurement;
    }
}
        std::cout << "[DEBUG] measureObject SONRASI" << std::endl;
        //--------------------------------------------------
        // 4) Gecerli projeksiyon bulundu
        //--------------------------------------------------

        if(measurement.valid && measurement.mmValid)
        {
            projectionStatus->setText(
                "✓ Projeksiyon Tespiti"
            );

            widthLabel->setText(
                QString("Genislik : %1 mm")
                    .arg(measurement.widthMM, 0, 'f', 2)
            );

            heightLabel->setText(
                QString("Yukseklik : %1 mm")
                    .arg(measurement.heightMM, 0, 'f', 2)
            );

            angleLabel->setText(
                QString("Aci : %1 derece")
                    .arg(measurement.rotationDeg, 0, 'f', 2)
            );

            //--------------------------------------------------
            // Konturu kamera goruntusunde goster
            //--------------------------------------------------

           //--------------------------------------------------
// HEDEF PROJEKSIYON CERCEVESI
//--------------------------------------------------

// 4 vida referans alaninin merkezi
const float targetCenterX = 222.5f / 2.0f;
const float targetCenterY = 150.0f / 2.0f;

// DLP'nin gercek fiziksel boyutu
const float halfWidth  = selectedModel.widthMM  / 2.0f;
const float halfHeight = selectedModel.heightMM / 2.0f;

// Hedefin mm cinsinden 4 kosesi:
// TL, TR, BR, BL
std::vector<cv::Point2f> targetWorldCorners = {
    {targetCenterX - halfWidth, targetCenterY - halfHeight},
    {targetCenterX + halfWidth, targetCenterY - halfHeight},
    {targetCenterX + halfWidth, targetCenterY + halfHeight},
    {targetCenterX - halfWidth, targetCenterY + halfHeight}
};

// GUI'deki homography:
// kamera pikseli -> gercek dunya mm
//
// Hedefi kameraya cizebilmek icin tersine ihtiyacimiz var:
// gercek dunya mm -> kamera pikseli
cv::Mat inverseHomography = homographyMatrix->inv();

std::vector<cv::Point2f> targetImageCorners;

cv::perspectiveTransform(
    targetWorldCorners,
    targetImageCorners,
    inverseHomography
);

if(targetImageCorners.size() == 4)
{
    // Simdilik hedef sari.
    // Bir sonraki adimda OK oldugunda yesile cevirecegiz.
    cv::Scalar targetColor(0, 255, 255);

    for(int i = 0; i < 4; ++i)
    {
        cv::line(
            display,
            targetImageCorners[i],
            targetImageCorners[(i + 1) % 4],
            targetColor,
            3,
            cv::LINE_AA
        );
    }
}

            
            //--------------------------------------------------
// YENI HIZALAMA SONUCUNU HESAPLA
//--------------------------------------------------

AlignmentStatus calculatedAlignment =
updateAlignmentStatus(
    measurement,
    selectedModel
);

//--------------------------------------------------
// YONLENDIRME KILIDI
//--------------------------------------------------

if(!*alignmentLocked)
{
// Ilk komutu kabul et
*lockedAlignment = calculatedAlignment;

*alignmentStartCenter =
    measurement.centerMM;

*alignmentStartAngle =
    measurement.rotationDeg;

*alignmentLocked = true;
}
else
{
// Komut verildiginden beri projeksiyon
// ne kadar hareket etti?
double moveDistance =
    cv::norm(
        measurement.centerMM -
        *alignmentStartCenter
    );

double angleChange =
    std::abs(
        measurement.rotationDeg -
        *alignmentStartAngle
    );

//--------------------------------------------------
// Kullanici gercekten bir hamle yaptiysa
// yeni yonlendirmeyi kabul et
//--------------------------------------------------

if(moveDistance > COMMAND_MOVE_THRESHOLD_MM ||
   angleChange > COMMAND_ANGLE_THRESHOLD_DEG)
{
    *lockedAlignment =
        calculatedAlignment;

    *alignmentStartCenter =
        measurement.centerMM;

    *alignmentStartAngle =
        measurement.rotationDeg;
}
}

// GUI artik anlik sonucu degil,
// kilitlenmis sonucu kullanacak.
AlignmentStatus alignment =
*lockedAlignment;

std::string turkishDirection =
alignmentStatusToTurkish(
    alignment
);
            //--------------------------------------------------
            // Yonlendirme sembolu
            //--------------------------------------------------
            
            QString directionIcon;
            
            switch(alignment)
            {
                case AlignmentStatus::MOVE_LEFT:
                    directionIcon = "←";
                    break;
            
                case AlignmentStatus::MOVE_RIGHT:
                    directionIcon = "→";
                    break;
            
                case AlignmentStatus::MOVE_UP:
                    directionIcon = "↑";
                    break;
            
                case AlignmentStatus::MOVE_DOWN:
                    directionIcon = "↓";
                    break;
            
                case AlignmentStatus::ROTATE_CW:
                    directionIcon = "↻";
                    break;
            
                case AlignmentStatus::ROTATE_CCW:
                    directionIcon = "↺";
                    break;
            
                case AlignmentStatus::MOVE_FORWARD:
                    directionIcon = "↑";
                    break;
            
                case AlignmentStatus::MOVE_BACKWARD:
                    directionIcon = "↓";
                    break;
            
                case AlignmentStatus::OK:
                    directionIcon = "✓";
                    break;
            
                default:
                    directionIcon = "";
                    break;
            }
            
            QString directionText =
                QString::fromStdString(turkishDirection);
            
            if(!directionIcon.isEmpty())
            {
                direction->setText(
                    directionIcon + "   " + directionText
                );
            }
            else
            {
                direction->setText(directionText);
            }

            //--------------------------------------------------
            // Duruma gore renk
            //--------------------------------------------------

            if(alignment == AlignmentStatus::OK)
            {
                direction->setStyleSheet(
                    "font-size: 24px;"
                    "font-weight: bold;"
                    "background-color: #b7f7b7;"
                    "color: #006400;"
                    "border-radius: 8px;"
                    "padding: 10px;"
                );
            }
            else
            {
                direction->setStyleSheet(
                    "font-size: 24px;"
                    "font-weight: bold;"
                    "background-color: #fff3b0;"
                    "color: #7a4b00;"
                    "border-radius: 8px;"
                    "padding: 10px;"
                );
            }
        }
        else
        {
            projectionStatus->setText(
                "○ Projeksiyon Tespiti"
            );

            widthLabel->setText(
                "Genislik : -- mm"
            );

            heightLabel->setText(
                "Yukseklik : -- mm"
            );

            angleLabel->setText(
                "Aci : -- derece"
            );

            direction->setText(
                "PROJEKSIYON ARANIYOR..."
            );

            direction->setStyleSheet(
                "font-size: 24px;"
                "font-weight: bold;"
                "background-color: #eeeeee;"
                "color: #202020;"
                "border-radius: 8px;"
                "padding: 10px;"
            );
        }
    }

    //--------------------------------------------------
    // 7) OpenCV -> Qt
    //--------------------------------------------------

    cv::Mat rgbFrame;
    cv::cvtColor(
        display,
        rgbFrame,
        cv::COLOR_BGR2RGB
    );

    QImage image(
        rgbFrame.data,
        rgbFrame.cols,
        rgbFrame.rows,
        static_cast<int>(rgbFrame.step),
        QImage::Format_RGB888
    );

    cameraView->setPixmap(
        QPixmap::fromImage(image.copy()).scaled(
            cameraView->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation
        )
    );
});


QObject::connect(stopButton, &QPushButton::clicked, [=]() {

    cameraTimer->stop();

    if(cap->isOpened())
        cap->release();

    cameraView->clear();
    cameraView->setText("Kamera goruntusu burada gosterilecek");

    cameraStatus->setText("○ Kamera");
    direction->setText("SISTEM DURDURULDU");
});
    window.setCentralWidget(central);
    window.show();

    return app.exec();
}
