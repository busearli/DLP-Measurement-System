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
#include <QMouseEvent>
#include <opencv2/opencv.hpp>
#include "screw_detector.hpp"
#include "measurement_engine.hpp"
#include "clickable_label.hpp"


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

//==================================================
// MANUEL VIDA REFERANSI: ekran (QLabel) koordinatini
// gercek kamera karesi koordinatina cevir
//==================================================
// FIX (manuel vida referansi): cameraView, goruntuyu KeepAspectRatio ile
// olcekleyip ortaliyor (letterbox). Kullanicinin tikladigi QLabel pikseli
// ile kameranin gercek kare pikseli farkli - bu fonksiyon olcek ve
// bosluklari (letterbox) hesaba katarak dogru donusumu yapar.

cv::Point2f mapLabelClickToFrame(const QPoint& clickPos, const QSize& labelSize, const cv::Size& frameSize)
{
    if(frameSize.width <= 0 || frameSize.height <= 0 ||
       labelSize.width() <= 0 || labelSize.height() <= 0)
    {
        return cv::Point2f(-1.0f, -1.0f);
    }

    double scale = std::min(
        static_cast<double>(labelSize.width())  / frameSize.width,
        static_cast<double>(labelSize.height()) / frameSize.height
    );

    double dispW = frameSize.width  * scale;
    double dispH = frameSize.height * scale;

    double offsetX = (labelSize.width()  - dispW) / 2.0;
    double offsetY = (labelSize.height() - dispH) / 2.0;

    double fx = (clickPos.x() - offsetX) / scale;
    double fy = (clickPos.y() - offsetY) / scale;

    if(fx < 0 || fy < 0 || fx >= frameSize.width || fy >= frameSize.height)
    {
        return cv::Point2f(-1.0f, -1.0f); // letterbox bosluguna tiklandi
    }

    return cv::Point2f(static_cast<float>(fx), static_cast<float>(fy));
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
    // FIX (manuel vida referansi): normal QLabel yerine ClickableLabel -
    // artik uzerine tiklamalari yakalayabiliyoruz.
    ClickableLabel *cameraView = new ClickableLabel("Kamera goruntusu burada gosterilecek");
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

// FIX (manuel vida referansi): son gelen HAM (undistort/gray donusum
// oncesi degil, camerayla ayni boyutta) karenin boyutu - tiklama
// koordinat donusumu icin gerekli.
cv::Size *lastFrameSize = new cv::Size(0, 0);

// FIX (manuel vida referansi): kullanicinin sirayla tikladigi 4 nokta
// (kamera KARE koordinatinda, QLabel koordinatinda DEGIL).
std::vector<cv::Point2f> *manualScrewPoints = new std::vector<cv::Point2f>();

const char* SCREW_CLICK_NAMES[4] = { "TL (sol ust)", "TR (sag ust)", "BR (sag alt)", "BL (sol alt)" };

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

    // FIX (manuel vida referansi): her yeni baslatmada tiklama listesini
    // sifirla, boylece onceki oturumdan kalan noktalar karismaz.
    manualScrewPoints->clear();

    direction->setText("KAMERA AKTIF - EKRANDAKI 4 REFERANS VIDAYA SIRAYLA TIKLAYIN (TL, TR, BR, BL)");
    screwStatus->setText("○ Vida Referansi 0/4");

    cameraTimer->start(30);
});

// (Eski otomatik-tespit degiskenleri asagida hala duruyor - screwReferenceLocked/
// lockedScrewQuad artik MANUEL tiklamayla dolduruluyor, mantik ayni kaldi ki
// homografi/olcum tarafina (asagisi) HIC DOKUNMAMIS olalim.)

int *goodScrewFrames = new int(0);          // artik kullanilmiyor (manuel modda), dokunulmadi
int *missedScrewFrames = new int(0);        // artik kullanilmiyor (manuel modda), dokunulmadi

std::vector<cv::Point2f> *previousScrewQuad =
    new std::vector<cv::Point2f>();         // artik kullanilmiyor (manuel modda), dokunulmadi

std::vector<cv::Point2f> *accumulatedScrewQuad =
    new std::vector<cv::Point2f>(
        4,
        cv::Point2f(0.0f, 0.0f)
    );                                       // artik kullanilmiyor (manuel modda), dokunulmadi

bool *screwReferenceLocked = new bool(false);

std::vector<cv::Point2f> *lockedScrewQuad =
    new std::vector<cv::Point2f>();

// main.cpp ile AYNI degerler (otomatik moda donuldugunde kullanilacak)
const int REQUIRED_SCREW_FRAMES = 8;
const double MAX_SCREW_JUMP_PX = 15.0;
const int MAX_MISSED_SCREW_FRAMES = 3;

//--------------------------------------------------
// FIX (manuel vida referansi): kamera goruntusune tiklandiginda calisir.
// 4. noktadan sonraki tiklamalar yok sayilir (reset icin sag tik kullanin).
//--------------------------------------------------

QObject::connect(cameraView, &ClickableLabel::clicked, [=](QPoint pos) {

    if(!cap->isOpened())
        return;

    if(*screwReferenceLocked)
    {
        // Referans zaten kilitlendiyse yeni tiklamalari yok say -
        // once sag tikla sifirlamak gerekir.
        return;
    }

    if(manualScrewPoints->size() >= 4)
        return;

    cv::Point2f framePt = mapLabelClickToFrame(pos, cameraView->size(), *lastFrameSize);

    if(framePt.x < 0 || framePt.y < 0)
    {
        std::cout << "[MANUEL VIDA] Tiklama goruntu disina denk geldi, yok sayildi." << std::endl;
        return;
    }

    manualScrewPoints->push_back(framePt);

    std::cout << "[MANUEL VIDA] Nokta " << manualScrewPoints->size() << "/4 ("
              << SCREW_CLICK_NAMES[manualScrewPoints->size() - 1] << "): ("
              << framePt.x << ", " << framePt.y << ")" << std::endl;

    screwStatus->setText(
        QString("○ Vida Referansi %1/4").arg(manualScrewPoints->size())
    );

    if(manualScrewPoints->size() < 4)
    {
        direction->setText(
            QString("SIMDI TIKLAYIN: %1").arg(SCREW_CLICK_NAMES[manualScrewPoints->size()])
        );
    }
    else
    {
        //--------------------------------------------------
        // 4 nokta tamamlandi - homografiyi HEMEN hesapla.
        //--------------------------------------------------

        *lockedScrewQuad = *manualScrewPoints;

        std::vector<cv::Point2f> worldPoints = {
            {0.0f, 0.0f},
            {222.5f, 0.0f},
            {222.5f, 150.0f},
            {0.0f, 150.0f}
        };

        std::cout << "[MANUEL VIDA] 4 nokta tamamlandi, homografi hesaplaniyor..." << std::endl;

        *screwReferenceLocked = true;

        screwStatus->setText("✓ Vida Referansi");
        direction->setText("VIDA REFERANSI TAMAMLANDI - HOMOGRAFI HESAPLANIYOR...");

        // NOT: homografi hesaplama ve stage2Active/homographyReady atamasi
        // asagida (timer lambda'sinin en ustunde) tek seferlik olarak
        // yapiliyor - orada cv::findHomography sonucu KONTROL EDILEBILIYOR
        // (bos donerse kullaniciya hata gosterip sifirlayabiliyoruz).
    }
});

//--------------------------------------------------
// FIX (manuel vida referansi): sag tik = referansi sifirla, yeniden
// tikla. Yanlis bir noktaya tiklandiginda 4'u de yeniden yapmaya
// gerek kalmadan (veya kilitlendikten sonra yanlis cikarsa) kullanmak icin.
//--------------------------------------------------

QObject::connect(cameraView, &ClickableLabel::rightClicked, [=](QPoint) {

    if(!cap->isOpened())
        return;

    manualScrewPoints->clear();
    lockedScrewQuad->clear();
    *screwReferenceLocked = false;

    std::cout << "[MANUEL VIDA] Referans sifirlandi, yeniden tiklamaya baslayin." << std::endl;

    screwStatus->setText("○ Vida Referansi 0/4");
    direction->setText("REFERANS SIFIRLANDI - EKRANDAKI 4 VIDAYA SIRAYLA TIKLAYIN (TL, TR, BR, BL)");
});

//==================================================
// ASAMA 2 - HOMOGRAFI / PROJEKSIYON OLCUM DURUMU
//==================================================

cv::Mat *homographyMatrix = new cv::Mat();
bool *homographyReady = new bool(false);
bool *stage2Active = new bool(false);
//--------------------------------------------------
// ASAMA 2 - DIKLIK AYARI
//--------------------------------------------------

enum class PerpendicularStage
{
    TOP_BOTTOM,
    LEFT_RIGHT,
    COMPLETED
};

PerpendicularStage *perpendicularStage =
    new PerpendicularStage(
        PerpendicularStage::TOP_BOTTOM
    );

// Kenarlar kac ard arda kare tolerans icinde?
int *perpendicularStableFrames =
    new int(0);

// Tek bir olcume gore asama gecmeyelim.
const int PERPENDICULAR_REQUIRED_FRAMES = 5;

// Karsilikli kenarlar arasinda izin verilen fark.
// Ilk test icin 0.50 mm kullaniyoruz.
const double PERPENDICULAR_TOLERANCE_MM = 0.20;
// GUI yazilarini daha yavas guncelle
int *guiUpdateCounter = new int(0);

const int GUI_UPDATE_EVERY_N_FRAMES = 15;
//--------------------------------------------------
// DIKLIK ESITLENEN DEGERLERI KILITLE
//--------------------------------------------------

// Ust-alt esitlendigi anda bulunan ortak deger
double *lockedTopBottomMM =
    new double(0.0);

// Sol-sag esitlendigi anda bulunan ortak deger
double *lockedLeftRightMM =
    new double(0.0);

// Bu degerler su anda kilitli mi?
bool *topBottomValueLocked =
    new bool(false);

bool *leftRightValueLocked =
    new bool(false);
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

//==================================================
// ADIM ADIM HIZALAMA
//==================================================

// Verilen komutun hedef degeri.
// Ornegin MOVE_LEFT komutu verildiginde,
// hedef X koordinati burada tutulur.
double *alignmentTargetValue =
    new double(0.0);

// Hamle tamamlandi mi?
bool *alignmentStepCompleted =
    new bool(false);

// Tamamlandi mesajini birkac kare gostermek icin
int *alignmentCompletedFrames =
    new int(0);

const int ALIGNMENT_COMPLETED_DISPLAY_FRAMES = 15;
//--------------------------------------------------
// HIZALAMA BOZULMA KONTROLU
//--------------------------------------------------

int *alignmentLostFrames =
    new int(0);

const int ALIGNMENT_LOST_REQUIRED_FRAMES = 5;

//--------------------------------------------------
// KAMERA TIMER
//--------------------------------------------------

QObject::connect(cameraTimer, &QTimer::timeout, [=]() {

    static int frameCounter = 0;

    std::cout
        << "[FRAME] "
        << ++frameCounter
        << std::endl;

    if(!cap->isOpened())
        return;
//--------------------------------------------------
// ASAMA 2 - DIKLIK AYARI
//--------------------------------------------------


    cv::Mat frame;
    (*cap) >> frame;

    if(frame.empty())
        return;

    // FIX (manuel vida referansi): tiklama koordinat donusumu icin
    // kare boyutunu her zaman guncel tut.
    *lastFrameSize = frame.size();

    //--------------------------------------------------
    // 1) Gri goruntu
    //--------------------------------------------------

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::Mat display = frame.clone();

    //--------------------------------------------------
    // FIX (manuel vida referansi): eski otomatik tespit
    // (detectScrewCandidates / selectBestScrewQuad / sanityCheckQuad)
    // BURADAN KALDIRILDI - screw_detector.cpp/hpp icinde hala mevcut,
    // silinmedi. Otomatik moda donmek icin bu bloğun yerine eski
    // cagriyi geri koymak yeterli (asagidaki manuel cizim blogunu
    // kaldirip yerine eski "candidates/selectBestScrewQuad" akisini
    // eklemeniz yeterli).
    //--------------------------------------------------

    // Zaten tiklanmis (henuz 4'e tamamlanmamis) noktalari ciz.
    for(size_t i = 0; i < manualScrewPoints->size(); i++)
    {
        cv::circle(display, (*manualScrewPoints)[i], 8, cv::Scalar(0, 255, 0), -1);
        cv::putText(display, SCREW_CLICK_NAMES[i],
                    (*manualScrewPoints)[i] + cv::Point2f(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }

    //--------------------------------------------------
    // Referans daha yeni kilitlendiyse (4. tiklama bu karede degil,
    // onceki bir click-callback'te oldu) homografiyi TEK SEFER hesapla.
    //--------------------------------------------------

    if(*screwReferenceLocked && !*homographyReady && lockedScrewQuad->size() == 4)
    {
        std::vector<cv::Point2f> worldPoints = {
            {0.0f, 0.0f},
            {222.5f, 0.0f},
            {222.5f, 150.0f},
            {0.0f, 150.0f}
        };

        *homographyMatrix = cv::findHomography(*lockedScrewQuad, worldPoints);

        if(!homographyMatrix->empty())
        {
            *homographyReady = true;
            *stage2Active = true;

            std::cout << "[MANUEL VIDA] Homografi basariyla olusturuldu." << std::endl;
            std::cout << "[MANUEL VIDA] ASAMA 2 baslatildi." << std::endl;

            direction->setText("ASAMA 2 - PROJEKSIYON ARANIYOR...");
        }
        else
        {
            // FIX: 4 nokta neredeyse dogrusal/dejenere ise findHomography
            // bos donebilir - kullaniciyi bilgilendirip sifirla.
            std::cerr << "[MANUEL VIDA][HATA] Homografi hesaplanamadi "
                      << "(4 nokta dogrusal/gecersiz olabilir)." << std::endl;

            direction->setText("HOMOGRAFI HATASI - SAG TIKLA SIFIRLAYIP TEKRAR TIKLAYIN");

            manualScrewPoints->clear();
            lockedScrewQuad->clear();
            *screwReferenceLocked = false;

            screwStatus->setText("○ Vida Referansi 0/4");
        }
    }

    // Kilitli referansi (yesil dortgen) her karede ciz.
    if(*screwReferenceLocked && lockedScrewQuad->size() == 4)
    {
        const char* shortNames[4] = { "TL", "TR", "BR", "BL" };

        for(int i = 0; i < 4; i++)
        {
            cv::circle(display, (*lockedScrewQuad)[i], 8, cv::Scalar(0, 255, 0), -1);
            cv::putText(display, shortNames[i], (*lockedScrewQuad)[i] + cv::Point2f(10, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::line(display, (*lockedScrewQuad)[i], (*lockedScrewQuad)[(i + 1) % 4],
                     cv::Scalar(0, 255, 0), 2);
        }
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
// ASAMA 2 - GERCEK PROJEKSIYON SINIRI + KENAR OLCULERI
//--------------------------------------------------

if(measurement.imageCorners.size() == 4)
{
    const auto& c = measurement.imageCorners;

    cv::Scalar projectionColor(0, 255, 255); // sari yazi

     
    //--------------------------------------------------
    // KENAR ORTA NOKTALARI
    //--------------------------------------------------

    cv::Point2f topMid =
        (c[0] + c[1]) * 0.5f;

    cv::Point2f rightMid =
        (c[1] + c[2]) * 0.5f;

    cv::Point2f bottomMid =
        (c[2] + c[3]) * 0.5f;

    cv::Point2f leftMid =
        (c[3] + c[0]) * 0.5f;

    //--------------------------------------------------
    // OLCULEN GERCEK KENAR UZUNLUKLARI
    //--------------------------------------------------

    std::string topText =
        cv::format("%.2f mm", measurement.edge01MM);

    std::string rightText =
        cv::format("%.2f mm", measurement.edge12MM);

    std::string bottomText =
        cv::format("%.2f mm", measurement.edge23MM);

    std::string leftText =
        cv::format("%.2f mm", measurement.edge30MM);

    //--------------------------------------------------
    // UST KENAR
    //--------------------------------------------------

    cv::putText(
        display,
        topText,
        cv::Point(
            cvRound(topMid.x - 45),
            cvRound(topMid.y - 12)
        ),
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        cv::Scalar(0, 255, 255),
        2,
        cv::LINE_AA
    );

    //--------------------------------------------------
    // ALT KENAR
    //--------------------------------------------------

    cv::putText(
        display,
        bottomText,
        cv::Point(
            cvRound(bottomMid.x - 45),
            cvRound(bottomMid.y + 25)
        ),
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        cv::Scalar(0, 255, 255),
        2,
        cv::LINE_AA
    );

    //--------------------------------------------------
    // SOL KENAR
    //--------------------------------------------------

    cv::putText(
        display,
        leftText,
        cv::Point(
            cvRound(leftMid.x + 10),
            cvRound(leftMid.y)
        ),
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        cv::Scalar(0, 255, 255),
        2,
        cv::LINE_AA
    );

    //--------------------------------------------------
    // SAG KENAR
    //--------------------------------------------------

    cv::putText(
        display,
        rightText,
        cv::Point(
            cvRound(rightMid.x - 100),
            cvRound(rightMid.y)
        ),
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        cv::Scalar(0, 255, 255),
        2,
        cv::LINE_AA
    );
}

//--------------------------------------------------
// ASAMA 2 - DIKLIK KONTROLU
//--------------------------------------------------

if(*perpendicularStage != PerpendicularStage::COMPLETED)
{
    //--------------------------------------------------
    // 1. ADIM: UST - ALT
    //--------------------------------------------------

    if(*perpendicularStage == PerpendicularStage::TOP_BOTTOM)
    {
        double topBottomDiff =
            measurement.edge01MM - measurement.edge23MM;

        double absDiff = std::abs(topBottomDiff);

        if(absDiff <= PERPENDICULAR_TOLERANCE_MM)
        {
            (*perpendicularStableFrames)++;

            // Esitlik gercekten kararlı hale geldiyse kilitle.
            if(*perpendicularStableFrames >=
               PERPENDICULAR_REQUIRED_FRAMES)
            {
                *lockedTopBottomMM =
                    (measurement.edge01MM +
                     measurement.edge23MM) / 2.0;

                *topBottomValueLocked = true;

                direction->setText(
                    QString(
                        "✓ UST-ALT ESIT | Ust: %1 mm | Alt: %2 mm | Fark: %3 mm"
                    )
                    .arg(measurement.edge01MM, 0, 'f', 2)
                    .arg(measurement.edge23MM, 0, 'f', 2)
                    .arg(absDiff, 0, 'f', 2)
                );

                direction->setStyleSheet(
                    "font-size: 24px;"
                    "font-weight: bold;"
                    "background-color: #b7f7b7;"
                    "color: #006400;"
                    "border-radius: 8px;"
                    "padding: 10px;"
                );

                std::cout
                    << "[DIKLIK] UST-ALT ESITLENDI: "
                    << *lockedTopBottomMM
                    << " mm"
                    << std::endl;

                *perpendicularStage =
                    PerpendicularStage::LEFT_RIGHT;

                *perpendicularStableFrames = 0;
            }
        }
        else
        {
            *perpendicularStableFrames = 0;

            // Esitlik bozulduysa eski kilit artik gecerli degil.
            *topBottomValueLocked = false;

            (*guiUpdateCounter)++;

            if(*guiUpdateCounter >= GUI_UPDATE_EVERY_N_FRAMES)
            {
                *guiUpdateCounter = 0;

                if(topBottomDiff > 0.0)
                {
                    direction->setText(
                        QString(
                            "↑  YUKARI HAREKET ETTIR  |  "
                            "Ust: %1 mm  Alt: %2 mm  |  Fark: %3 mm"
                        )
                        .arg(measurement.edge01MM, 0, 'f', 2)
                        .arg(measurement.edge23MM, 0, 'f', 2)
                        .arg(absDiff, 0, 'f', 2)
                    );
                }
                else
                {
                    direction->setText(
                        QString(
                            "↓  ASAGI HAREKET ETTIR  |  "
                            "Ust: %1 mm  Alt: %2 mm  |  Fark: %3 mm"
                        )
                        .arg(measurement.edge01MM, 0, 'f', 2)
                        .arg(measurement.edge23MM, 0, 'f', 2)
                        .arg(absDiff, 0, 'f', 2)
                    );
                }

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
    }

    //--------------------------------------------------
    // 2. ADIM: SOL - SAG
    //--------------------------------------------------

    else if(*perpendicularStage == PerpendicularStage::LEFT_RIGHT)
    {
        double leftRightDiff =
            measurement.edge30MM - measurement.edge12MM;

        double absDiff = std::abs(leftRightDiff);

        if(absDiff <= PERPENDICULAR_TOLERANCE_MM)
        {
            (*perpendicularStableFrames)++;

            if(*perpendicularStableFrames >=
               PERPENDICULAR_REQUIRED_FRAMES)
            {
                *lockedLeftRightMM =
                    (measurement.edge30MM +
                     measurement.edge12MM) / 2.0;

                *leftRightValueLocked = true;

                direction->setText(
                    QString(
                        "✓ SOL-SAG ESIT | Sol: %1 mm | Sag: %2 mm | Fark: %3 mm"
                    )
                    .arg(measurement.edge30MM, 0, 'f', 2)
                    .arg(measurement.edge12MM, 0, 'f', 2)
                    .arg(absDiff, 0, 'f', 2)
                );

                direction->setStyleSheet(
                    "font-size: 24px;"
                    "font-weight: bold;"
                    "background-color: #b7f7b7;"
                    "color: #006400;"
                    "border-radius: 8px;"
                    "padding: 10px;"
                );

                std::cout
                    << "[DIKLIK] SOL-SAG ESITLENDI: "
                    << *lockedLeftRightMM
                    << " mm"
                    << std::endl;

                // SOL-SAG esitlendi.
                // ASAMA 3 su an kapali.
                // Tekrar UST-ALT kontrolune donerek
                // dikligi surekli kontrol ediyoruz.
                *perpendicularStage =
                    PerpendicularStage::TOP_BOTTOM;

                *perpendicularStableFrames = 0;

                std::cout
                    << "[DIKLIK] SOL-SAG TAMAM. "
                    << "UST-ALT TEKRAR KONTROL EDILIYOR."
                    << std::endl;
            }
        }
        else
        {
            *perpendicularStableFrames = 0;

            *leftRightValueLocked = false;

            (*guiUpdateCounter)++;

            if(*guiUpdateCounter >= GUI_UPDATE_EVERY_N_FRAMES)
            {
                *guiUpdateCounter = 0;

                if(leftRightDiff > 0.0)
                {
                    direction->setText(
                        QString(
                            "←  SOLA HAREKET ETTIR  |  "
                            "Sol: %1 mm  Sag: %2 mm  |  Fark: %3 mm"
                        )
                        .arg(measurement.edge30MM, 0, 'f', 2)
                        .arg(measurement.edge12MM, 0, 'f', 2)
                        .arg(absDiff, 0, 'f', 2)
                    );
                }
                else
                {
                    direction->setText(
                        QString(
                            "→  SAGA HAREKET ETTIR  |  "
                            "Sol: %1 mm  Sag: %2 mm  |  Fark: %3 mm"
                        )
                        .arg(measurement.edge30MM, 0, 'f', 2)
                        .arg(measurement.edge12MM, 0, 'f', 2)
                        .arg(absDiff, 0, 'f', 2)
                    );
                }

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
    }
}          

//--------------------------------------------------
// ASAMA 3 - MERKEZLEME / HIZALAMA
// Sadece diklik tamamlandiktan sonra calisir.
//--------------------------------------------------

if(false && *perpendicularStage == PerpendicularStage::COMPLETED)
{

            //--------------------------------------------------
// YENI HIZALAMA SONUCUNU HESAPLA
//--------------------------------------------------

AlignmentStatus calculatedAlignment =
updateAlignmentStatus(
    measurement,
    selectedModel
);

//--------------------------------------------------
// ADIM ADIM HIZALAMA KILIDI
//--------------------------------------------------

// Yeni bir komut baslat
if(!*alignmentLocked)
{
    *lockedAlignment = calculatedAlignment;
    *alignmentStepCompleted = false;
    *alignmentCompletedFrames = 0;
    *alignmentLocked = true;

    // Bu degerler debug / takip icin tutuluyor
    *alignmentStartCenter = measurement.centerMM;
    *alignmentStartAngle  = measurement.rotationDeg;
}

//--------------------------------------------------
// MEVCUT KOMUT GERCEKTEN TAMAMLANDI MI?
//--------------------------------------------------
//--------------------------------------------------
// HIZALAMA TAMAMLANDIKTAN SONRA BOZULDU MU?
//--------------------------------------------------

if(*lockedAlignment == AlignmentStatus::OK &&
    !*alignmentStepCompleted)
 {
     if(calculatedAlignment != AlignmentStatus::OK)
     {
         (*alignmentLostFrames)++;
 
         std::cout
             << "[ALIGNMENT] Hizalama disina cikma: "
             << *alignmentLostFrames
             << "/"
             << ALIGNMENT_LOST_REQUIRED_FRAMES
             << std::endl;
 
         // Gercekten bozulduguna emin olduktan sonra
         // yeniden yonlendirme baslat.
         if(*alignmentLostFrames >=
            ALIGNMENT_LOST_REQUIRED_FRAMES)
         {
             *lockedAlignment = calculatedAlignment;
 
             *alignmentStepCompleted = false;
             *alignmentCompletedFrames = 0;
             *alignmentLostFrames = 0;
 
             *alignmentStartCenter =
                 measurement.centerMM;
 
             *alignmentStartAngle =
                 measurement.rotationDeg;
 
             std::cout
                 << "[ALIGNMENT] Hizalama bozuldu. "
                 << "Yeni yonlendirme baslatildi."
                 << std::endl;
         }
     }
     else
     {
         // Hala hizaliysa sayaci sifirla.
         *alignmentLostFrames = 0;
     }
 }

if(!*alignmentStepCompleted)
{
    bool stepFinished = false;

    switch(*lockedAlignment)
    {
        //--------------------------------------------------
        // YATAY HIZALAMA
        //--------------------------------------------------
        case AlignmentStatus::MOVE_LEFT:
        case AlignmentStatus::MOVE_RIGHT:
        {
            constexpr double TARGET_X = 220.41 / 2.0;

            double xError =
                std::abs(measurement.centerMM.x - TARGET_X);

            if(xError <= CENTER_TOLERANCE_MM)
                stepFinished = true;

            break;
        }

        //--------------------------------------------------
        // DIKEY HIZALAMA
        //--------------------------------------------------
        case AlignmentStatus::MOVE_UP:
        case AlignmentStatus::MOVE_DOWN:
        {
            constexpr double TARGET_Y = 145.60 / 2.0;

            double yError =
                std::abs(measurement.centerMM.y - TARGET_Y);

            if(yError <= CENTER_TOLERANCE_MM)
                stepFinished = true;

            break;
        }

        //--------------------------------------------------
        // MESAFE / BOYUT HIZALAMA
        //--------------------------------------------------
        case AlignmentStatus::MOVE_FORWARD:
        case AlignmentStatus::MOVE_BACKWARD:
        {
            double nominalAvg =
                (selectedModel.widthMM +
                 selectedModel.heightMM) / 2.0;

            double measuredAvg =
                (measurement.widthMM +
                 measurement.heightMM) / 2.0;

            double sizeDiffPct =
                (nominalAvg > 1e-6)
                ? std::abs(
                    (measuredAvg - nominalAvg) /
                    nominalAvg * 100.0
                  )
                : 0.0;

            if(sizeDiffPct <= SIZE_TOLERANCE_PCT)
                stepFinished = true;

            break;
        }

        //--------------------------------------------------
        // DONUS HIZALAMA
        //--------------------------------------------------
        case AlignmentStatus::ROTATE_CW:
        case AlignmentStatus::ROTATE_CCW:
        {
            if(std::abs(measurement.rotationDeg)
               <= ANGLE_TOLERANCE_DEG)
            {
                stepFinished = true;
            }

            break;
        }

        //--------------------------------------------------
        // ZATEN TAM HIZALI
        //--------------------------------------------------
        case AlignmentStatus::OK:
        {
            break;
        }

        default:
            break;
    }

    //--------------------------------------------------
    // BU ADIM TAMAMLANDI
    //--------------------------------------------------

    if(stepFinished)
    {
        *alignmentStepCompleted = true;
        *alignmentCompletedFrames = 0;
    }
}

//--------------------------------------------------
// TAMAMLANAN ADIMI BIR SURE YESIL GOSTER
//--------------------------------------------------

if(*alignmentStepCompleted)
{
    (*alignmentCompletedFrames)++;

    if(*alignmentCompletedFrames >=
       ALIGNMENT_COMPLETED_DISPLAY_FRAMES)
    {
        // Artik siradaki gerekli hareketi hesapla
        *lockedAlignment = calculatedAlignment;

        *alignmentStepCompleted = false;
        *alignmentCompletedFrames = 0;

        *alignmentStartCenter = measurement.centerMM;
        *alignmentStartAngle  = measurement.rotationDeg;
    }
}

//--------------------------------------------------
// GUI'DE GOSTERILECEK DURUM
//--------------------------------------------------

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

            if(alignment == AlignmentStatus::OK ||
                *alignmentStepCompleted)
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

    // FIX (manuel vida referansi): DURDUR sonrasi tekrar SISTEMI BASLAT'a
    // basildiginda temiz bir durumdan baslasin.
    manualScrewPoints->clear();
    lockedScrewQuad->clear();
    *screwReferenceLocked = false;
    *homographyReady = false;
    *stage2Active = false;
    *alignmentLocked = false;
    *hasLastValidMeasurement = false;

    screwStatus->setText("○ Vida Referansi");
    projectionStatus->setText("○ Projeksiyon Tespiti");
    widthLabel->setText("Genislik : -- mm");
    heightLabel->setText("Yukseklik : -- mm");
    angleLabel->setText("Aci : -- derece");
});
    window.setCentralWidget(central);
    window.show();

    return app.exec();
}
