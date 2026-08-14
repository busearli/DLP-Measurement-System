#pragma once

#include <QLabel>
#include <QMouseEvent>

// FIX (manuel vida referansi): normal QLabel fare tiklamasi icin sinyal
// yaymaz. Kamera goruntusu uzerine tiklayip 4 referans vidayi elle
// secebilmek icin bu kucuk alt-sinifi ekliyoruz - mousePressEvent'i
// yakalayip clicked(QPoint) sinyali olarak disariya veriyor.
//
// NOT: bu dosyanin CMakeLists.txt'deki add_executable(MachineVisionGUI ...)
// kaynak listesine EKLENMESI ve projede AUTOMOC'un ACIK olmasi gerekiyor
// (asagidaki CMakeLists notuna bakin) - aksi halde Q_OBJECT/moc hatasi alirsiniz.

class ClickableLabel : public QLabel
{
    Q_OBJECT

public:
    explicit ClickableLabel(const QString& text = QString(), QWidget* parent = nullptr)
        : QLabel(text, parent)
    {
    }

signals:
    void clicked(QPoint pos);
    void rightClicked(QPoint pos);

protected:
    void mousePressEvent(QMouseEvent* event) override
    {
        if(event->button() == Qt::LeftButton)
        {
            emit clicked(event->pos());
        }
        else if(event->button() == Qt::RightButton)
        {
            emit rightClicked(event->pos());
        }

        QLabel::mousePressEvent(event);
    }
};