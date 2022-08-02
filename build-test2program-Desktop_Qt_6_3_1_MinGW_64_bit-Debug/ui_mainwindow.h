/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QPushButton *send_values;
    QPushButton *clear_values;
    QSlider *horizontalSlider1;
    QDoubleSpinBox *spinBox1;
    QSlider *horizontalSlider2;
    QDoubleSpinBox *spinBox2;
    QSlider *horizontalSlider3;
    QDoubleSpinBox *spinBox3;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(540, 200);
        MainWindow->setMinimumSize(QSize(540, 200));
        MainWindow->setMaximumSize(QSize(550, 200));
        QIcon icon;
        icon.addFile(QString::fromUtf8("//home.org.aalto.fi/ahvonea2/data/Desktop/188168.jpg"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindow->setWindowIcon(icon);
        MainWindow->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        send_values = new QPushButton(centralwidget);
        send_values->setObjectName(QString::fromUtf8("send_values"));
        send_values->setGeometry(QRect(10, 140, 251, 50));
        QFont font;
        font.setPointSize(15);
        font.setBold(true);
        send_values->setFont(font);
        clear_values = new QPushButton(centralwidget);
        clear_values->setObjectName(QString::fromUtf8("clear_values"));
        clear_values->setGeometry(QRect(279, 140, 251, 50));
        clear_values->setFont(font);
        horizontalSlider1 = new QSlider(centralwidget);
        horizontalSlider1->setObjectName(QString::fromUtf8("horizontalSlider1"));
        horizontalSlider1->setGeometry(QRect(10, 95, 160, 20));
        horizontalSlider1->setCursor(QCursor(Qt::ArrowCursor));
        horizontalSlider1->setMouseTracking(false);
        horizontalSlider1->setMinimum(-20);
        horizontalSlider1->setMaximum(-10);
        horizontalSlider1->setSingleStep(1);
        horizontalSlider1->setOrientation(Qt::Horizontal);
        spinBox1 = new QDoubleSpinBox(centralwidget);
        spinBox1->setObjectName(QString::fromUtf8("spinBox1"));
        spinBox1->setGeometry(QRect(10, 40, 160, 41));
        spinBox1->setFont(font);
        spinBox1->setAlignment(Qt::AlignCenter);
        spinBox1->setButtonSymbols(QAbstractSpinBox::NoButtons);
        spinBox1->setMinimum(-2.000000000000000);
        spinBox1->setMaximum(-1.000000000000000);
        spinBox1->setSingleStep(0.100000000000000);
        spinBox1->setValue(-1.000000000000000);
        horizontalSlider2 = new QSlider(centralwidget);
        horizontalSlider2->setObjectName(QString::fromUtf8("horizontalSlider2"));
        horizontalSlider2->setGeometry(QRect(190, 95, 160, 20));
        horizontalSlider2->setMinimum(-40);
        horizontalSlider2->setMaximum(40);
        horizontalSlider2->setOrientation(Qt::Horizontal);
        spinBox2 = new QDoubleSpinBox(centralwidget);
        spinBox2->setObjectName(QString::fromUtf8("spinBox2"));
        spinBox2->setGeometry(QRect(190, 40, 160, 41));
        spinBox2->setFont(font);
        spinBox2->setAlignment(Qt::AlignCenter);
        spinBox2->setButtonSymbols(QAbstractSpinBox::NoButtons);
        spinBox2->setMinimum(-40.000000000000000);
        spinBox2->setMaximum(40.000000000000000);
        spinBox2->setSingleStep(0.100000000000000);
        horizontalSlider3 = new QSlider(centralwidget);
        horizontalSlider3->setObjectName(QString::fromUtf8("horizontalSlider3"));
        horizontalSlider3->setGeometry(QRect(370, 95, 160, 20));
        horizontalSlider3->setMinimum(0);
        horizontalSlider3->setMaximum(20);
        horizontalSlider3->setOrientation(Qt::Horizontal);
        spinBox3 = new QDoubleSpinBox(centralwidget);
        spinBox3->setObjectName(QString::fromUtf8("spinBox3"));
        spinBox3->setGeometry(QRect(370, 40, 160, 41));
        spinBox3->setFont(font);
        spinBox3->setAlignment(Qt::AlignCenter);
        spinBox3->setButtonSymbols(QAbstractSpinBox::NoButtons);
        spinBox3->setMinimum(0.000000000000000);
        spinBox3->setMaximum(20.000000000000000);
        spinBox3->setSingleStep(0.100000000000000);
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 10, 63, 20));
        label->setFont(font);
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(190, 10, 63, 20));
        label_2->setFont(font);
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(370, 10, 63, 20));
        label_3->setFont(font);
        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        send_values->setText(QCoreApplication::translate("MainWindow", "Send", nullptr));
        clear_values->setText(QCoreApplication::translate("MainWindow", "Clear", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "X", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Z", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "Pitch", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
