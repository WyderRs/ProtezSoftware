/********************************************************************************
** Form generated from reading UI file 'secondwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.16
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SECONDWINDOW_H
#define UI_SECONDWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionAngle_control;
    QAction *actionPWM_control;
    QWidget *centralwidget;
    QPushButton *pushButton_11;
    QLabel *label_6;
    QLabel *label_7;
    QComboBox *comboBox;
    QComboBox *comboBox_1;
    QPushButton *pushButton_12;
    QMenuBar *menubar;
    QMenu *menunual_operation;
    QMenu *menuType_module;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1250, 613);
        MainWindow->setInputMethodHints(Qt::ImhNone);
        actionAngle_control = new QAction(MainWindow);
        actionAngle_control->setObjectName(QString::fromUtf8("actionAngle_control"));
        actionPWM_control = new QAction(MainWindow);
        actionPWM_control->setObjectName(QString::fromUtf8("actionPWM_control"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        pushButton_11 = new QPushButton(centralwidget);
        pushButton_11->setObjectName(QString::fromUtf8("pushButton_11"));
        pushButton_11->setGeometry(QRect(20, 20, 101, 31));
        label_6 = new QLabel(centralwidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(130, 10, 51, 21));
        label_6->setFrameShape(QFrame::Panel);
        label_6->setAlignment(Qt::AlignCenter);
        label_7 = new QLabel(centralwidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(130, 40, 51, 21));
        label_7->setFrameShape(QFrame::Panel);
        label_7->setAlignment(Qt::AlignCenter);
        comboBox = new QComboBox(centralwidget);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));
        comboBox->setGeometry(QRect(190, 10, 131, 22));
        comboBox_1 = new QComboBox(centralwidget);
        comboBox_1->setObjectName(QString::fromUtf8("comboBox_1"));
        comboBox_1->setGeometry(QRect(190, 40, 131, 22));
        pushButton_12 = new QPushButton(centralwidget);
        pushButton_12->setObjectName(QString::fromUtf8("pushButton_12"));
        pushButton_12->setGeometry(QRect(20, 100, 101, 31));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1250, 21));
        menunual_operation = new QMenu(menubar);
        menunual_operation->setObjectName(QString::fromUtf8("menunual_operation"));
        menunual_operation->setGeometry(QRect(296, 101, 174, 74));
        menuType_module = new QMenu(menunual_operation);
        menuType_module->setObjectName(QString::fromUtf8("menuType_module"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        menubar->addAction(menunual_operation->menuAction());
        menunual_operation->addSeparator();
        menunual_operation->addAction(menuType_module->menuAction());
        menuType_module->addAction(actionAngle_control);
        menuType_module->addAction(actionPWM_control);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        actionAngle_control->setText(QCoreApplication::translate("MainWindow", "Angle control", nullptr));
        actionPWM_control->setText(QCoreApplication::translate("MainWindow", "PWM control", nullptr));
        pushButton_11->setText(QCoreApplication::translate("MainWindow", "Search device", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "Upper", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "Lower", nullptr));
        pushButton_12->setText(QCoreApplication::translate("MainWindow", "TEST", nullptr));
        menunual_operation->setTitle(QCoreApplication::translate("MainWindow", "File", nullptr));
        menuType_module->setTitle(QCoreApplication::translate("MainWindow", "Type module", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SECONDWINDOW_H
