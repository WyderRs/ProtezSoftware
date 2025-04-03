#include "mainwindow.h"
#include <QApplication>
#include "MyThread.h"

extern MainWindow *GLB_mainwindow;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    // MainWindow w;
    MainWindow* w = new MainWindow();
    GLB_mainwindow = w;









    w->show();
    return a.exec();
}
