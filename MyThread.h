#ifndef MYTHREAD_H
#define MYTHREAD_H

#include <QThread>
#include "mainwindow.h"


/*Thread_1*/
class MyThread_1 : public QThread
{
    Q_OBJECT

public:
    MyThread_1(MainWindow* mainWindowInstance1, QObject *parent = nullptr);
    ~MyThread_1();

private:
    MainWindow* mainWindow;
    QSerialPort *serialDevice1;  // Первый последовательный порт

public slots:
    void ComPortRead();

signals:
    void PaintGraph_signal();
    void PaintGraph2_signal();

protected:
    void run() override;
};
/*Thread_2*/

class MyThread_2 : public QThread
{
    Q_OBJECT

public:
    MyThread_2(MainWindow* mainWindowInstance2, QObject *parent = nullptr);
    ~MyThread_2();

private:
    MainWindow* mainWindow;

protected:
    void run() override;
};

#endif // MYTHREAD_H
