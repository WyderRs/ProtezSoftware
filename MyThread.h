#ifndef MYTHREAD_H
#define MYTHREAD_H

#include <QThread>
#include "mainwindow.h"
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QDateTime>



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
    QVector<uint8_t> ComPortRead();
    QList<QString> ComPortSearch();
    QString ComPortFoundPort();
    bool ComPortConnect();
    void ComPortClose();
    QString ComPortWrite(uint8_t *data, uint32_t cntdata);


signals:
    void PaintGraph_signal();
    void PaintGraph2_signal();

    void ComportDataUpdate_signal();
    void ComportConnect_signal();
    void ComportClose_signal();
    void ComPortConnect_signal();
    void ComportRead_signal();
    void ComPortWrite_signal(QString back);


    // void ComportSelect_signal(uint8_t numcom, bool state);
    // void ComportOpenPort_signal(QString portName, qint32 baudRate);
    // QList<QString> ComportSearch_signal();


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
    // QSerialPort *serialDevice1;  // Первый последовательный порт

protected:
    void run() override;

signals:
    // void ComportSelect_signal(uint8_t numcom, bool state);
    // void ComportOpenPort_signal(QString portName, qint32 baudRate);
    // void ComportClose_signal();
    // QList<QString> ComportSearch_signal();
};

#endif // MYTHREAD_H
