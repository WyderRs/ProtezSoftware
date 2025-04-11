#include "MyThread.h"
#include <QTimer>
#include "mainwindow.h"
#include <QDateTime>


extern bool GLB_Thread_Flag[2];
extern QSerialPort *GLB_Ports[2];
extern uint32_t PackToRecv;
extern uint32_t GLB_I;
extern QList<QString> GLB_Comports;

/*Graph variables*/
extern QVector<uint8_t> GLB_Graph_x, GLB_Graph_y;

/*Thread 1*/
MyThread_1::MyThread_1(MainWindow* mainWindowInstance1, QObject *parent) : QThread(parent), mainWindow(mainWindowInstance1)
{

}
MyThread_1::~MyThread_1()
{
    qDebug() << "Thread1 is killed!";
}
void MyThread_1::run()
{
    GLB_Graph_y.clear();
    GLB_Graph_x.clear();
    // GLB_Ports[0]->clear();
    if (GLB_Ports[0]->isOpen())
    {
        while(GLB_Thread_Flag[0])
        {
            ComPortRead();
        }
    }
    qDebug() << "Thread #1 is disable!";
}

// void MyThread_1::ComPortRead()
// {
//     uint32_t dataRecvd = 0;

//     while (dataRecvd < PackToRecv)
//     {
//         QByteArray newData = GLB_Ports[0]->readAll(); // Читаем все доступные данные

//         if (!newData.isEmpty())
//         {
//             for (char byte : newData) {
//                 GLB_Graph_y.append(static_cast<uint8_t>(byte));
//                 dataRecvd++;
//                 // qDebug() << "DataRecv: " << dataRecvd;
//             }
//         }
//         if (dataRecvd >= PackToRecv) break;
//         // Задержка для снижения нагрузки на процессор
//         QThread::msleep(12);
//     }
//     GLB_Thread_Flag[0] = false;
//     GLB_Ports[0]->clear();
//     emit PaintGraph2_signal(); // Сигнал для обновления графика

// }
void MyThread_1::ComPortRead()
{
    uint32_t dataRecvd = 0;
    bool flagHaveData = false;
    qint64 lastTime = QDateTime::currentMSecsSinceEpoch();
    while (1)
    {
        QByteArray newData = GLB_Ports[0]->readAll();
        if (!newData.isEmpty())
        {
            for (char byte : newData) {
                GLB_Graph_y.append(static_cast<uint8_t>(byte));
                dataRecvd++;
                lastTime = QDateTime::currentMSecsSinceEpoch();
            }
            flagHaveData = true;
            break;
        }
        if (QDateTime::currentMSecsSinceEpoch() - lastTime > 5000)
        {
            flagHaveData = false;
            break;
        }
        QThread::msleep(20);
    }

    GLB_Thread_Flag[0] = false;
    GLB_Ports[0]->clear();
    if(flagHaveData) emit PaintGraph2_signal(); // Сигнал для обновления графика
}



/*Thread 2*/
MyThread_2::MyThread_2(MainWindow* mainWindowInstance2, QObject *parent) : QThread(parent), mainWindow(mainWindowInstance2)
{

}
MyThread_2::~MyThread_2()
{
    qDebug() << "Thread2 is killed!";
}
void MyThread_2::run()
{
    uint8_t NumCom;
    bool flag;
    bool flagFoundPort;
    emit ComportSearch_signal();
    GLB_Comports = GLB_Comports;
    while(1)
    {
        if(!GLB_Comports.isEmpty())
        {
            GLB_Thread_Flag[1] = true;
            break;
        }
    }
    if(GLB_Thread_Flag[1])
    {
        while(1)
        {
            if(!flag)
            {
                ComportOpenPort_signal(GLB_Comports[NumCom], 115200);
                flag = true;
            }
            if (GLB_Ports[0]->isOpen())
            {
                qint64 lastTime = QDateTime::currentMSecsSinceEpoch();
                QByteArray receivedData;
                while (1)
                {
                    QByteArray newData = GLB_Ports[0]->readAll();
                    if (!newData.isEmpty())
                    {
                        receivedData.append(newData);
                        if (receivedData.contains(QByteArray::fromHex("FFFFFF")))
                        {
                            emit ComportSelect_signal(NumCom, true);
                            flagFoundPort = true;
                        }
                    }
                    else if (QDateTime::currentMSecsSinceEpoch() - lastTime > 500)
                    {
                        NumCom++;
                        break;
                    }
                    if(flagFoundPort) break;
                    QThread::msleep(20);
                }
                if(!flagFoundPort) emit ComportClose_signal();
                flag = false;
                if(GLB_Comports.length() == (NumCom + 1))
                {
                    break;
                }
                else if(flagFoundPort) break;
            }
        }
        GLB_Thread_Flag[1] = false;
    }
}
