#include "MyThread.h"


extern bool GLB_Thread_Flag[2];
extern QSerialPort *GLB_Ports[2];
extern uint32_t PackToRecv;
extern uint32_t GLB_I;

extern QList<QString> GLB_Comports;
extern uint8_t TypeThreadInterrupt;
extern QString CurrentComPort;
extern uint32_t CurrentBoundRate;
extern bool isConnectedComPort;
extern bool ThreadAutoConnectState;
extern uint8_t ComportDataToSend[50];
extern uint8_t ComportCountDataToSend;

bool flag_com = false;

/*Port data variables*/
QVector<uint8_t> dataRecvComport;
uint32_t ComportCountdataRecv;

/*Graph variables*/
extern QVector<uint8_t> GLB_Graph_y;

/*Thread 1*/
MyThread_1::MyThread_1(MainWindow* mainWindowInstance1, QObject *parent) : QThread(parent), mainWindow(mainWindowInstance1)
{

}
MyThread_1::~MyThread_1()
{
    qDebug() << "Thread1 is killed!";

    if (serialDevice1->isOpen())
    {
        serialDevice1->close();
    }

    delete serialDevice1;
}
void MyThread_1::run()
{
    if(flag_com == false)
    {
        serialDevice1 = new QSerialPort();
    }
    flag_com = true;

    if(TypeThreadInterrupt == 0)    // READ COMPORT
    {
        if (serialDevice1->isOpen())
        {
            dataRecvComport.clear();
            while(GLB_Thread_Flag[0])
            {
                dataRecvComport = ComPortRead();
            }
            //
            GLB_Graph_y.clear();
            GLB_Graph_y = dataRecvComport;
            //
            if(!GLB_Graph_y.isEmpty())
            {
                emit PaintGraph_signal(); // Сигнал для обновления графика
            }
            emit ComportRead_signal();
        }
        TypeThreadInterrupt = 0;
    }
    else if(TypeThreadInterrupt == 1)   // SEARCH COMPORT
    {
        if(ThreadAutoConnectState) ComPortFoundPort();
        else ComPortSearch();

        TypeThreadInterrupt = 0;
        emit ComportDataUpdate_signal();
    }
    else if(TypeThreadInterrupt == 2)   // CONNECT TO COMPORT
    {
        ComPortConnect();

        emit ComportConnect_signal();
        TypeThreadInterrupt = 0;
    }
    else if(TypeThreadInterrupt == 3)   // CLOSE COMPORT
    {
        ComPortClose();

        emit ComportClose_signal();
        TypeThreadInterrupt = 0;
    }
    else if(TypeThreadInterrupt == 4)   // COMPORT WRITE
    {
        QString feddat = ComPortWrite(ComportDataToSend, ComportCountDataToSend);

        memset(ComportDataToSend, 0, ComportCountDataToSend);
        ComportCountDataToSend = 0;

        emit ComPortWrite_signal(feddat);
        TypeThreadInterrupt = 0;
    }
    qDebug() << "Thread #1 is disable!";
}

QVector<uint8_t> MyThread_1::ComPortRead()
{
    QVector<uint8_t> GLB_RecvData;
    uint32_t cnt_dataRecvd = 0;
    bool flagHaveData = false;
    serialDevice1->waitForReadyRead(10);
    qint64 lastTime = QDateTime::currentMSecsSinceEpoch();
    while (1)
    {
        QByteArray newData = serialDevice1->readAll();
        if (!newData.isEmpty())
        {
            for (char byte : newData) {
                GLB_RecvData.append(static_cast<uint8_t>(byte));
                cnt_dataRecvd++;
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

    ComportCountdataRecv = cnt_dataRecvd;
    GLB_Thread_Flag[0] = false;
    serialDevice1->clear();
    return GLB_RecvData;

}
QList<QString> MyThread_1::ComPortSearch()
{
    uint8_t ii = 0;
    QList<QString> ComportList;
    ComportList.append("Select comport...");
    for(uint8_t i = 1; i < 10; i++)
    {
        QString portName = QString("COM%1").arg(i);
        QSerialPortInfo ComPortInfo(portName);
        if (ComPortInfo.isValid())
        {
            ComportList.append(portName);
            ii++;
        }
    }
    GLB_Comports = ComportList;
    return ComportList;
}
QString MyThread_1::ComPortFoundPort()
{
    QList<QString> trg_comports = ComPortSearch();

    if(serialDevice1->isOpen())
    {
        serialDevice1->close();
    }
    for(uint8_t i = 0; i < trg_comports.length(); i++)
    {
        if(trg_comports[i] != "Select comport...")
        {
            serialDevice1->setPortName(trg_comports[i]);
            serialDevice1->setBaudRate(CurrentBoundRate);
            serialDevice1->setParity(QSerialPort::NoParity);
            serialDevice1->setDataBits(QSerialPort::Data8);
            serialDevice1->setStopBits(QSerialPort::OneStop);
            serialDevice1->setFlowControl(QSerialPort::NoFlowControl);
            serialDevice1->open(QIODevice::ReadWrite);

            if(serialDevice1->isOpen())
            {
                QByteArray receivedData;
                qint64 lastTime = QDateTime::currentMSecsSinceEpoch();
                serialDevice1->waitForReadyRead(500);

                while(1)
                {
                    QByteArray newData = serialDevice1->readAll();
                    if(!newData.isEmpty())
                    {
                        receivedData.append(newData);
                        if(receivedData.contains(QByteArray::fromHex("FFFFFF")))
                        {
                            isConnectedComPort = true;
                            CurrentComPort = trg_comports[i];

                            return trg_comports[i];
                            break;
                        }
                    }
                    if (QDateTime::currentMSecsSinceEpoch() - lastTime > 1000)
                    {
                        serialDevice1->close();
                        break;
                    }
                    QThread::msleep(200);

                }
            }
        }
    }
    return "";
}
bool MyThread_1::ComPortConnect()
{
    QSerialPortInfo ComPortInfo(CurrentComPort);

    if(isConnectedComPort) ComPortClose();
    else if(CurrentComPort != "Select comport...") emit ComportClose_signal();
    if(ComPortInfo.isValid() && (CurrentComPort != "Select comport..."))
    {
        if(CurrentComPort != "Select comport...")
        {
            serialDevice1->setPortName(CurrentComPort);
            serialDevice1->setBaudRate(CurrentBoundRate);
            serialDevice1->setParity(QSerialPort::NoParity);
            serialDevice1->setDataBits(QSerialPort::Data8);
            serialDevice1->setStopBits(QSerialPort::OneStop);
            serialDevice1->setFlowControl(QSerialPort::NoFlowControl);
            serialDevice1->open(QIODevice::ReadWrite);

            if(serialDevice1->isOpen())
            {
                qDebug() << "Port connect to " + CurrentComPort;
            }
            isConnectedComPort = true;
            return 1;
        }
    }
    return 0;
}
void MyThread_1::ComPortClose()
{
    if (serialDevice1->isOpen())
    {
        isConnectedComPort = false;
        serialDevice1->close();
    }
    emit ComportClose_signal();
}

QString MyThread_1::ComPortWrite(uint8_t *data, uint32_t cntdata)
{
    if(serialDevice1->isOpen())
    {
        qint64 bytesWritten = serialDevice1->write((char*)data, cntdata);
        serialDevice1->waitForBytesWritten(100);
        if (bytesWritten == -1)
        {
            return "Failed to send data.";
        }
        else
        {
            return "Data is sent.";
        }
    }
    else
    {
        return "Please connect device.";
    }
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


}
