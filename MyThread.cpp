#include "MyThread.h"


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

extern LastTypeCommand LTC;

bool flag_com = false;

/*Port data variables*/
QVector<uint8_t> ComportdataRecv;
extern uint32_t ComportCountdataRecv;

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
    while(1)
    {
        if(flag_com == false)
        {
            serialDevice1 = new QSerialPort();
            qRegisterMetaType<QSerialPort::SerialPortError>("QSerialPort::SerialPortError");
            connect(serialDevice1, &QSerialPort::errorOccurred, this, &MyThread_1::ComPort_handleError);
            connect(serialDevice1, &QSerialPort::aboutToClose, this, &MyThread_1::onPortClosed);

            // connect(serialDevice1, &QSerialPort::readyRead, this, &MyThread_1::READDATA);
        }
        flag_com = true;

        if(TypeThreadInterrupt == 1)   // SEARCH COMPORT
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
            serialDevice1->clear(QSerialPort::Input);
            QString feddat = ComPortWrite(ComportDataToSend, ComportCountDataToSend);

            memset(ComportDataToSend, 0, ComportCountDataToSend);
            ComportCountDataToSend = 0;

            emit ComPortWrite_signal(feddat);
            TypeThreadInterrupt = 0;
        }
        else if(TypeThreadInterrupt == 5)    // READ COMPORT
        {
            if (serialDevice1->isOpen())
            {
                // GLB_Graph_y.clear();
                ComportdataRecv = ComPortReadData();
                GLB_Graph_y = ComportdataRecv;

                emit PaintGraph_signal();
            }
            if(TypeThreadInterrupt == 5) TypeThreadInterrupt = 0;
        }
    }
    qDebug() << "Thread #1 is disable!";
}

QVector<uint8_t> MyThread_1::ComPortReadData()
{
    QVector<uint8_t> GLB_RecvData;
    uint32_t cnt_dataRecvd = 0;
    QByteArray newData;
    ComportCountdataRecv = 0;
    qint64 lastTime = QDateTime::currentMSecsSinceEpoch();
    while(1)
    {
        if(serialDevice1->waitForReadyRead(300))
        {
            newData = serialDevice1->readAll();
            while(serialDevice1->waitForReadyRead(50))
            {
                newData += serialDevice1->readAll();
            }
            for (char byte : newData)
            {
                GLB_RecvData.append(static_cast<uint8_t>(byte));
                cnt_dataRecvd++;
            }
            break;
        }
        else
        {
            // GLB_RecvData.clear();
            // newData.clear();
        }
        if((QDateTime::currentMSecsSinceEpoch() - lastTime > 10000) || (TypeThreadInterrupt != 5))
        {
            break;
        }
    }
    ComportCountdataRecv = cnt_dataRecvd;
    // qDebug() <<"PACK Cnt: " << ComportCountdataRecv;
    return GLB_RecvData;
}

QList<QString> MyThread_1::ComPortSearch()
{
    uint8_t ii = 0;
    QList<QString> ComportList;
    ComportList.append("Select comport...");
    for(uint8_t i = 1; i < 15; i++)
    {
        QString portName = QString("COM%1").arg(i);
        QSerialPortInfo ComPortInfo(portName);
        if (ComPortInfo.isValid())
        {
            ComportList.append(portName);
            ii++;
        }
    }
    CurrentComPort = ComportList[0];
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
                QByteArray newData;
                qint64 lastTime = QDateTime::currentMSecsSinceEpoch();
                while(1)
                {
                    if(serialDevice1->waitForReadyRead(1000))
                    {
                        newData = serialDevice1->read(3);
                        while((!newData.isEmpty()) || (serialDevice1->waitForReadyRead(200)))
                        {
                            newData += serialDevice1->read(3);
                            if(newData.contains(QByteArray::fromHex("DDDDDD")))
                            {
                                isConnectedComPort = true;
                                CurrentComPort = trg_comports[i];
                                uint8_t dat[4] = {0x44, 0x44, 0x44, 0x44};
                                ComPortWrite(dat, 4);
                                return trg_comports[i];
                                break;
                            }
                            break;
                        }
                    }
                    if (QDateTime::currentMSecsSinceEpoch() - lastTime > 2000)
                    {
                        serialDevice1->close();
                        break;
                    }
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
    if(serialDevice1->isOpen())
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
        while(serialDevice1->waitForBytesWritten(10)) { }
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
void MyThread_1::ComPort_handleError(QSerialPort::SerialPortError error)
{
    switch (error)
    {
    case QSerialPort::NoError:
        // No Error
        break;
    case QSerialPort::DeviceNotFoundError:
        qDebug() << "DeviceNotFoundError";
        break;
    case QSerialPort::PermissionError:
        qDebug() << "PermissionError";
        break;
    case QSerialPort::OpenError:
        qDebug() << "OpenError";
        break;
    case QSerialPort::ParityError:
        qDebug() << "ParityError";
        break;
    case QSerialPort::FramingError:
        qDebug() << "FramingError";
        break;
    case QSerialPort::BreakConditionError:
        qDebug() << "BreakConditionError";
        break;
    case QSerialPort::WriteError:
        qDebug() << "WriteError";
        break;
    case QSerialPort::ReadError:
        qDebug() << "ReadError";
        break;
    case QSerialPort::ResourceError:
        qDebug() << "ResourceError";
        break;
    case QSerialPort::UnsupportedOperationError:
        qDebug() << "UnsupportedOperationError";
        break;
    case QSerialPort::UnknownError:
        qDebug() << "UnknownError";
        break;
    case QSerialPort::TimeoutError:
        qDebug() << "TimeoutError";
        break;
    case QSerialPort::NotOpenError:
        qDebug() << "NotOpenError";
        break;
    default:
        break;
    }
}
void MyThread_1::onPortClosed()
{
    qDebug() << "Port close";
}
void MyThread_1::onCheckConnect()
{
    if (serialDevice1->isOpen())
    {
        qDebug() << "Port disconnected.";
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
