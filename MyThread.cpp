#include "MyThread.h"

bool FLAG_1;
bool FLAG_2; QString PortName; qint32 BaudRate;

MyThread_1::MyThread_1(QObject *parent)
    : QThread(parent)
{

}

MyThread_1::~MyThread_1()
{
    if (serialPort) {
        if (serialPort->isOpen())
            serialPort->close();
        delete serialPort;
        serialPort = nullptr;
    }
    qDebug() << "Thread1 is killed!";
}

void MyThread_1::on_ComPortSearch() {
    FLAG_1 = true;
}

void MyThread_1::on_ComPortConnect(QString portName, qint32 baudRate) {
    PortName = portName;
    BaudRate = baudRate;
    FLAG_2 = true;
}



QList<QString> MyThread_1::ComPortSearch()
{

    /*Данный вариант быстрее работает так как выделяет сразу те порты, с которыми можно работать
      Но не все имеют возможность работать адекватно.
    */

    // QList<QString> ComportList;
    // ComportList.append("Select comport...");
    // const auto ports = QSerialPortInfo::availablePorts();
    // for (const QSerialPortInfo &info : ports) {
    //     ComportList.append(info.portName());
    // }
    // emit signal_ComportSearch(ComportList);
    // return ComportList;

    /*В свою очередь этот - ищет долго, но выделяет, те с которыми нет проблем.
    */
    QList<QString> ComportList;
    ComportList.append("Select comport...");
    for(uint8_t i = 1; i < 15; i++)
    {
        QString portName = QString("COM%1").arg(i);
        QSerialPortInfo ComPortInfo(portName);
        if (ComPortInfo.isValid()) ComportList.append(portName);
    }
    return ComportList;
}

bool MyThread_1::ComPortConnect(const QString &portName, qint32 baudRate)
{
    if (serialPort) {
        if (serialPort->isOpen())
            serialPort->close();
        delete serialPort;
        serialPort = nullptr;
    }

    serialPort = new QSerialPort(portName);

    serialPort->setBaudRate(baudRate);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (!serialPort->open(QIODevice::ReadWrite)) {
        emit errorOccurred(QString("Failed to open port %1, error: %2").arg(portName, serialPort->errorString()));
        delete serialPort;
        serialPort = nullptr;
        return false;
    }
    connect(serialPort, &QSerialPort::readyRead, this, &MyThread_1::onReadyRead);
    return true;
}

void MyThread_1::ComPortClose()
{
    if (serialPort && serialPort->isOpen()) {
        serialPort->close();
        emit portClosed();
    }
}

QString MyThread_1::ComPortWrite(const uint8_t *data, uint32_t cntdata)
{
    if (!serialPort || !serialPort->isOpen())
        return "Port not opened";

    qint64 bytesWritten = serialPort->write(reinterpret_cast<const char*>(data), cntdata);
    if (bytesWritten == -1) {
        return QString("Write error: %1").arg(serialPort->errorString());
    } else if (bytesWritten != cntdata) {
        return QString("Could not write all data to port");
    }

    if (!serialPort->waitForBytesWritten(1000)) {
        return QString("Timeout while writing data");
    }

    return "OK";
}

void MyThread_1::run()
{
    while (!stopThread) {
        if (FLAG_1)
        {
            QList<QString> ports = ComPortSearch();
            emit signal_ComportSearchBack(ports);
            FLAG_1 = false;
        }
        else if (FLAG_2)
        {
            if (ComPortConnect(PortName, BaudRate))
            {
                emit signal_ComportConnectBack(PortName);
                PortName = ""; BaudRate = 0;
                FLAG_2 = false;
            }
            else if (PortName == "Select comport...")
            {
                emit signal_ComportConnectBack(PortName);
                PortName = ""; BaudRate = 0;
                FLAG_2 = false;
            }
        }
    }
}

void MyThread_1::onReadyRead()
{
    if (!serialPort)
        return;

    QByteArray data = serialPort->readAll();
    while (serialPort->waitForReadyRead(10))
        data += serialPort->readAll();

    emit dataReceived(data);
}
