#include "MyThread.h"
#include "protezcommand.h"
bool FLAG_1;
bool FLAG_2; QString PortName; qint32 BaudRate;
bool FLAG_3;
bool FLAG_4; std::vector<uint8_t> DataToSend;
bool FLAG_5;

extern QVector<uint8_t> GLB_RecvRowData;

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

void MyThread_1::on_ComportSearch() {
    FLAG_1 = true;
}

void MyThread_1::on_ComportConnect(QString portName, qint32 baudRate)
{
    PortName = portName;
    BaudRate = baudRate;
    FLAG_2 = true;
}
void MyThread_1::on_ComportClose()
{
    FLAG_3 = true;
}
void MyThread_1::on_ComportWrite(std::vector<uint8_t> data)
{
    DataToSend = data;
    FLAG_4 = true;
}
void MyThread_1::on_ComportStartRead()
{

    FLAG_5 = true;
}


QList<QString> MyThread_1::ComPortSearch()
{

    /*Данный вариант быстрее работает так как выделяет сразу те порты, с которыми можно работать
      Но не все имеют возможность работать адекватно.
    */
    if (serialPort) {
        if (serialPort->isOpen())
            emit signal_ComportCloseBack(serialPort->portName());
        serialPort->close();
        delete serialPort;
        serialPort = nullptr;
    }
    QList<QString> ComportList;
    ComportList.append("Select comport...");
    const auto ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : ports) {
        ComportList.append(info.portName());
    }
    // emit signal_ComportSearch(ComportList);
    return ComportList;

    /*В свою очередь этот - ищет долго, но выделяет, те с которыми нет проблем.
    */
    // QList<QString> ComportList;
    // ComportList.append("Select comport...");
    // for(uint8_t i = 1; i < 15; i++)
    // {
    //     QString portName = QString("COM%1").arg(i);
    //     QSerialPortInfo ComPortInfo(portName);
    //     if (ComPortInfo.isValid()) ComportList.append(portName);
    // }
    // return ComportList;
}

bool MyThread_1::ComPortConnect(const QString &portName, qint32 baudRate)
{
    if (serialPort) {
        if (serialPort->isOpen())
            emit signal_ComportCloseBack(serialPort->portName());
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
    return true;
}

void MyThread_1::ComPortClose()
{
    if (serialPort && serialPort->isOpen()) {
        serialPort->close();
        emit portClosed();
    }
}

QString MyThread_1::ComPortWrite(std::vector<uint8_t> data)
{
    if (!serialPort || !serialPort->isOpen())
        return "Port not opened";

    // qint64 bytesWritten = serialPort->write((char*)&data, data.size());
    qint64 bytesWritten = serialPort->write(reinterpret_cast<const char*>(data.data()), data.size());
    if (bytesWritten == -1) {
        return QString("Write error: %1").arg(serialPort->errorString());
    } else if (bytesWritten != (qint64)data.size()) {
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
        else if (FLAG_3)
        {
            ComPortClose();
            if (serialPort)
                emit signal_ComportCloseBack(serialPort->portName());

            FLAG_3 = false;
        }
        else if (FLAG_4)
        {
            ComPortWrite(DataToSend);
            emit signal_ComportWriteBack();
            DataToSend.clear();
            FLAG_4 = false;
        }
        else if (FLAG_5)
        {
            ComPortRead();
            FLAG_5 = false;
        }
    }
}

// void MyThread_1::onReadyRead()
// {
//     if (!serialPort)
//         return;
//     QByteArray data = serialPort->readAll();

//     while (serialPort->waitForReadyRead(10)) {
//         if ()
//         data += serialPort->readAll();

//     }
//     qInfo() << "RECV:";
//     qInfo() << data;
//     emit dataReceived(data);
// }


// ProtezCommand::GlobalDataRecv
// void MyThread_1::ComPortRead()
// {
//     if (!serialPort)
//         return;

//     if (serialPort->waitForReadyRead(10)) {
//         QByteArray data = serialPort->read(2);

//         while (data.size() >= 2)
//         {
//             data = serialPort->read(2);
//             if ((uint8_t)data[0] == PR_PROTOCOL_ADC_PACK_START.first && (uint8_t)data[1] == PR_PROTOCOL_ADC_PACK_START.second)
//             {
//                 qInfo() << "RECV: START PACK";
//             }
//             else if ((uint8_t)data[0] == PR_PROTOCOL_ADC_PACK_STOP.first && (uint8_t)data[1] == PR_PROTOCOL_ADC_PACK_STOP.second)
//             {
//                 qInfo() << "RECV: STOP PACK";
//             }
//             else /*если флаг в старте установился*/
//             {

//             }
//         }
//         // qInfo() << "ProtezCommand::GlobalDataRecv after processing: ";
//         // for (unsigned char byte : ProtezCommand::GlobalDataRecv) {
//         //     qInfo() << static_cast<int>(byte) << " ";
//         // }
//     }
// }

void MyThread_1::ComPortRead()
{
    if (!serialPort)
        return;

    QVector<uint8_t> GLB_RecvData;
    QByteArray newData;
    uint32_t c_data = 0;
    qint64 lastTime = QDateTime::currentMSecsSinceEpoch();
    while(1)
    {
        if(serialPort->waitForReadyRead(1000))
        {
            newData = serialPort->readAll();
            while(serialPort->waitForReadyRead(500))
            {
                newData += serialPort->readAll();
                lastTime = QDateTime::currentMSecsSinceEpoch();
            }
            for (auto &byte : newData)
            {
                GLB_RecvData.append(static_cast<uint8_t>(byte));
                c_data++;
            }
            break;
        }
        else
        {
            GLB_RecvData.clear();
            newData.clear();
        }
        if(QDateTime::currentMSecsSinceEpoch() - lastTime > 2000)
        {
            break;
        }
    }

    if(GLB_RecvData.isEmpty()) return;
    else {
        bool find_ADC_data = false;
        bool find_Feedback_data = false;



        uint8_t last_byte = 0;
        GLB_RecvRowData.clear();


        for (auto &dat : GLB_RecvData) {
            if (!find_Feedback_data)
            {
                if (last_byte == PR_PROTOCOL_PACK_ADC_START.first && dat == PR_PROTOCOL_PACK_ADC_START.second) {
                    find_ADC_data = true;
                    qInfo() << "start adc in pack";
                }
                else if (last_byte == PR_PROTOCOL_PACK_ADC_STOP.first && dat == PR_PROTOCOL_PACK_ADC_STOP.second) {
                    find_ADC_data = false;
                    qInfo() << "stop adc in pack";

                    if(!GLB_RecvRowData.isEmpty()) emit signal_PaintADC();
                }
                else if (find_ADC_data)
                {
                    GLB_RecvRowData.push_back(dat);
                }
            }
            else if (!find_ADC_data)
            {
                if (last_byte == PR_PROTOCOL_PACK_FEEDBACK_START.first && dat == PR_PROTOCOL_PACK_FEEDBACK_START.second)
                {
                    find_Feedback_data = true;
                    qInfo() << "Find start FeedBack Data";
                }
                else if (last_byte == PR_PROTOCOL_PACK_FEEDBACK_STOP.first && dat == PR_PROTOCOL_PACK_FEEDBACK_STOP.second)
                {
                    GLB_RecvRowData.pop_back();
                    find_Feedback_data = false;
                    qInfo() << "Find stop FeedBack Data";

                    if(!GLB_RecvRowData.isEmpty()) emit signal_PaintFeedBack();
                }
                else if (find_Feedback_data)
                {
                    GLB_RecvRowData.push_back(dat);
                }
            }
            last_byte = dat;
        }
        if (!GLB_RecvRowData.empty()) {
            GLB_RecvRowData.pop_back();
        }
        qInfo() << "Count data: " << c_data;
    }
    return /*GLB_RecvData*/;
}










