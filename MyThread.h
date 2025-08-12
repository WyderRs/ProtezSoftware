#include <QThread>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <QWaitCondition>
#include <QTimer>
#include <QDateTime>

class MyThread_1 : public QThread
{
    Q_OBJECT
public:
    explicit MyThread_1(QObject *parent = nullptr);
    ~MyThread_1() override;

    QList<QString> ComPortSearch();
    bool ComPortConnect(const QString &portName, qint32 baudRate = QSerialPort::Baud115200);
    void ComPortClose();

    QString ComPortWrite(std::vector<uint8_t>);
    void ComPortRead();
signals:
    void dataReceived(const std::vector<uint8_t> &data);
    void portClosed();
    void errorOccurred(const QString &error);






    void signal_ComportSearchBack(QList<QString>);
    void signal_ComportConnectBack(QString);
    void signal_ComportCloseBack(QString);
    void signal_ComportWriteBack();
    void signal_ComportReadBack();

    void signal_PaintGraph();
public slots:
    void on_ComportSearch();
    void on_ComportConnect(QString, qint32 baudRate);
    void on_ComportClose();
    void on_ComportWrite(std::vector<uint8_t>);
    void on_ComportStartRead();

protected:
    void run() override;
private:
    QSerialPort *serialPort = nullptr;
    bool stopThread = false;
};
