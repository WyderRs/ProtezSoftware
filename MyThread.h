#include <QThread>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <QWaitCondition>

class MyThread_1 : public QThread
{
    Q_OBJECT
public:
    explicit MyThread_1(QObject *parent = nullptr);
    ~MyThread_1() override;

    QList<QString> ComPortSearch();
    bool ComPortConnect(const QString &portName, qint32 baudRate = QSerialPort::Baud115200);
    void ComPortClose();

    QString ComPortWrite(const uint8_t *data, uint32_t cntdata);

signals:
    void dataReceived(const QByteArray &data);
    void portClosed();
    void errorOccurred(const QString &error);






    void signal_ComportSearchBack(QList<QString>);
    void signal_ComportConnectBack(QString);
    void signal_ComportCloseBack(QString);
public slots:
    void on_ComPortSearch();
    void on_ComPortConnect(QString, qint32 baudRate);

protected:
    void run() override;

private slots:
    void onReadyRead();

private:
    QSerialPort *serialPort = nullptr;
    bool stopThread = false;
};
