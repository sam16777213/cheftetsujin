#include "CDobotConnector.h"
#include <QSerialPort>
#include <QSerialPortInfo>
#include "DobotType.h"
#include "DobotDevice/UdpSearch.h"
#include "DobotDevice/UdpPort.h"
#include <QThread>

CDobotConnector::CDobotConnector()
    :QObject(0),
    ioDevice(NULL)
{

}

CDobotConnector::~CDobotConnector()
{

}

void CDobotConnector::onInit(void)
{
    qDebug() << metaObject()->className() << ":" << QThread::currentThread();
}

//Q_SIGNAL void newConnectStatus(bool connected, QIODevice *ioDevice);

void CDobotConnector::onBytesToWrite(const char *data, qint64 maxSize)
{
    if (ioDevice) {
        ioDevice->write(data, maxSize);
    }
}

//Q_SIGNAL void bytesWritten(void);

void CDobotConnector::onBytesReady(void)
{
    emit bytesReady(ioDevice->readAll());
}

//Q_SIGNAL void bytesReady(QByteArray data);


void CDobotConnector::searchDobot(void *isFinishedAddr, void *resultAddr, void *dobotNameListAddr, unsigned int maxLen)
{
    bool *isFinished = (bool *)isFinishedAddr;
    int *result = (int *)resultAddr;

    char *dobotNameList = (char *)dobotNameListAddr;

    QList<QSerialPortInfo> availablePorts = QSerialPortInfo::availablePorts();
    QStringList udpPortList = UdpSearch::Instance(this)->SearchDobotDevice();
    dobotPortNameList.clear();

    foreach(QSerialPortInfo portInfo, availablePorts) {
        if (portInfo.description().contains("USB-to-Serial")
            || portInfo.description().contains("CH340")
            || portInfo.description().contains("CP210")
            || portInfo.description().contains("USB2.0-Serial")) {
            dobotPortNameList.push_back(portInfo.portName());
        }
    }
    // Now all the dobot informations are stored in dobotPortNameList
    do {
        if (dobotNameList == NULL) {
            break;
        }
        strcpy(dobotNameList, "");
        foreach(const QString &dobotStr, dobotPortNameList) {
            std::string stdString = dobotStr.toStdString();
            const char *portName = stdString.c_str();
            uint32_t nextLen = strlen(dobotNameList) + 1/*\0*/ + strlen(portName) + 1/* */;

            if (nextLen > maxLen) {
                break;
            }
            strcat(dobotNameList, portName);
            strcat(dobotNameList, " ");
        }

        for (int i = 0; i < udpPortList.size(); ++i){
            const char *idAddr = udpPortList.at(i).toLocal8Bit().constData();
            uint32_t nextLen = strlen(dobotNameList) + 1/*\0*/ + strlen(idAddr) + 1/* */;

            if (nextLen > maxLen) {
                break;
            }
            strcat(dobotNameList, idAddr);
            strcat(dobotNameList, " ");
        }

        // Trim the last blank
        if (dobotNameList[strlen(dobotNameList) - 1] == ' ') {
            dobotNameList[strlen(dobotNameList) - 1] = '\0';
        }
    } while (0);

    *result = dobotPortNameList.count() + udpPortList.count();
    *isFinished = true;
    return;
}

void CDobotConnector::connectDobot(void *isFinishedAddr, void *resultAddr, void *portNameAddr, unsigned int baudrate)
{
    bool *isFinished = (bool *)isFinishedAddr;
    int *result = (int *)resultAddr;

    const char *portName = (const char *)portNameAddr;

    QString connectPortName;
    if (portName == NULL || strcmp(portName, "") == 0) {
        bool _isFinished;
        int _result;
        searchDobot((void *)&_isFinished, (void *)&_result, 0, 0);
        if (dobotPortNameList.count() < 1) {
            *result = DobotConnect_NotFound;
            *isFinished = true;
            return;
        }
        connectPortName = dobotPortNameList.at(0);
    } else{
        connectPortName = QString(portName);
    }

    QRegExp regExpIP("((25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9][0-9]|[0-9])[\\.]){3}(25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9][0-9]|[0-9])");
    if (!regExpIP.exactMatch(connectPortName)) {
        QSerialPort *serialPort = new QSerialPort(connectPortName, this);
        serialPort->setBaudRate(baudrate);
        serialPort->setDataBits(QSerialPort::Data8);
        serialPort->setParity(QSerialPort::NoParity);
        serialPort->setStopBits(QSerialPort::OneStop);
        ioDevice = serialPort;
    } else {
        UdpPort *udpPort = new UdpPort(this);
        udpPort->setAddr(connectPortName);
        ioDevice = udpPort;
    }
    if (ioDevice->open(QIODevice::ReadWrite) == false) {
        ioDevice->deleteLater();
        ioDevice = 0;
        // Serial port is occupied!
        *result = DobotConnect_Occupied;
        *isFinished = true;
        return;
    }
    connect(ioDevice, SIGNAL(bytesWritten(qint64)), this, SIGNAL(bytesWritten()));
    connect(ioDevice, SIGNAL(readyRead()), this, SLOT(onBytesReady()));

    emit newConnectStatus(true);

    *result = DobotConnect_NoError;
    *isFinished = true;
    return;
}

void CDobotConnector::disconnectDobot(void *isFinishedAddr, void *resultAddr)
{
    bool *isFinished = (bool *)isFinishedAddr;
    int *result = (int *)resultAddr;

    emit newConnectStatus(false);
    if (ioDevice) {
        ioDevice->close();
        ioDevice->deleteLater();
        ioDevice = 0;
    }

    *result = DobotConnect_NoError;
    *isFinished = true;
    return;
}
