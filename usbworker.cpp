#include "usbworker.h"
#include <QDebug>
#include <libusb.h>
#include <QThread>

UsbWorker::UsbWorker(QObject *parent)
    : QObject(parent), pHandle(nullptr){}

UsbWorker::~UsbWorker(){closeUSB(); }

void UsbWorker::initializeUSB()
{
    QMutexLocker locker(&mutex);
    int nRet;
    nRet = libusb_init(NULL);
    if (nRet < 0) {
        emit errorOccurred("初始化失败");
        return;
    }
    pHandle = libusb_open_device_with_vid_pid(NULL, 0x4B4, 0x00F1);
    if (pHandle == NULL) {
        emit errorOccurred("打开USB设备失败");
        libusb_exit(NULL);
        return;
    }

    nRet = libusb_claim_interface(pHandle, 0);
    if (nRet < 0) {
        libusb_close(pHandle);
        libusb_exit(NULL);
        return;
    }
}

void UsbWorker::closeUSB()
{
    QMutexLocker locker(&mutex);

    if (pHandle) {
        libusb_release_interface(pHandle, 0);
        libusb_close(pHandle);
    }
    libusb_exit(NULL);
}

void UsbWorker::sendCommand(const uint8_t* command, int size)
{
    QMutexLocker locker(&mutex);

    if (!pHandle) {
        emit errorOccurred("USB设备未打开");
        return;
    }

}

bool UsbWorker::sendBulkCommand(const uint8_t* command, int size)
{
    int len = size;
    int nRet = libusb_bulk_transfer(pHandle, 0x01, const_cast<uint8_t*>(command), len, &len, 10);
    return nRet >= 0;
}

bool UsbWorker::receiveBulkData(uint8_t* buffer, int size)
{
    int len = size;
    int nRet = libusb_bulk_transfer(pHandle, 0x81, buffer, len, &len, 1000);
    return nRet == 0;
}
void UsbWorker::on_buttonExposure_clicked()
{
    QMutexLocker locker(&mutex);

    if (!pHandle) {
        emit errorOccurred("USB设备未打开");
        return;
    }

    // 使用当前设置的曝光时间
    uint32_t exposureTime = currentExposureTime;
    exposureTime *= 5; // 转换为设备所需的单位

    CmdOutBuf[0] = exposureTime & 0xFF;
    CmdOutBuf[1] = (exposureTime >> 8) & 0xFF;
    CmdOutBuf[2] = (exposureTime >> 16) & 0xFF;
    CmdOutBuf[3] = 0x02; // 设置曝光时间命令码

    // 发送设置曝光时间命令
    if (!sendBulkCommand(CmdOutBuf, 4)) {
        return;
    }

    // 接收响应
    if (!receiveBulkData(DataInBuf, 4) ||
            !(DataInBuf[0] == 0xf0 && DataInBuf[1] == 0xff &&
              DataInBuf[2] == 0xf0 && DataInBuf[3] == 0xff)) {
        return;
    }

    // 启用曝光命令
    CmdOutBuf[0] = 0x03;
    CmdOutBuf[1] = 0x00;
    CmdOutBuf[2] = 0x00;
    CmdOutBuf[3] = 0x03; // 启用曝光命令码

    // 发送启用曝光命令
    if (!sendBulkCommand(CmdOutBuf, 4)) {
        return;
    }

    // 接收响应
    if (!receiveBulkData(DataInBuf, 4) ||
            !(DataInBuf[0] == 0xf0 && DataInBuf[1] == 0xff &&
              DataInBuf[2] == 0xf0 && DataInBuf[3] == 0xff)) {
        emit errorOccurred("启用曝光失败或响应不正确");
        return;
    }
    isExposureSet = true; // 标记曝光时间已成功设置
}

void UsbWorker::setExposureTime(int exposureTimeMs)
{
    QMutexLocker locker(&mutex);
    currentExposureTime = exposureTimeMs;
}
void UsbWorker::on_buttonUsbCapture_clicked()
{

    if (!isExposureSet) {
        on_buttonExposure_clicked();
        if (!isExposureSet) {
            emit captureCompleted(); // 曝光设置失败，发射信号并返回
            return;
        }
    }

    QMutexLocker locker(&mutex);

    if (!pHandle) {
        emit errorOccurred("USB设备未打开");
        emit captureCompleted(); // 设备未打开，发射信号并返回
        return;
    }

    int nRet, nActualBytes;
    uint8_t CmdOutBuf[4];
    QVector<double> ccd_data(1045); // 数据长度为1045
    CmdOutBuf[3] = 0x10;
    CmdOutBuf[2] = 0x00;
    CmdOutBuf[1] = 0x00;
    CmdOutBuf[0] = 0x00;

    nRet = libusb_bulk_transfer(pHandle, 0x01, CmdOutBuf, 4, &nActualBytes, 10);
    if (nRet < 0) {
        emit errorOccurred("发送指令失败");
        emit captureCompleted(); // 发送指令失败，发射信号并返回
        return;
    }

    QThread::msleep(100);//100

    // 接收数据
    nRet = libusb_bulk_transfer(pHandle, 0x81, DataInBuf, 4096, &nActualBytes, 1000);

    if (nRet == 0) {
        int j = 0;
        for (int i = 0; i < nActualBytes - 1; i += 2) {
            uint8_t lbyte = DataInBuf[i];       // 低字节
            uint8_t hbyte = DataInBuf[i + 1];   // 高字节
            uint16_t value = static_cast<uint16_t>(lbyte) | (static_cast<uint16_t>(hbyte) << 8);
            double ccd_value = static_cast<double>(value) / 1.0;
            if (ccd_value < 65535 && ccd_value != 0) {
                ccd_data[j] = ccd_value;
                j++;
            }
        }
        std::reverse(ccd_data.begin(), ccd_data.begin() + j);
        emit dataReceived(ccd_data); // 发射数据接收信号
    }

    emit captureCompleted();
}

