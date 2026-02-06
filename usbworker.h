#ifndef USBWORKER_H
#define USBWORKER_H

#include <QObject>
#include <libusb.h>
#include <QVector>
#include <QMutex>

/**
 * @brief UsbWorker 类用于管理与 USB 设备的通信。
 *
 * 该类封装了 USB 设备的初始化、命令发送、数据接收和关闭等操作。
 * 它通过信号与槽机制与其他模块（如主线程）进行通信。
 */
class UsbWorker : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief 构造函数，初始化 UsbWorker 对象。
     * @param parent 父对象，用于 QObject 的继承机制。
     */
    explicit UsbWorker(QObject *parent = nullptr);

    /**
     * @brief 析构函数，负责清理资源。
     */
    ~UsbWorker();

signals:
    /**
     * @brief 当数据采集完成时发出的信号。
     * @param data 采集到的数据。
     */
    void dataReceived(const QVector<double>& data);

    /**
     * @brief 当发生错误时发出的信号。
     * @param message 错误信息。
     */
    void errorOccurred(const QString& message);

    /**
     * @brief 当数据采集完成时发出的信号。
     */
    void captureCompleted();

public slots:
    /**
     * @brief 初始化 USB 设备。
     *
     * 包括打开设备、声明接口等操作。
     */
    void initializeUSB();

    /**
     * @brief 关闭 USB 设备。
     *
     * 释放接口并退出 libusb。
     */
    void closeUSB();

    /**
     * @brief 发送命令到 USB 设备。
     * @param command 命令数据。
     * @param size 命令数据的大小。
     */
    void sendCommand(const uint8_t* command, int size);

    /**
     * @brief 触发 USB 数据采集。
     */
    void on_buttonUsbCapture_clicked();

    /**
     * @brief 设置曝光时间并启用曝光。
     */
    void on_buttonExposure_clicked();

    /**
     * @brief 设置曝光时间。
     * @param exposureTimeMs 曝光时间（毫秒）。
     */
    void setExposureTime(int exposureTimeMs);

private:
    /**
     * @brief USB 设备句柄。
     */
    libusb_device_handle* pHandle;

    /**
     * @brief 命令输出缓冲区。
     */
    uint8_t CmdOutBuf[10];

    /**
     * @brief 数据输入缓冲区。
     */
    uint8_t DataInBuf[4096];

    /**
     * @brief 标记曝光时间是否已成功设置。
     */
    bool isExposureSet = false;

    /**
     * @brief 互斥锁，用于线程安全。
     */
    QMutex mutex;

    /**
     * @brief 当前曝光时间（毫秒）。
     */
    int currentExposureTime = 1000;

    /**
     * @brief 打开 USB 设备。
     * @return 成功返回 true，失败返回 false。
     */
    bool openDevice();

    /**
     * @brief 关闭 USB 设备。
     */
    void closeDevice();

    /**
     * @brief 发送批量命令到 USB 设备。
     * @param command 命令数据。
     * @param size 命令数据的大小。
     * @return 成功返回 true，失败返回 false。
     */
    bool sendBulkCommand(const uint8_t* command, int size);

    /**
     * @brief 从 USB 设备接收批量数据。
     * @param buffer 数据缓冲区。
     * @param size 缓冲区大小。
     * @return 成功返回 true，失败返回 false。
     */
    bool receiveBulkData(uint8_t* buffer, int size);
};

#endif // USBWORKER_H
