#ifndef DATAWORKER_H
#define DATAWORKER_H

#include <QObject>
#include <QVector>
#include <QMutex>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <QPointF>
#include <QDebug>
#include <QMutex>
#include <fftw3.h>
#include <QThread>
#include <QElapsedTimer>
#include <QtGlobal>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <vector>

struct SmoothConfig;
struct BaselineConfig;

class DataWorker : public QObject
{
    Q_OBJECT
public:
    explicit DataWorker(QObject *parent = nullptr);
    ~DataWorker();

    /*-------- 平滑算法 --------*/
    QVector<double> performSmoothing(const QVector<double> &data);                ///< 主入口：自适应窗口 + FFT 低通 + SG 保护峰
    int  calculateAdaptiveWindowSize(const QVector<double>& data, int index);     ///< 根据局部方差返回 3~6 点窗口
    QVector<double> performConservativeFFTWFiltering(const QVector<double>& data);///< FFT 硬阈值低通，保留 ≤ 12.5 % 高频
    void setSmoothConfig(const SmoothConfig &cfg);                                ///< 线程安全更新平滑参数
    SmoothConfig smoothConfig() const;                                            ///< 线程安全读取平滑参数

    /*-------- 基线算法 --------*/
    QVector<double> performBaselineCorrection(const QVector<double> &data);       ///< 主入口：ALS + 非零保护
    void setBaselineConfig(const BaselineConfig &cfg);                            ///< 线程安全更新基线参数
    BaselineConfig baselineConfig() const;                                        ///< 线程安全读取基线参数
    QVector<double> ensureNonZeroResult(const QVector<double>& original,
                                        const QVector<double>& corrected);        ///< 防止过度校正导致零/负值
    Eigen::VectorXd computeBaselineALS(const Eigen::VectorXd &y,
                                       double lambda,
                                       double p,
                                       int maxIter);


signals:
    void dataProcessed(const QVector<double>& yProcessed,
                       const QVector<double>& xOriginal); ///< 处理完成主出口
    void peakProcessed(const QVector<double> &data);      ///< 预留：峰形拟合结果
    void peaksFound(const QVector<QPointF> &peaks);       ///< 预留：峰位列表
    void errorOccurred(const QString &msg);               ///< 算法异常提示
    void baselineSkipped(const QString &info);            ///< 基线被跳过提示
    void smoothElapsedUs(qint64 us);                      ///< 平滑耗时（μs）
    void baselineElapsedUs(qint64 us);                    ///< 去基线耗时（μs）

public slots:
    void smoothData(const QVector<double>& y, const QVector<double>& x);      ///< 槽：请求平滑
    void removeBaseline(const QVector<double>& y, const QVector<double>& x);  ///< 槽：请求去基线

private:
    mutable QMutex m_mutex;           ///< 保护参数与调试标志
    SmoothConfig   *m_smoothCfg;      ///< 平滑参数指针（线程安全读写）
    BaselineConfig *m_baselineCfg;    ///< 基线参数指针（线程安全读写）
};



class SmoothConfig
{
public:
    int  windowLen = 5;   ///< 局部 SG 窗口大小（奇数）
    int  polyOrder = 2;   ///< SG 多项式阶数
    bool checkFinite = true; ///< 是否提前检查非有限值

    void adjustWindowLen() { if (windowLen % 2 == 0) windowLen += 1; }
};


struct BaselineConfig
{
    double lambda = 10000.0; ///< 平滑强度（↑ 越平滑）
    double p = 0.001;        ///< 不对称权重（↑ 越贴近下包络）
    int maxIter = 20;        ///< 最大迭代次数
};

#endif // DATAWORKER_H
