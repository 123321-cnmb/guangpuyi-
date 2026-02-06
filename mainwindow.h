#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QThread>
#include <QMainWindow>
#include <QPushButton>
#include <QChartView>
#include <QGridLayout>
#include <QVector>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QMessageBox>
#include <QFileDialog>
#include <QTextStream>
#include <QFile>
#include <QDebug>
#include "usbworker.h"
#include <QScatterSeries>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsProxyWidget>
#include <QInputDialog>
#include <QProcess>
#include <QtConcurrent>
#include <QProcess>
#include <algorithm>
#include <QLabel>
#include <QBarSeries>
#include <QBarSet>
#include <QBarCategoryAxis>
#include <QListWidgetItem>
#include <QComboBox>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QProgressDialog>
#include <QMutex>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <QPointF>
#include <fftw3.h>
#include <QElapsedTimer>
#include <numeric>
#include <cmath>
#include <vector>

QT_CHARTS_USE_NAMESPACE

// ==================== 参数结构体 ====================

/**
 * @struct SmoothConfig
 * @brief Savitzky-Golay + FFT 混合平滑配置
 *
 * 窗口长度须为奇数，内部提供 adjustWindowLen() 自动修正。
 */
struct SmoothConfig
{
public:
    int  windowLen = 5;   ///< 局部 SG 窗口大小（奇数）
    int  polyOrder = 2;   ///< SG 多项式阶数
    bool checkFinite = true; ///< 是否提前检查非有限值

    void adjustWindowLen() { if (windowLen % 2 == 0) windowLen += 1; }
};

/**
 * @struct BaselineConfig
 * @brief ALS 不对称最小二乘基线校正参数
 */
struct BaselineConfig
{
    double lambda = 10000.0; ///< 平滑强度（↑ 越平滑）
    double p = 0.001;        ///< 不对称权重（↑ 越贴近下包络）
    int maxIter = 20;        ///< 最大迭代次数
};

/**
 * @brief 自动对焦上下文结构体
 *
 * 用于存储自动对焦过程中的状态信息，包括是否首次对焦、
 * 移动方向、上次评分、最佳评分以及最佳位置等。
 */
struct AutoFocusContext {
    bool first = true;      ///< 是否为首次对焦
    bool up = true;         ///< 移动方向标志
    double lastScore = -1;  ///< 上次的对焦评分
    double bestScore = 0.0; ///< 最佳对焦评分
    int bestPosition = 0;   ///< 最佳位置对应的Z轴坐标
    bool peakFound = false;  // 新增：标记是否已找到峰值点
};

enum ClearMode {
    CLEAR_NORMAL,
    CLEAR_MAPPING
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    /*-------- 数据处理算法 --------*/
    QVector<double> performSmoothing(const QVector<double> &data);                ///< 主入口：自适应窗口 + FFT 低通 + SG 保护峰
    int  calculateAdaptiveWindowSize(const QVector<double>& data, int index);     ///< 根据局部方差返回 3~6 点窗口
    QVector<double> performConservativeFFTWFiltering(const QVector<double>& data);///< FFT 硬阈值低通，保留 ≤ 12.5 % 高频
    void setSmoothConfig(const SmoothConfig &cfg);                                ///< 线程安全更新平滑参数
    SmoothConfig smoothConfig() const;                                            ///< 线程安全读取平滑参数

    QVector<double> performBaselineCorrection(const QVector<double> &data);       ///< 主入口：ALS + 非零保护
    void setBaselineConfig(const BaselineConfig &cfg);                            ///< 线程安全更新基线参数
    BaselineConfig baselineConfig() const;                                        ///< 线程安全读取基线参数
    QVector<double> ensureNonZeroResult(const QVector<double>& original,
                                        const QVector<double>& corrected);        ///< 防止过度校正导致零/负值
    /**
         * @brief ALS 不对称最小二乘核心实现
         * @param y       输入光谱
         * @param lambda  平滑系数（越大越平滑）
         * @param p       不对称权重（0 < p < 1，通常 0.001）
         * @param maxIter 最大迭代次数
         * @return        估算出的基线向量
         */
    Eigen::VectorXd computeBaselineALS(const Eigen::VectorXd &y,
                                       double lambda,
                                       double p,
                                       int maxIter);

private slots:
    /* 界面按钮槽函数 */
    void resetZAxis();              ///< 重置Z轴位置到初始状态
    void on_buttonClearChart_clicked(); ///< 清空图表

    void on_saveOpen_clicked();        // 合并保存/打开按钮
    void on_savedraw_clicked();     ///< 保存当前绘图为图片
    void on_savedata_clicked();     ///< 保存当前数据到文件
    void on_findpeak_clicked();     ///< 寻峰功能
    void on_baseline_clicked();     ///< 去除基线
    void on_openfile_clicked();     ///< 打开外部数据文件
    void on_smooth_clicked();       ///< 数据平滑处理
    void on_autoFocus_clicked();    ///< 启动自动对焦
    void on_mapping_clicked();      ///< 启动Mapping扫描
    void on_analysis_clicked();  // 新增分析按钮的槽函数
    void on_buttonProcessSpectrum_clicked() ;
    void on_buttonSave_clicked();
    void on_buttonReset_clicked();
    void on_laser_clicked();  // 激光按钮点击事件
    void on_smooth_mapping_clicked();    // 批量平滑Mapping光谱
    void on_baseline_mapping_clicked();  // 批量去基线Mapping光谱
    void on_multiCapture_clicked();  // 新增的多重采集按钮

    /* Mapping相关槽函数 */
    void on_squareMapping_clicked();
    void startMapping(const QString &scriptPath, int totalSteps);
    void finishMapping();           ///< 完成Mapping扫描

    /* 数据接收与处理槽函数 */
    void handleDataReceived(const QVector<double>& data); ///< 处理USB接收到的原始数据
    void handleUsbError(const QString& message); ///< 处理USB错误信息
    void handleDataProcessed(const QVector<double>& yProcessed,
                             const QVector<double>& xOriginal); ///< 处理完成的数据

    /* 自动对焦状态机相关槽函数 */
    void runAutoFocusStateMachine(); ///< 自动对焦状态机主入口
    void finishAutoFocus();          ///< 完成自动对焦
    void handleCoarseSearch(double score, AutoFocusContext& ctx); ///< 粗搜索策略
    void moveToPosition(int targetPosition); ///< 移动Z轴到指定位置
    void calculateFinalBestPosition();       ///< 计算最终最佳位置
    void plotMappingSpectra();
    void saveRawDataToFile(const QVector<double>& xData, const QVector<double>& yData);

    /* 数据处理槽函数 */
    void smoothData(const QVector<double>& y, const QVector<double>& x);      ///< 槽：请求平滑
    void removeBaseline(const QVector<double>& y, const QVector<double>& x);  ///< 槽：请求去基线

signals:
    void afFirstFrameArrived(); ///< 自动对焦第一帧数据到达信号
    void requestSmooth(const QVector<double>& y, const QVector<double>& x); ///< 请求平滑处理
    void requestBaseline(const QVector<double>& y, const QVector<double>& x); ///< 请求去除基线
    void requestUsbClick(); ///< 请求触发USB采集
    void afDataReady();     ///< 自动对焦单步数据处理完成
    void mappingDataReady(); ///< Mapping单步数据处理完成
    void multiCaptureDataReady();  // 新增：多重采集专用信号

    /*-------- 数据处理信号 --------*/
    void dataProcessed(const QVector<double>& yProcessed,
                       const QVector<double>& xOriginal); ///< 处理完成主出口
    void peakProcessed(const QVector<double> &data);      ///< 预留：峰形拟合结果
    void peaksFound(const QVector<QPointF> &peaks);       ///< 预留：峰位列表
    void errorOccurred(const QString &msg);               ///< 算法异常提示
    void baselineSkipped(const QString &info);            ///< 基线被跳过提示
    void smoothElapsedUs(qint64 us);                      ///< 平滑耗时（μs）
    void baselineElapsedUs(qint64 us);                    ///< 去基线耗时（μs）

private:
    // 多重采集数据结构
    struct MultiCaptureData {
        QVector<QVector<double>> allSpectra;
        QVector<double> xData;
        int currentCount = 0;
        int totalCount = 0;
        int exposureTime = 1000;
        bool isRunning = false;
    };

    bool isMappingDataPresent = false;
    bool isMappingProcessed = false; // 标记是否进行了 mapping 处理

    MultiCaptureData m_multiCaptureData;
    QProgressDialog* m_multiCaptureProgressDialog = nullptr;

    // 新增成员函数
    void startNextMultiCapture();
    void onMultiCaptureDataReady();
    void finishMultiCapture();
    void displayAveragedSpectrum(const QVector<double>& spectrum, const QVector<double>& xData);
    void updateMultiCaptureProgress();
    void hideMultiCaptureProgress();
    void setAllButtonsEnabled(bool enabled);

    // 新增：批量处理相关函数
    bool prepareBatchData();                            ///< 准备批量处理数据
    void clearTempBatchData();                          ///< 清除临时批量数据
    void applyBatchProcessing();                        ///< 应用批量处理结果
    void checkAndClearTempData();                       ///< 检查并清除临时数据
    bool processSingleSpectrumInBatch(int index, bool isSmooth);  ///< 处理批量中的单个光谱
    bool validateData(const QVector<double>& yData, const QVector<double>& xData);
    void resetAllStates();  // 重置所有状态
    void ensureChartHasValidSeries();
    void rebuildChartCompletely(const QColor& lineColor, const QColor& scatterColor);

    /* 自动对焦控制参数 */
    bool m_silentMode = false;  ///< 是否静默模式（自动对焦/Mapping中屏蔽绘图）
    // 进度对话框相关
    QProgressDialog* m_progressDialog = nullptr;
    bool m_operationInProgress = false;
    AutoFocusContext m_afContext;  ///< 自动对焦上下文
    bool m_autoFocusRunning = false; ///< 自动对焦状态标志
    int m_afIteration = 0;      ///< 当前对焦迭代次数
    const int MAX_AF_ITERATIONS = 500; ///< 最大迭代次数，防止死循环

    bool m_fineSampling = false; ///< 是否处于精细采样阶段
    int m_samplingStep = 0;      ///< 精细采样当前步骤
    QVector<double> m_fineScores; ///< 精细采样评分缓存
    double m_bestScore = 0.0;    ///< 最佳评分
    int m_bestPosition = 0;      ///< 最佳位置

    /* Mapping控制参数 */
    bool m_mappingRunning = false; ///< Mapping运行标志
    int m_mappingPoints = 0;       ///< 已采集点数
    const int MAPPING_TOTAL_POINTS = 100; ///< 总采集点数
    QVector<double> m_mappingIntensities; ///< 各点峰值强度缓存
    QDialog* m_mappingDialog = nullptr;   ///< Mapping结果显示窗口

    /* UI成员 */
    Ui::MainWindow *ui; ///< UI界面指针
    QPushButton* exposureTimeButton;  // 曝光时间按钮
    int currentExposureTime = 1000;   // 当前曝光时间，默认1秒
    QPushButton* multiCaptureButton; // 新增按钮

    QPushButton* buttonUsbCapture;  ///< USB采集按钮
    QPushButton* buttonExposure; ///< 预留按钮
    QPushButton* buttonSaveOpen ;
    QPushButton* buttonProcessSpectrum ;
    QPushButton* buttonClearChart; ///< 清空图表按钮
    QPushButton* buttonSaveDraw;   ///< 保存绘图按钮
    QPushButton* buttonSaveData;   ///< 保存数据按钮
    QPushButton* buttonFindPeak;   ///< 寻峰按钮
    QPushButton* buttonBaseline;   ///< 去基线按钮
    QPushButton* buttonOpenFile;   ///< 打开文件按钮
    QPushButton* buttonSmooth;     ///< 平滑按钮
    QPushButton* autoFocus;        ///< 自动对焦按钮
    QPushButton* mappingButton;    ///< Mapping按钮
    QPushButton* buttonReset;
    QPushButton* laserButton;  // 激光按钮

    QSerialPort* serialPort;   // 串口对象
    QPushButton* analysisButton;  // 新增分析按钮

    /* 图表成员 */
    QChartView* chartView; ///< 图表视图
    QChart* chart;         ///< 图表对象
    QLineSeries* series;   ///< 主光谱曲线
    QValueAxis *xAxis, *yAxis, *topAxis, *rightAxis; ///< 坐标轴
    QGridLayout* gridLayout; ///< 布局管理器
    QLabel *zAxisLabel; ///< Z轴位置标签

    /* 工作线程 */
    UsbWorker* usbWorker; ///< USB通信工作对象
    QThread* usbThread;   ///< USB线程

    /* 图表标注与拟合 */
    QScatterSeries* peakSeries=nullptr; ///< 峰值散点系列
    QList<QGraphicsTextItem*> textItems; ///< 图表文本标注
    QList<QGraphicsTextItem*> peakLabels; ///< 峰值标签
    QLineSeries* fitSeries = nullptr; ///< 拟合曲线系列
    bool chearflag = false;  ///< 清除标志1
    bool chearflag2 = false; ///< 清除标志2

    /* Mapping数据缓存 */
    QVector<double> m_mappingWave;    ///< 波数轴（仅保存一次）
    int  m_mappingStep = 0;           ///< 当前Mapping步骤
    static constexpr int MAPPING_STEPS = 20; ///< 总Mapping步数
    QVector<QVector<double>> m_mappingSpectra; ///< 每张处理后的完整光谱
    QList<int> m_mappingPeaks; ///< 统一峰位索引（以第一张光谱为准）
    QVector<double> m_mappingAvg; ///< 平均光谱

    /* 新增：临时批量处理容器 */
    QVector<QVector<double>> m_tempBatchSpectra;  ///< 用于批量处理的临时副本
    QVector<double> m_tempBatchXData;             ///< 临时X数据
    bool m_isBatchDataReady = false;              ///< 批量数据是否已准备
    bool m_isBatchProcessing = false;             ///< 是否正在批量处理
    bool m_shouldClearTempOnNextOp = true;        ///< 下次操作时是否清除临时数据

    /* 数据处理参数 */
    mutable QMutex m_mutex;           ///< 保护参数与调试标志
    SmoothConfig   *m_smoothCfg;      ///< 平滑参数指针（线程安全读写）
    BaselineConfig *m_baselineCfg;    ///< 基线参数指针（线程安全读写）
    bool            m_debugMode;      ///< 调试开关（打印中间结果）

    /* 工具函数 */
    QPushButton* createButton(const QString& text, const QString& styleSheet); ///< 创建统一风格按钮
    void setupLayout(); ///< 初始化界面布局
    double gaussLorentz(int x, double x0, double sigma, double gamma, double amplitude); ///< 高斯-洛伦兹混合函数
    void gaussLorentzDetectPeaks(const QVector<double>& originalData, QList<int>& peaks,
                                 double sigma, double gamma, double amplitude); ///< 混合模型寻峰算法
    void executeMappingStep(const QString& script, int msDelay);
    void executeMappingStepWithProgress(const QString& script, int stepNum, int totalSteps, int msDelay);
    void gaussLorentzDetectPeaksForAF(const QVector<double>& originalData, QList<int>& peaks);
    void doSquare9Mapping();
    void performMultiCapture(int exposureTime, int captureCount); // 执行多重采集
    void setupSerialPort();    // 初始化串口
    void sendLaserCommand(bool isOn, int current);  // 发送激光指令
    QByteArray generateLaserCommand(int current);   // 生成激光指令
    void saveAFProcessData(int position, double score, const QVector<double>& spectrum);
    double calculateAFScore(const QVector<double>& data); // 评分函数
    QString m_afLogFileName; // 记录当前对焦实验的文件名
};

#endif // MAINWINDOW_H
