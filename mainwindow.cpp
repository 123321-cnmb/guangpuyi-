#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "usbworker.h"

// 全局变量，用于记录 up.sh 和 down.sh 的使用次数
int g_zAxisPosition = 0;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // 设置窗口大小
    this->resize(800, 480);
    setupLayout();

    // 创建USB通信线程
    usbThread = new QThread(this);
    usbWorker = new UsbWorker();
    usbWorker->moveToThread(usbThread);
    connect(usbThread, &QThread::started, usbWorker, &UsbWorker::initializeUSB);
    connect(usbWorker, &UsbWorker::errorOccurred, this, &MainWindow::handleUsbError);
    connect(usbWorker, &UsbWorker::dataReceived, this, &MainWindow::handleDataReceived);
    usbThread->start();

    m_smoothCfg = new SmoothConfig;
    m_baselineCfg = new BaselineConfig;
    m_debugMode = false;

    connect(this, &MainWindow::dataProcessed, this, &MainWindow::handleDataProcessed);

    connect(buttonUsbCapture, &QPushButton::clicked, usbWorker, &UsbWorker::on_buttonUsbCapture_clicked);
    connect(this, &MainWindow::requestSmooth, this, &MainWindow::smoothData, Qt::QueuedConnection);
    connect(this, &MainWindow::requestBaseline, this, &MainWindow::removeBaseline, Qt::QueuedConnection);
    connect(this, &MainWindow::requestUsbClick, usbWorker, &UsbWorker::on_buttonUsbCapture_clicked, Qt::QueuedConnection);
    connect(usbWorker, &UsbWorker::captureCompleted, this, &MainWindow::onMultiCaptureDataReady);

    setupSerialPort();
}


MainWindow::~MainWindow()
{
    // 先停止所有操作
    m_autoFocusRunning = false;
    m_multiCaptureData.isRunning = false;

    // 安全停止USB线程
    if (usbThread && usbThread->isRunning()) {
        usbThread->quit();
        usbThread->wait(1000); // 等待1秒
    }

    // 删除对象
    delete usbWorker;
    delete usbThread;
    delete serialPort; // 新增
    delete m_multiCaptureProgressDialog; // 新增
    delete m_smoothCfg;
    delete m_baselineCfg;

    if (peakSeries) {
        delete peakSeries;
    }
    delete ui;
}


// 平滑接口
void MainWindow::smoothData(const QVector<double>& y, const QVector<double>& x)
{
    if (y.size() < 3) {
        emit dataProcessed(y, x);   // 数据太短，原样返回
        return;
    }
    QVector<double> sm = performSmoothing(y);
    emit dataProcessed(sm, x);
}

// 去基线接口
void MainWindow::removeBaseline(const QVector<double> &data, const QVector<double>& x)
{
    QMutexLocker lock(&m_mutex);
    QElapsedTimer t;
    t.start();
    if (data.isEmpty()) {
        emit errorOccurred("Empty data for baseline removal.");
        return;
    }

    // 检查非有限值
    auto containsNonFinite = [](const QVector<double> &v) {
        return std::any_of(v.constBegin(), v.constEnd(), [](double x) { return !std::isfinite(x); });
    };

    if (containsNonFinite(data)) {
        emit errorOccurred("Data contains NaN/Inf.");
        return;
    }

    QVector<double> corrected = performBaselineCorrection(data);
    corrected = ensureNonZeroResult(data, corrected);

    // 找到平移后的最小值
    double minCorrectedValue = *std::min_element(corrected.begin(), corrected.end());

    // 平移数据，使得最小值为 0
    for (double& value : corrected) {
        value -= minCorrectedValue;
    }
    emit dataProcessed(corrected, x);
    emit baselineElapsedUs(t.nsecsElapsed() / 1000);
}

// 平滑主算法
QVector<double> MainWindow::performSmoothing(const QVector<double>& data) {
    int size = data.size();
    if (size < 3) return data;

    QVector<double> fftwFiltered = performConservativeFFTWFiltering(data);
    QVector<double> smoothed(size);

    for (int i = 0; i < size; ++i) {
        int adaptiveWindowSize = calculateAdaptiveWindowSize(data, i);

        double sum = 0.0;
        int count = 0;
        int start = std::max(0, i - adaptiveWindowSize / 2);
        int end = std::min(size - 1, i + adaptiveWindowSize / 2);

        for (int j = start; j <= end; ++j) {
            sum += fftwFiltered[j];
            count++;
        }

        double originalValue = data[i];
        double smoothedValue = sum / count;

        // 动态平滑系数：基于原始值和平滑值的差异
        double difference = std::abs(originalValue - smoothedValue);
        double maxAllowedDrop = originalValue * 0.5; // 最多允许降低50%

        // 如果平滑会导致过度降低，调整平滑程度
        double alpha = 0.7;
        if (smoothedValue < originalValue - maxAllowedDrop) {
            alpha = 0.9; // 更倾向于保留原始值
        }

        smoothed[i] = alpha * originalValue + (1 - alpha) * smoothedValue;

        // 最终保护：确保不会过度降低
        if (originalValue > 0 && smoothed[i] < originalValue * 0.3) {
            smoothed[i] = originalValue * 0.3;
        }
    }

    return smoothed;
}

// 自适应窗口大小计算
int MainWindow::calculateAdaptiveWindowSize(const QVector<double>& data, int index) {
    int baseWindowSize = 3; // 基础窗口大小
    int maxWindowSize = 6;  // 最大窗口大小

    // 计算局部方差，方差大的区域使用小窗口（保护峰值）
    int localRange = 2;
    double localMean = 0.0;
    int count = 0;

    for (int i = std::max(0, index - localRange); i <= std::min(data.size()-1, index + localRange); ++i) {
        localMean += data[i];
        count++;
    }
    localMean /= count;

    double localVariance = 0.0;
    for (int i = std::max(0, index - localRange); i <= std::min(data.size()-1, index + localRange); ++i) {
        localVariance += std::pow(data[i] - localMean, 2);
    }
    localVariance /= count;

    // 方差大（可能是峰值区域）使用小窗口，方差小（平坦区域）使用大窗口
    double varianceThreshold = 20.0; // 根据你的数据调整
    if (localVariance > varianceThreshold) {
        return baseWindowSize; // 峰值区域，小窗口
    } else {
        return std::min(maxWindowSize, baseWindowSize + 2);
    }
}

// FFT滤波
QVector<double> MainWindow::performConservativeFFTWFiltering(const QVector<double>& data) {
    int n = data.size();
    if (n < 3) return data;

    fftw_complex *in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n);
    fftw_complex *out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n);

    for (int i = 0; i < n; ++i) {
        in[i][0] = data[i];
        in[i][1] = 0.0;
    }

    fftw_plan p = fftw_plan_dft_1d(n, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(p);

    // 增强高频滤除：使用更严格的截止频率
    int lowCutoff = n / 20;    // 保留更多低频（5%）
    int highCutoff = n / 8;    // 更早开始滤除高频（12.5%）

    for (int i = 0; i < n; ++i) {
        if (i > highCutoff && i < n - highCutoff) {
            // 高频部分：完全滤除（置零）
            out[i][0] = 0.0;
            out[i][1] = 0.0;
        } else if (i > lowCutoff && i <= highCutoff) {
            // 中高频部分：使用平滑过渡衰减
            double ratio = static_cast<double>(i - lowCutoff) / (highCutoff - lowCutoff);
            double attenuation = 1.0 - ratio; // 从1线性衰减到0
            out[i][0] *= attenuation;
            out[i][1] *= attenuation;
        }
        // 低频部分（i <= lowCutoff）：完全保留
    }

    fftw_plan p_inverse = fftw_plan_dft_1d(n, out, in, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(p_inverse);

    QVector<double> filtered(n);
    for (int i = 0; i < n; ++i) {
        filtered[i] = in[i][0] / n;

        // 确保结果合理
        if (filtered[i] < 0) {
            filtered[i] = 0;
        }
    }

    fftw_destroy_plan(p);
    fftw_destroy_plan(p_inverse);
    fftw_free(in);
    fftw_free(out);

    return filtered;
}

// 确保非零结果
QVector<double> MainWindow::ensureNonZeroResult(const QVector<double>& original, const QVector<double>& corrected)
{
    QVector<double> safeResult = corrected;
    int zeroCount = 0;

    for (int i = 0; i < original.size(); ++i) {
        if (std::abs(safeResult[i]) < 1e-10) {
            zeroCount++;
            // 如果结果为0或接近0，回退到保守处理
            safeResult[i] = original[i] * 0.5; // 至少保留50%的原始值
        }
    }
    if (zeroCount > original.size() * 0.1) { // 如果超过10%的点有问题
        qDebug() << "Warning: Many zero values detected after processing, using fallback strategy";
        // 回退到更保守的处理
        for (int i = 0; i < original.size(); ++i) {
            safeResult[i] = original[i] * 0.8; // 保留80%原始值
        }
    }

    return safeResult;
}

// 基线校正主算法
QVector<double> MainWindow::performBaselineCorrection(const QVector<double> &data)
{
   const int n = data.size();
   if (n < 3) {
       return data; // 数据太短，无法进行基线校正
   }

   // 将数据转换为Eigen格式
   Eigen::VectorXd y = Eigen::VectorXd::Map(data.constData(), n);

   // 调用ALS算法计算基线
   Eigen::VectorXd baseline = computeBaselineALS(y, m_baselineCfg->lambda, m_baselineCfg->p, m_baselineCfg->maxIter);

   // 从原始数据中减去基线的一部分
   double correctionFactor = 0.5; // 调整因子，可以根据需要修改
   QVector<double> corrected(n);
   for (int i = 0; i < n; ++i) {
       corrected[i] = data[i] - correctionFactor * baseline[i];
   }

   return corrected;
}

// ALS基线算法核心
Eigen::VectorXd MainWindow::computeBaselineALS(const Eigen::VectorXd &y, double lambda, double p, int maxIter)
{
   const int n = y.size();
   Eigen::VectorXd w = Eigen::VectorXd::Ones(n);

   // 二阶差分矩阵
   Eigen::SparseMatrix<double> D(n - 2, n);
   std::vector<Eigen::Triplet<double>> tri;
   tri.reserve(3 * (n - 2));
   for (int i = 0; i < n - 2; ++i) {
       tri.emplace_back(i, i, 1);
       tri.emplace_back(i, i + 1, -2);
       tri.emplace_back(i, i + 2, 1);
   }
   D.setFromTriplets(tri.begin(), tri.end());

   Eigen::VectorXd z;
   const double yMean = y.mean();
   for (int iter = 0; iter < maxIter; ++iter) {
       Eigen::SparseMatrix<double> W(n, n);
       std::vector<Eigen::Triplet<double>> wtri;
       wtri.reserve(n);
       for (int i = 0; i < n; ++i) wtri.emplace_back(i, i, w[i]);
       W.setFromTriplets(wtri.begin(), wtri.end());

       Eigen::SparseMatrix<double> A = W + lambda * D.transpose() * D;
       Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
       solver.compute(A);
       if (solver.info() != Eigen::Success) break;
       Eigen::VectorXd z_new = solver.solve(W * y);

       if (iter > 0 && (z_new - z).norm() < 1e-6 * z.norm()) break;
       z = z_new;

       // 权重更新
       for (int i = 0; i < n; ++i) {
           double r = y[i] - z[i];
           w[i] = r > 0 ? p : (1 - p);
       }
   }
   return z;
}

// 参数设置函数
SmoothConfig MainWindow::smoothConfig() const {
    QMutexLocker l(&m_mutex);
    return *m_smoothCfg;
}

void MainWindow::setSmoothConfig(const SmoothConfig &cfg) {
    QMutexLocker l(&m_mutex);
    *m_smoothCfg = cfg;
}

void MainWindow::setBaselineConfig(const BaselineConfig &cfg) {
    QMutexLocker l(&m_mutex);
    *m_baselineCfg = cfg;
}

BaselineConfig MainWindow::baselineConfig() const {
    QMutexLocker l(&m_mutex);
    return *m_baselineCfg;
}
QPushButton* MainWindow::createButton(const QString& text, const QString& styleSheet)//创建按钮
{
    QPushButton* button = new QPushButton(text, this);
    button->setFixedSize(80, 40); // 设置大小
    QFont buttonFont("Arial", 11, QFont::Bold); // 设置字体
    button->setFont(buttonFont);
    button->setStyleSheet(styleSheet); // 设置样式表
    return button;
}

void MainWindow::setupLayout()
{
    this->setStyleSheet("QWidget { background-color: #FFFFFF; }");
    QString buttonStyle = "QPushButton { background-color: lightblue; color: black; border: 2px solid blue; }";

    // 创建按钮
    buttonUsbCapture = createButton("采集", buttonStyle);
    buttonClearChart = createButton("清空", buttonStyle);
    buttonSaveOpen = createButton("保存/打开", buttonStyle); // 合并按钮
    buttonProcessSpectrum = createButton("处理", buttonStyle); // 合并平滑操作、去基线、寻峰
    autoFocus = createButton("自动对焦", buttonStyle);
    mappingButton = createButton("Mapping", buttonStyle);
    buttonReset = createButton("复位", buttonStyle);
    multiCaptureButton = createButton("曝光+次数", buttonStyle);
    laserButton = createButton("激光", buttonStyle);
    analysisButton = createButton("分析", buttonStyle);

    //用于显示 Z 轴位置
    zAxisLabel = new QLabel("Z轴：0", this);

    // 创建图表控件
    series = new QLineSeries();
    chart = new QChart();
    chart->addSeries(series);
    chart->setAnimationOptions(QChart::NoAnimation);
    chart->setBackgroundBrush(QBrush(QColor("white")));
    chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    // 创建坐标轴并设置样式
    xAxis = new QValueAxis;
    xAxis->setRange(300, 2000);
    xAxis->setTitleText("Raman Shift(cm-1)");
    xAxis->setLabelFormat("%i");
    QFont xAxisFont("Arial", 14, QFont::Bold);
    xAxis->setLabelsFont(xAxisFont);
    xAxis->setLinePen(QPen(Qt::black, 3));
    chart->addAxis(xAxis, Qt::AlignBottom);
    series->attachAxis(xAxis);

    yAxis = new QValueAxis;
    yAxis->setRange(300, 2000);
    yAxis->setTitleText("Intensity");
    yAxis->setLabelFormat("%i");
    QFont yAxisFont("Arial", 14, QFont::Bold);
    yAxis->setLabelsFont(yAxisFont);
    yAxis->setLinePen(QPen(Qt::black, 3));
    chart->addAxis(yAxis, Qt::AlignLeft);

    xAxis->setTickCount(5);
    yAxis->setTickCount(5);

    QPen pen(Qt::red, 1);
    series->setPen(pen);
    series->attachAxis(yAxis);

    // 移除默认的坐标轴网格线
    xAxis->setGridLineVisible(false);
    yAxis->setGridLineVisible(false);

    // 创建上轴和右轴，仅显示轴线
    topAxis = new QValueAxis;
    topAxis->setRange(200, 2000);
    topAxis->setLabelsVisible(false);
    topAxis->setLinePen(QPen(Qt::black, 3));
    topAxis->setTickCount(0);
    topAxis->setGridLineVisible(false);
    chart->addAxis(topAxis, Qt::AlignTop);

    rightAxis = new QValueAxis;
    rightAxis->setRange(0, 2000);
    rightAxis->setLabelsVisible(false);
    rightAxis->setLinePen(QPen(Qt::black, 3));
    rightAxis->setTickCount(0);
    rightAxis->setGridLineVisible(false);
    chart->addAxis(rightAxis, Qt::AlignRight);

    // 隐藏图例
    chart->legend()->hide();
    chartView->setFixedSize(770, 470);

    // 将按钮和图表视图添加到主窗口中
    buttonUsbCapture->setParent(ui->centralwidget);
    buttonClearChart->setParent(ui->centralwidget);
    buttonProcessSpectrum->setParent(ui->centralwidget);
    autoFocus->setParent(ui->centralwidget);
    mappingButton->setParent(ui->centralwidget);
    chartView->setParent(ui->centralwidget);
    buttonReset->setParent(ui->centralwidget);
    analysisButton->setParent(ui->centralwidget);

    // 设置按钮和图表视图的位置
    zAxisLabel->move(200, 10);
    laserButton->move(102, 10);

    buttonUsbCapture->move(0, 10);
    multiCaptureButton->move(0, 60);
    analysisButton->move(0, 110);
    buttonSaveOpen->move(0, 160);
    buttonProcessSpectrum->move(0, 210);
    buttonClearChart->move(0, 260);
    autoFocus->move(0, 310);
    mappingButton->move(0, 360);
    buttonReset->move(0, 410);


    chartView->move(80, 10);


    connect(laserButton, &QPushButton::clicked, this, &MainWindow::on_laser_clicked);
    connect(buttonClearChart, &QPushButton::clicked, this, &MainWindow::on_buttonClearChart_clicked);
    connect(buttonProcessSpectrum, &QPushButton::clicked, this, &MainWindow::on_buttonProcessSpectrum_clicked);
    connect(autoFocus, &QPushButton::clicked, this, &MainWindow::on_autoFocus_clicked);
    connect(mappingButton, &QPushButton::clicked, this, &MainWindow::on_mapping_clicked);
    connect(buttonReset, &QPushButton::clicked, this, &MainWindow::on_buttonReset_clicked);
    connect(multiCaptureButton, &QPushButton::clicked, this, &MainWindow::on_multiCapture_clicked);
    connect(buttonSaveOpen, &QPushButton::clicked, this, &MainWindow::on_saveOpen_clicked);
    connect(analysisButton, &QPushButton::clicked, this, &MainWindow::on_analysis_clicked);
}

void MainWindow::handleUsbError(const QString& message)//USB错误警报
{
    QMessageBox::critical(this, "[Error]", message);
}

void MainWindow::handleDataProcessed(const QVector<double>& data,const QVector<double>& xData)
{
    if (m_silentMode)
        return;

    if (data.isEmpty() || xData.isEmpty()) return;

    // 确保主光谱可见
    if (series) {
        series->setVisible(true);
    }

    QVector<QPointF> points;
    points.reserve(data.size());

    double miny =  std::numeric_limits<double>::max();
    double maxy = -std::numeric_limits<double>::max();
    double minx =  std::numeric_limits<double>::max();
    double maxx = -std::numeric_limits<double>::max();

    for (int i = 0; i < data.size(); ++i) {
        double x = xData[i];
        double y = data[i];
        points.append(QPointF(x, y));
        minx = std::min(minx, x);
        maxx = std::max(maxx, x);
        miny = std::min(miny, y);
        maxy = std::max(maxy, y);
    }

    series->replace(points);
    xAxis->setRange(minx, maxx);
    yAxis->setRange(0, maxy * 1.1); // 最大值比光谱最大值高 10%

    chartView->update();
}

void MainWindow::handleDataReceived(const QVector<double>& data)
{

    if (data.size() < 70) return; // 如果数据量不足 70，直接返回

    // 剔除 y 轴数据的前 61 个和最后 9 个值
    QVector<double> filteredY = data.mid(61, data.size() - 70);

    // 定义全局常量
    QVector<double> xData = {
        187.703, 191.654, 195.602, 199.548, 203.493, 207.435, 211.376, 215.314, 219.251, 223.186,
        227.119, 231.05, 234.979, 238.906, 242.831, 246.754, 250.676, 254.595, 258.513, 262.428,
        266.342, 270.254, 274.164, 278.071, 281.977, 285.882, 289.784, 293.684, 297.582, 301.479,
        305.373, 309.266, 313.157, 317.046, 320.933, 324.818, 328.701, 332.582, 336.462, 340.339,
        344.215, 348.088, 351.96, 355.83, 359.698, 363.564, 367.428, 371.291, 375.151, 379.01,
        382.867, 386.721, 390.574, 394.425, 398.275, 402.122, 405.967, 409.811, 413.652, 417.492,
        421.33, 425.166, 429, 432.833, 436.663, 440.492, 444.318, 448.143, 451.966, 455.787, 459.606,
        463.424, 467.239, 471.053, 474.865, 478.675, 482.483, 486.289, 490.093, 493.896, 497.697,
        501.495, 505.292, 509.087, 512.881, 516.672, 520.462, 524.249, 528.035, 531.819, 535.601,
        539.382, 543.16, 546.937, 550.712, 554.485, 558.256, 562.025, 565.793, 569.558, 573.322,
        577.084, 580.844, 584.603, 588.359, 592.114, 595.867, 599.618, 603.367, 607.114, 610.86,
        614.604, 618.345, 622.086, 625.824, 629.56, 633.295, 637.028, 640.759, 644.488, 648.215,
        651.941, 655.665, 659.387, 663.107, 666.825, 670.542, 674.256, 677.969, 681.68, 685.39,
        689.097, 692.803, 696.507, 700.209, 703.909, 707.608, 711.304, 714.999, 718.693, 722.384,
        726.073, 729.761, 733.447, 737.131, 740.814, 744.494, 748.173, 751.85, 755.525, 759.199,
        762.871, 766.54, 770.209, 773.875, 777.54, 781.202, 784.863, 788.523, 792.18, 795.836,
        799.49, 803.142, 806.792, 810.441, 814.088, 817.733, 821.376, 825.018, 828.657, 832.295,
        835.932, 839.566, 843.199, 846.83, 850.459, 854.086, 857.712, 861.336, 864.958, 868.579,
        872.197, 875.814, 879.43, 883.043, 886.655, 890.265, 893.873, 897.479, 901.084, 904.687,
        908.288, 911.887, 915.485, 919.081, 922.675, 926.268, 929.859, 933.448, 937.035, 940.621,
        944.204, 947.786, 951.367, 954.945, 958.522, 962.098, 965.671, 969.243, 972.813, 976.381,
        979.947, 983.512, 987.075, 990.637, 994.196, 997.754, 1001.31, 1004.86, 1008.42, 1011.97,
        1015.52, 1019.07, 1022.61, 1026.16, 1029.7, 1033.24, 1036.78, 1040.32, 1043.85, 1047.39,
        1050.92, 1054.45, 1057.98, 1061.5, 1065.03, 1068.55, 1072.08, 1075.6, 1079.11, 1082.63,
        1086.15, 1089.66, 1093.17, 1096.68, 1100.19, 1103.7, 1107.2, 1110.71, 1114.21, 1117.71,
        1121.21, 1124.7, 1128.2, 1131.69, 1135.18, 1138.67, 1142.16, 1145.65, 1149.13, 1152.61,
        1156.1, 1159.58, 1163.05, 1166.53, 1170.01, 1173.48, 1176.95, 1180.42, 1183.89, 1187.35,
        1190.82, 1194.28, 1197.74, 1201.2, 1204.66, 1208.12, 1211.57, 1215.03, 1218.48, 1221.93,
        1225.37, 1228.82, 1232.27, 1235.71, 1239.15, 1242.59, 1246.03, 1249.46, 1252.9, 1256.33,
        1259.76, 1263.19, 1266.62, 1270.05, 1273.47, 1276.9, 1280.32, 1283.74, 1287.16, 1290.57,
        1293.99, 1297.4, 1300.81, 1304.22, 1307.63, 1311.04, 1314.45, 1317.85, 1321.25, 1324.65,
        1328.05, 1331.45, 1334.84, 1338.24, 1341.63, 1345.02, 1348.41, 1351.8, 1355.18, 1358.56,
        1361.95, 1365.33, 1368.71, 1372.08, 1375.46, 1378.84, 1382.21, 1385.58, 1388.95, 1392.32,
        1395.68, 1399.05, 1402.41, 1405.77, 1409.13, 1412.49, 1415.85, 1419.2, 1422.55, 1425.91,
        1429.26, 1432.6, 1435.95, 1439.3, 1442.64, 1445.98, 1449.32, 1452.66, 1456, 1459.33,
        1462.67, 1466, 1469.33, 1472.66, 1475.99, 1479.31, 1482.64, 1485.96, 1489.28, 1492.6,
        1495.92, 1499.24, 1502.55, 1505.87, 1509.18, 1512.49, 1515.8, 1519.1, 1522.41, 1525.71,
        1529.01, 1532.32, 1535.61, 1538.91, 1542.21, 1545.5, 1548.79, 1552.09, 1555.38, 1558.66,
        1561.95, 1565.23, 1568.52, 1571.8, 1575.08, 1578.36, 1581.64, 1584.91, 1588.18, 1591.46,
        1594.73, 1598, 1601.26, 1604.53, 1607.8, 1611.06, 1614.32, 1617.58, 1620.84, 1624.09,
        1627.35, 1630.6, 1633.86, 1637.11, 1640.35, 1643.6, 1646.85, 1650.09, 1653.33, 1656.58,
        1659.82, 1663.05, 1666.29, 1669.53, 1672.76, 1675.99, 1679.22, 1682.45, 1685.68, 1688.9,
        1692.13, 1695.35, 1698.57, 1701.79, 1705.01, 1708.22, 1711.44, 1714.65, 1717.86, 1721.07,
        1724.28, 1727.49, 1730.7, 1733.9, 1737.1, 1740.3, 1743.5, 1746.7, 1749.9, 1753.09,
        1756.29, 1759.48, 1762.67, 1765.86, 1769.05, 1772.23, 1775.42, 1778.6, 1781.78, 1784.96,
        1788.14, 1791.32, 1794.49, 1797.66, 1800.84, 1804.01, 1807.18, 1810.34, 1813.51, 1816.68,
        1819.84, 1823, 1826.16, 1829.32, 1832.48, 1835.63, 1838.79, 1841.94, 1845.09, 1848.24,
        1851.39, 1854.53, 1857.68, 1860.82, 1863.96, 1867.11, 1870.24, 1873.38, 1876.52, 1879.65,
        1882.79, 1885.92, 1889.05, 1892.18, 1895.3, 1898.43, 1901.55, 1904.68, 1907.8, 1910.92,
        1914.04, 1917.15, 1920.27, 1923.38, 1926.49, 1929.6, 1932.71, 1935.82, 1938.93, 1942.03,
        1945.14, 1948.24, 1951.34, 1954.44, 1957.54, 1960.63, 1963.73, 1966.82, 1969.91, 1973,
        1976.09, 1979.18, 1982.26, 1985.35, 1988.43, 1991.51, 1994.59, 1997.67, 2000.75, 2003.82,
        2006.9, 2009.97, 2013.04, 2016.11, 2019.18, 2022.24, 2025.31, 2028.37, 2031.44, 2034.5,
        2037.56, 2040.61, 2043.67, 2046.73, 2049.78, 2052.83, 2055.88, 2058.93, 2061.98, 2065.03,
        2068.07, 2071.11, 2074.16, 2077.2, 2080.24, 2083.27, 2086.31, 2089.34, 2092.38, 2095.41,
        2098.44, 2101.47, 2104.5, 2107.52, 2110.55, 2113.57, 2116.59, 2119.61, 2122.63, 2125.65,
        2128.67, 2131.68, 2134.69, 2137.71, 2140.72, 2143.73, 2146.73, 2149.74, 2152.74, 2155.75,
        2158.75, 2161.75, 2164.75, 2167.75, 2170.74, 2173.74, 2176.73, 2179.72, 2182.71, 2185.7,
        2188.69, 2191.68, 2194.66, 2197.64, 2200.63, 2203.61, 2206.59, 2209.56, 2212.54, 2215.51,
        2218.49, 2221.46, 2224.43, 2227.4, 2230.37, 2233.33, 2236.3, 2239.26, 2242.23, 2245.19,
        2248.15, 2251.1, 2254.06, 2257.02, 2259.97, 2262.92, 2265.87, 2268.82, 2271.77, 2274.72,
        2277.66, 2280.61, 2283.55, 2286.49, 2289.43, 2292.37, 2295.31, 2298.24, 2301.18, 2304.11,
        2307.04, 2309.97, 2312.9, 2315.83, 2318.75, 2321.68, 2324.6, 2327.52, 2330.44, 2333.36,
        2336.28, 2339.2, 2342.11, 2345.03, 2347.94, 2350.85, 2353.76, 2356.67, 2359.57, 2362.48,
        2365.38, 2368.28, 2371.19, 2374.08, 2376.98, 2379.88, 2382.78, 2385.67, 2388.56, 2391.45,
        2394.34, 2397.23, 2400.12, 2403.01, 2405.89, 2408.78, 2411.66, 2414.54, 2417.42, 2420.3,
        2423.17, 2426.05, 2428.92, 2431.79, 2434.66, 2437.53, 2440.4, 2443.27, 2446.14, 2449,
        2451.86, 2454.72, 2457.58, 2460.44, 2463.3, 2466.16, 2469.01, 2471.87, 2474.72, 2477.57,
        2480.42, 2483.27, 2486.11, 2488.96, 2491.8, 2494.65, 2497.49, 2500.33, 2503.17, 2506,
        2508.84, 2511.67, 2514.51, 2517.34, 2520.17, 2523, 2525.83, 2528.65, 2531.48, 2534.3,
        2537.13, 2539.95, 2542.77, 2545.59, 2548.4, 2551.22, 2554.03, 2556.85, 2559.66, 2562.47,
        2565.28, 2568.09, 2570.89, 2573.7, 2576.5, 2579.31, 2582.11, 2584.91, 2587.71, 2590.5,
        2593.3, 2596.1, 2598.89, 2601.68, 2604.47, 2607.26, 2610.05, 2612.84, 2615.62, 2618.41,
        2621.19, 2623.97, 2626.75, 2629.53, 2632.31, 2635.09, 2637.86, 2640.63, 2643.41, 2646.18,
        2648.95, 2651.72, 2654.48, 2657.25, 2660.01, 2662.78, 2665.54, 2668.3, 2671.06, 2673.82,
        2676.58, 2679.33, 2682.09, 2684.84, 2687.59, 2690.34, 2693.09, 2695.84, 2698.58, 2701.33,
        2704.07, 2706.82, 2709.56, 2712.3, 2715.04, 2717.77, 2720.51, 2723.24, 2725.98, 2728.71,
        2731.44, 2734.17, 2736.9, 2739.63, 2742.35, 2745.08, 2747.8, 2750.52, 2753.24, 2755.96,
        2758.68, 2761.4, 2764.11, 2766.83, 2769.54, 2772.25, 2774.96, 2777.67, 2780.38, 2783.09,
        2785.79, 2788.5, 2791.2, 2793.9, 2796.6, 2799.3, 2802, 2804.7, 2807.39, 2810.08, 2812.78,
        2815.47, 2818.16, 2820.85, 2823.54, 2826.22, 2828.91, 2831.59, 2834.27, 2836.95, 2839.63,
        2842.31, 2844.99, 2847.67, 2850.34, 2853.02, 2855.69, 2858.36, 2861.03, 2863.7, 2866.37,
        2869.03, 2871.7, 2874.36, 2877.02, 2879.69, 2882.35, 2885, 2887.66, 2890.32, 2892.97,
        2895.63, 2898.28, 2900.93, 2903.58, 2906.23, 2908.88, 2911.52, 2914.17, 2916.81, 2919.45,
        2922.1, 2924.74, 2927.37, 2930.01, 2932.65, 2935.28, 2937.92, 2940.55, 2943.18, 2945.81,
        2948.44, 2951.07, 2953.69, 2956.32, 2958.94, 2961.57, 2964.19, 2966.81, 2969.43, 2972.05,
        2974.66, 2977.28, 2979.89, 2982.5, 2985.12, 2987.73, 2990.34, 2992.94, 2995.55, 2998.16,
        3000.76, 3003.36, 3005.97, 3008.57, 3011.17, 3013.76, 3016.36, 3018.96, 3021.55, 3024.15,
        3026.74, 3029.33, 3031.92, 3034.51, 3037.09, 3039.68, 3042.26, 3044.85, 3047.43, 3050.01,
        3052.59, 3055.17, 3057.75, 3060.32, 3062.9, 3065.47, 3068.05, 3070.62, 3073.19, 3075.76,
        3078.33, 3080.89, 3083.46, 3086.02, 3088.59, 3091.15, 3093.71, 3096.27, 3098.83, 3101.38,
        3103.94, 3106.49, 3109.05, 3111.6, 3114.15, 3116.7, 3119.25, 3121.8, 3124.34, 3126.89,
        3129.43, 3131.98, 3134.52, 3137.06, 3139.6, 3142.14, 3144.67, 3147.21, 3149.74, 3152.28,
        3154.81, 3157.34, 3159.87, 3162.4, 3164.92, 3167.45, 3169.98, 3172.5, 3175.02, 3177.54,
        3180.06, 3182.58, 3185.1, 3187.62, 3190.13, 3192.65, 3195.16, 3197.67, 3200.18, 3202.69,
        3205.2, 3207.71, 3210.21, 3212.72, 3215.22, 3217.73, 3220.23, 3222.73, 3225.23, 3227.72,
        3230.22, 3232.72, 3235.21, 3237.7, 3240.2, 3242.69, 3245.18, 3247.67, 3250.15, 3252.64,
        3255.12, 3257.61, 3260.09
    };

    // 剔除 x 轴数据的前 61 个和最后 9 个值
    QVector<double> filteredX = xData;

    // 如果没有有效数据，直接返回
    if (filteredX.isEmpty() || filteredY.isEmpty()) {
        return;
    }

    // 保存全量 x 和 y 数据到文件
    saveRawDataToFile(filteredX, filteredY);

    // 截取 x 在 (345, 2002) 范围内的数据
    QVector<double> finalX;
    QVector<double> finalY;
    for (int i = 0; i < filteredX.size(); ++i) {
        if (filteredX[i] > 300 && filteredX[i] < 2002) {
            finalX.append(filteredX[i]);
            finalY.append(filteredY[i]);
        }
    }

    // 如果没有有效数据，直接返回
    if (finalX.isEmpty() || finalY.isEmpty()) {
        return;
    }

    // 清空图表数据
    series->clear();
    chearflag = true;

    // 确保主光谱可见（在数据接收时恢复显示）
    if (series) {
        series->setVisible(true);
    }

    // 将截取的数据绘制到图表中
    QVector<QPointF> points;
    double miny = std::numeric_limits<double>::max();
    double maxy = std::numeric_limits<double>::lowest();
    double maxx = 0.0;

    for (int i = 0; i < finalX.size(); ++i) {
        double x = finalX[i];
        double y = finalY[i];

        bool isTargetPoint = std::abs(x - 835.932) < 0.1;
        if (isTargetPoint && i > 0) {
            y = finalY[i-1];  // 使用前一个点的y值
        }
        points.append(QPointF(x, y));
        miny = std::min(miny, y);
        maxy = std::max(maxy, y);
        maxx = std::max(maxx, x);
    }

    series->replace(points);

    // 设置显示范围
    xAxis->setRange(300, 2002);
    yAxis->setRange(0, maxy * 1.1);

    chartView->update();

    // 信号发射
    if (receivers(SIGNAL(afFirstFrameArrived())) > 0) {
        emit afFirstFrameArrived();
    }
    if (m_autoFocusRunning) {
        QTimer::singleShot(100, this, [this]() {
            emit afDataReady();
        });
    }
    emit mappingDataReady();
}

void MainWindow::setupSerialPort() {
    serialPort = new QSerialPort(this);

    // 枚举所有可用的串口
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()) {
        // 检查串口名称是否包含“3”
        if (serialPortInfo.portName().contains("3")) {
            // 设置串口名称
            serialPort->setPortName(serialPortInfo.portName());
            break;  // 找到第一个符合条件的串口后停止搜索
        }
    }

    // 如果没有找到符合条件的串口，弹出错误提示
    if (serialPort->portName().isEmpty()) {
        QMessageBox::critical(this, "串口错误", "未找到串口");
        return;
    }

    // 设置串口参数
    serialPort->setBaudRate(QSerialPort::Baud9600);  // 设置波特率
    serialPort->setDataBits(QSerialPort::Data8);     // 设置数据位
    serialPort->setParity(QSerialPort::NoParity);    // 设置校验位
    serialPort->setStopBits(QSerialPort::OneStop);   // 设置停止位
    serialPort->setFlowControl(QSerialPort::NoFlowControl);  // 设置流控制

    // 打开串口
    if (!serialPort->open(QIODevice::ReadWrite)) {
        QMessageBox::critical(this, "串口错误", "无法打开串口");
    }
}

void MainWindow::on_analysis_clicked()
{
    // 确保主光谱可见
    if (series) {
        series->setVisible(true);
    }

    // 获取当前光谱数据
    if (series->count() == 0) {
        QMessageBox::warning(this, "警告", "没有数据可供分析。请加载数据。");
        return;
    }

    // 提取原始光谱数据
    QVector<double> xData, yData;
    for (int i = 0; i < series->count(); ++i) {
        xData.append(series->at(i).x());
        yData.append(series->at(i).y());
    }

    // 调用数据处理函数：平滑处理
    QVector<double> smoothedY = yData;
    emit requestSmooth(smoothedY, xData);

    // 调用数据处理函数：去基线处理
    QVector<double> baselineCorrectedY = smoothedY;
    emit requestBaseline(baselineCorrectedY, xData);

    // 调用数据处理函数：寻峰处理
    QList<int> peakIndices;
    gaussLorentzDetectPeaks(baselineCorrectedY, peakIndices, 1.0, 1.0, 20.0);

    // 构建峰值列表
    QVector<QPair<double, double>> detectedPeaks;
    for (int idx : peakIndices) {
        if (idx >= 0 && idx < xData.size()) {
            detectedPeaks.append(qMakePair(xData[idx], baselineCorrectedY[idx]));
        }
    }

    // 定义物质库（包含硅片参考峰）
    struct Material {
        QString name;
        QVector<QPair<double, double>> peakRanges;  // 特征峰值区间(cm-1)
        double calibrationFactor;  // 校准因子 (单位: 浓度单位/强度单位)
        QString unit;  // 浓度单位
        double minSimilarity;  // 最小相似度阈值(0-1)
        bool isSiliconReference;  // 是否为硅片参考峰
    };

    // 物质库数据 - 硅片参与匹配但特殊处理
    QVector<Material> materialLibrary = {
        {"4MBA", {{1565, 1595}, {1065, 1095}, {835, 865}}, 1.5e-4, "μg/cm²", 0.7, false},
        {"R6G", {{1350, 1378}, {1495, 1521}, {605, 635}}, 2.3e-3, "μM", 0.65, false},
        {"AOF", {{1435, 1465}, {1635, 1665}, {1275, 1305}}, 8.7e-6, "ng/mL", 0.6, false},
        {"MB", {{445, 475}, {1385, 1415}, {1615, 1645}}, 5.2e-4, "μg/mL", 0.55, false},
        {"纳米粒子", {{1370, 1380}, {1600, 1605}, {1770, 1780}}, 5.2e-4, "μg/mL", 0.55, false},
        {"Silicon", {{510, 530}}, 0.0, "", 0.5, true}  // 硅片参考峰，参与匹配但设置较低阈值
    };

    // 分析当前光谱
    QString matchedMaterial = "未知物质";
    double similarity = 0.0;
    double concentration = 0.0;
    QString concentrationUnit = "";
    QVector<QPair<double, double>> matchedPeaks;
    bool isSiliconDetected = false;

    // 匹配物质库（硅片也参与匹配）
    for (const auto& material : materialLibrary) {
        double currentSimilarity = 0.0;
        int matchedPeakCount = 0;
        QVector<QPair<double, double>> currentMatchedPeaks;

        // 检查每个特征峰范围
        for (const auto& peakRange : material.peakRanges) {
            bool foundMatch = false;
            // 在检测到的峰中查找匹配峰
            for (const auto& peak : detectedPeaks) {
                // 检查峰位置是否在特征峰范围内
                if (peak.first >= peakRange.first && peak.first <= peakRange.second) {
                    matchedPeakCount++;
                    currentMatchedPeaks.append(peak);
                    foundMatch = true;
                    break;
                }
            }
        }

        // 计算匹配相似度（基于匹配峰的数量和位置）
        if (material.peakRanges.size() > 0) {
            currentSimilarity = static_cast<double>(matchedPeakCount) / material.peakRanges.size();

            // 检查是否达到最小相似度阈值
            if (currentSimilarity >= material.minSimilarity && currentSimilarity > similarity) {
                similarity = currentSimilarity;
                matchedMaterial = material.name;
                concentrationUnit = material.unit;
                matchedPeaks = currentMatchedPeaks;
                isSiliconDetected = material.isSiliconReference;
            }
        }
    }

    // 计算特征峰总面积（用于浓度计算）
    double characteristicPeakArea = 0.0;
    if (!matchedPeaks.isEmpty() && !isSiliconDetected) {
        // 对每个匹配的峰，计算其附近区域的面积（硅片不计算浓度）
        for (const auto& peak : matchedPeaks) {
            // 在xData中找到最接近peak.first的索引
            int closestIdx = -1;
            double minDiff = std::numeric_limits<double>::max();
            for (int i = 0; i < xData.size(); ++i) {
                double diff = std::abs(xData[i] - peak.first);
                if (diff < minDiff) {
                    minDiff = diff;
                    closestIdx = i;
                }
            }

            // 计算以特征峰为中心的±10 cm-1区域面积
            if (closestIdx >= 0) {
                int startIdx = closestIdx;
                int endIdx = closestIdx;

                // 向前搜索
                while (startIdx > 0 && (peak.first - xData[startIdx]) <= 10.0) {
                    startIdx--;
                }

                // 向后搜索
                while (endIdx < xData.size()-1 && (xData[endIdx] - peak.first) <= 10.0) {
                    endIdx++;
                }

                // 计算该区域的积分
                for (int i = startIdx + 1; i <= endIdx; ++i) {
                    if (i < xData.size()) {
                        double dx = xData[i] - xData[i-1];
                        double avgY = (baselineCorrectedY[i] + baselineCorrectedY[i-1]) / 2.0;
                        characteristicPeakArea += dx * avgY;
                    }
                }
            }
        }
    }

    // 计算浓度（仅当匹配到已知物质且不是硅片）
    if (matchedMaterial != "未知物质" && !isSiliconDetected) {
        for (const auto& material : materialLibrary) {
            if (material.name == matchedMaterial) {
                // 使用特征峰总面积和校准因子计算浓度
                concentration = characteristicPeakArea * material.calibrationFactor;

                // 根据浓度范围调整显示精度
                if (concentration < 0.001) {
                    concentration = std::round(concentration * 1e6) / 1e6; // 保留6位小数
                } else if (concentration < 0.01) {
                    concentration = std::round(concentration * 1e4) / 1e4; // 保留4位小数
                } else if (concentration < 1.0) {
                    concentration = std::round(concentration * 1e3) / 1e3; // 保留3位小数
                } else {
                    concentration = std::round(concentration * 1e2) / 1e2; // 保留2位小数
                }
                break;
            }
        }
    }

    // 构建结果信息
    QString resultText;
    if (matchedMaterial == "未知物质") {
        resultText = "<b>分析结果:</b> 未匹配到已知物质<br>";

        // 显示检测到的峰
        if (!detectedPeaks.isEmpty()) {
            resultText += "<b>检测到的峰位置(cm-1):</b><br>";
            for (const auto& peak : detectedPeaks) {
                resultText += QString("• %1 cm-1 (强度: %2)<br>")
                        .arg(peak.first, 0, 'f', 1)
                        .arg(peak.second, 0, 'f', 2);
            }
        }
    } else {
        // 硅片检测的特殊显示
        if (isSiliconDetected) {
            resultText = QString("<b>检测到硅片参考峰</b><br>");
            resultText += "<b>硅片特征峰位置(cm-1):</b><br>";
        } else {
            resultText = QString("<b>匹配物质:</b> %1<br>").arg(matchedMaterial);
            resultText += "<b>匹配峰位置(cm-1):</b><br>";
        }

        // 添加匹配峰信息
        for (const auto& peak : matchedPeaks) {
            resultText += QString("• %1 cm-1 (强度: %2)<br>")
                    .arg(peak.first, 0, 'f', 1)
                    .arg(peak.second, 0, 'f', 2);
        }

        // === 显示百分比相似度 ===
        double similarityPercent = similarity * 100.0;

        // 格式化显示：四舍五入到1位小数，如果是100%则显示100%
        QString similarityStr;
        if (similarityPercent >= 99.95) {
            similarityStr = "100%";
        } else {
            similarityStr = QString("%1%").arg(similarityPercent, 0, 'f', 1);
        }

        // 根据相似度选择颜色
        QString color;
        if (similarityPercent >= 90) {
            color = "green";
        } else if (similarityPercent >= 80) {
            color = "blue";
        } else if (similarityPercent >= 70) {
            color = "orange";
        } else {
            color = "red";
        }

        // 硅片和普通物质的显示区别
        if (isSiliconDetected) {
            resultText += QString("<b>参考峰匹配度:</b> <font color='%1'>%2</font><br>")
                    .arg(color)
                    .arg(similarityStr);
            resultText += "<i>（硅片参考峰用于仪器校准，不计算浓度）</i><br>";
        } else {
            resultText += QString("<b>匹配相似度:</b> <font color='%1'>%2</font><br>")
                    .arg(color)
                    .arg(similarityStr);

            // 显示浓度（非硅片物质）
            if (!concentrationUnit.isEmpty() && concentration > 0) {
                resultText += QString("<b>浓度:</b> %1 %2<br>")
                        .arg(concentration, 0, 'f', 4)
                        .arg(concentrationUnit);
            }
        }
        // ===================================
    }

    // 显示结果对话框
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("光谱分析结果");
    msgBox.setIcon(QMessageBox::Information);

    QLabel *resultLabel = new QLabel(resultText);
    resultLabel->setTextFormat(Qt::RichText);
    msgBox.layout()->addWidget(resultLabel);

    msgBox.exec();

}

void MainWindow::on_laser_clicked() {
    QDialog dialog(this);
    dialog.setFixedSize(300, 350); // 调整对话框大小

    // 设置对话框的样式表，添加黑色边框
    dialog.setStyleSheet("QDialog { border: 2px solid black; background-color: white; }");

    QVBoxLayout layout(&dialog);

    // 标题
    QLabel *titleLabel = new QLabel("激光控制", &dialog);
    titleLabel->setAlignment(Qt::AlignCenter);
    QFont titleFont("Arial", 16, QFont::Bold);
    titleLabel->setFont(titleFont);

    // 滑块和数字显示
    QSlider* slider = new QSlider(Qt::Horizontal, &dialog);
    slider->setRange(20, 400);  // 电流范围 20~400
    slider->setValue(100);      // 默认值 100mA

    QLabel* currentLabel = new QLabel("电流: 100 mA", &dialog);
    currentLabel->setAlignment(Qt::AlignCenter);
    QFont labelFont("Arial", 14, QFont::Bold);
    currentLabel->setFont(labelFont);

    // 更新电流显示
    connect(slider, &QSlider::valueChanged, [&slider, currentLabel](int value) {
        currentLabel->setText(QString("电流: %1 mA").arg(value));
    });

    layout.addWidget(titleLabel);
    layout.addSpacing(20);
    layout.addWidget(slider);
    layout.addWidget(currentLabel);

    // 按钮样式
    QString buttonStyle =
            "QPushButton { "
        "   background-color: lightblue; "
        "   color: black; "
        "   border: 2px solid blue; "
        "   font-size: 14px; "
        "   font-weight: bold; "
        "   min-height: 50px; "
        "   min-width: 150px; "
        "}"
        "QPushButton:hover { "
        "   background-color: #87CEFA; "
        "}"
        "QPushButton:pressed { "
        "   background-color: #1E90FF; "
        "}";

    // 按钮
    QPushButton* laserOnButton = new QPushButton("打开激光", &dialog);
    laserOnButton->setStyleSheet(buttonStyle);
    connect(laserOnButton, &QPushButton::clicked, [&]() {
        int current = slider->value();
        sendLaserCommand(true, current);
    });

    QPushButton* laserOffButton = new QPushButton("关闭激光", &dialog);
    laserOffButton->setStyleSheet(buttonStyle);
    connect(laserOffButton, &QPushButton::clicked, [&]() {
        sendLaserCommand(false, 0);
    });

    QPushButton* cancelButton = new QPushButton("取消", &dialog);
    cancelButton->setStyleSheet(buttonStyle);
    connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::close);

    layout.addSpacing(20);
    layout.addWidget(laserOnButton);
    layout.addSpacing(20);
    layout.addWidget(laserOffButton);
    layout.addSpacing(20);
    layout.addWidget(cancelButton);

    dialog.exec();
}

void MainWindow::sendLaserCommand(bool isOn, int current) {
    if (isOn) {
        // 发送激光打开指令
        QByteArray command = generateLaserCommand(current);
        serialPort->write(command);
    } else {
        // 发送激光关闭指令
        QByteArray command = "\x55\xAA\x03\x00\x03";
        serialPort->write(command);
    }
}

QByteArray MainWindow::generateLaserCommand(int current) {
    QByteArray command(7, 0);
    command[0] = 0x55;
    command[1] = 0xAA;
    command[2] = 0x05;
    command[3] = 0x04;

    // 将电流值转换为16位（高位和低位）
    command[4] = (current >> 8) & 0xFF;  // 高位
    command[5] = current & 0xFF;        // 低位

    // 计算校验和
    command[6] = (command[2] + command[3] + command[4] + command[5]) & 0xFF;

    return command;
}

void MainWindow::saveRawDataToFile(const QVector<double>& xData, const QVector<double>& yData)
{
    QString defaultPath = "/home/Datas/"; // 默认保存路径
    QString currentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
    QString fileName = currentTime + "_raw_data.txt";
    QString filePath = defaultPath + fileName;

    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly)) {
        QTextStream out(&file);
        for (int i = 0; i < xData.size(); ++i) {
            out << xData[i] << "\t" << yData[i] << "\n"; // 每行保存 x 和 y
        }
        file.close();
    }
}

void MainWindow::on_saveOpen_clicked()
{
    QDialog dialog(this);
    dialog.setFixedSize(200, 350); // 调整对话框大小
    dialog.setStyleSheet("QDialog { border: 2px solid black; background-color: white; }");

    QVBoxLayout layout(&dialog);

    // 标题
    QLabel *titleLabel = new QLabel("文件操作", &dialog);
    titleLabel->setAlignment(Qt::AlignCenter);
    QFont titleFont("Arial", 16, QFont::Bold);
    titleLabel->setFont(titleFont);

    // 大按钮样式
    QString bigButtonStyle =
            "QPushButton { "
        "   background-color: lightblue; "
        "   color: black; "
        "   border: 2px solid blue; "
        "   font-size: 14px; "
        "   font-weight: bold; "
        "   min-height: 50px; "
        "   min-width: 150px; "
        "}"
        "QPushButton:hover { "
        "   background-color: #87CEFA; "
        "}"
        "QPushButton:pressed { "
        "   background-color: #1E90FF; "
        "}";

    // 大按钮
    QPushButton* saveDrawButton = new QPushButton("保存绘图", &dialog);
    saveDrawButton->setStyleSheet(bigButtonStyle);
    connect(saveDrawButton, &QPushButton::clicked, this, &MainWindow::on_savedraw_clicked);

    QPushButton* saveDataButton = new QPushButton("保存数据", &dialog);
    saveDataButton->setStyleSheet(bigButtonStyle);
    connect(saveDataButton, &QPushButton::clicked, this, &MainWindow::on_savedata_clicked);

    QPushButton* openFileButton = new QPushButton("打开文件", &dialog);
    openFileButton->setStyleSheet(bigButtonStyle);
    connect(openFileButton, &QPushButton::clicked, this, &MainWindow::on_openfile_clicked);

    QPushButton* cancelButton = new QPushButton("取消", &dialog);
    cancelButton->setStyleSheet(bigButtonStyle);
    connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::close);

    // 添加到布局
    layout.addWidget(titleLabel);
    layout.addSpacing(20);
    layout.addWidget(saveDrawButton);
    layout.addSpacing(20);
    layout.addWidget(saveDataButton);
    layout.addSpacing(20);
    layout.addWidget(openFileButton);
    layout.addSpacing(20);
    layout.addWidget(cancelButton);

    dialog.exec();
}

void MainWindow::on_buttonSave_clicked() // 保存按钮的槽函数
{
    QDialog dialog(this);
    dialog.setWindowTitle("保存选项");
    QVBoxLayout layout(&dialog);

    QPushButton* saveDrawButton = new QPushButton("保存绘图", &dialog);
    connect(saveDrawButton, &QPushButton::clicked, this, &MainWindow::on_savedraw_clicked);
    layout.addWidget(saveDrawButton);

    QPushButton* saveDataButton = new QPushButton("保存数据", &dialog);
    connect(saveDataButton, &QPushButton::clicked, this, &MainWindow::on_savedata_clicked);
    layout.addWidget(saveDataButton);

    QPushButton* cancelButton = new QPushButton("取消", &dialog);
    connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::close);
    layout.addWidget(cancelButton);

    dialog.setLayout(&layout);
    dialog.exec();
}

void MainWindow::on_buttonClearChart_clicked()
{
    // 1. 清空图表系列（主要光谱显示）
    if (series) {
        series->clear();
    }

    // 2. 清空峰值标注（图表上的标注）
    if (peakSeries) {
        peakSeries->clear();
    }
    for (QGraphicsTextItem* textItem : textItems) {
        if (textItem && chartView && chartView->scene()) {
            chartView->scene()->removeItem(textItem);
            delete textItem;
        }
    }
    textItems.clear();

    // 3. 安全地清空Mapping的图表显示
    QList<QAbstractSeries*> seriesToRemove;
    QList<QAbstractSeries*> allSeries = chart->series();

    for (QAbstractSeries* s : allSeries) {
        if (s != series && s != peakSeries) {
            seriesToRemove.append(s);
        }
    }

    // 安全删除系列
    for (QAbstractSeries* s : seriesToRemove) {
        chart->removeSeries(s);
        s->deleteLater(); // 安全删除
    }

    // 4. 重置图表显示范围
    if (xAxis) xAxis->setRange(300, 2000);
    if (yAxis) yAxis->setRange(0, 2000);

    // 5. 更新界面
    chartView->update();
}

void MainWindow::on_savedraw_clicked() // 保存绘图
{
    QPixmap pixmap = chartView->grab();
    QString defaultPath = "/home/picture/"; // 默认保存路径
    QString currentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
    QString fileName = currentTime + "_chart.png";
    QString filePath = QFileDialog::getSaveFileName(this, "Save Chart", defaultPath + fileName, "PNG (*.png)");

    if (!filePath.isEmpty()) {
        pixmap.save(filePath);
    }
}

void MainWindow::on_savedata_clicked() // 保存数据
{

    // 确保主光谱可见
    if (series) {
        series->setVisible(true);
    }

    QString defaultPath = "/home/Datas/"; // 默认保存路径
    QString currentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
    QString fileName = currentTime + "_data.txt";
    QString filePath = QFileDialog::getSaveFileName(this, "Save Data", defaultPath + fileName, "Text (*.txt)");

    if (!filePath.isEmpty()) {
        QFile file(filePath);
        if (file.open(QIODevice::WriteOnly)) {
            QTextStream out(&file);
            for (int i = 0; i < series->count(); ++i) {
                QPointF point = series->at(i);
                out << point.x() << "\t" << point.y() << "\n"; // 使用制表符分隔
            }
            file.close();
        } else {
            QMessageBox::critical(this, "Error", "Failed to open file for writing.");
        }
    }
}
void MainWindow::on_openfile_clicked()//打开数据
{
    chearflag=true;

    // 确保主光谱可见
    if (series) {
        series->setVisible(true);
    }

    QString filePath = QFileDialog::getOpenFileName(this, "Open File", "", "Text (*.txt)");
    if (!filePath.isEmpty()) {
        QFile file(filePath);
        if (file.open(QIODevice::ReadOnly)) {
            QTextStream in(&file);
            int lineCount = 0;
            while (!in.atEnd()) {
                in.readLine();
                lineCount++;
            }
            file.seek(0); // 重置文件指针到文件开头
            QVector<QPointF> points;
            points.reserve(lineCount); // 预分配空间

            double minx = std::numeric_limits<double>::max();
            double maxx = std::numeric_limits<double>::lowest();
            double miny = std::numeric_limits<double>::max();
            double maxy = std::numeric_limits<double>::lowest();
            // 批量加载数据
            while (!in.atEnd()) {
                QString line;
                in.readLineInto(&line); // 直接读取到QString中
                QStringList values = line.split('\t'); // 按制表符分割
                if (values.size() == 2) {
                    bool xOk, yOk;
                    double x = values[0].toDouble(&xOk);
                    double y = values[1].toDouble(&yOk);
                    if (xOk && yOk) {
                        minx = std::min(minx, x);
                        maxx = std::max(maxx, x);
                        miny = std::min(miny, y);
                        maxy = std::max(maxy, y);
                        points.append(QPointF(x, y));
                    }
                }
            }
            file.close();
            series->replace(points);
            xAxis->setRange(minx, maxx);
            yAxis->setRange(0, maxy*1.1);
            chartView->update();
        }
    }
}

void MainWindow::on_multiCapture_clicked()
{
    on_buttonClearChart_clicked();
    QDialog dialog(this);
    dialog.setWindowTitle("多重采集设置");
    dialog.setFixedSize(350, 300);
    dialog.setStyleSheet("QDialog { border: 2px solid black; background-color: white; }");

    QVBoxLayout layout(&dialog);

    // 标题
    QLabel *titleLabel = new QLabel("多重采集设置", &dialog);
    titleLabel->setAlignment(Qt::AlignCenter);
    QFont titleFont("Arial", 16, QFont::Bold);
    titleLabel->setFont(titleFont);

    // 曝光时间选择
    QHBoxLayout *exposureLayout = new QHBoxLayout();
    QLabel *exposureLabel = new QLabel("ExposureTime:", &dialog);
    QComboBox *exposureCombo = new QComboBox(&dialog);
    for (int i = 1; i <= 5; i++) {
        exposureCombo->addItem(QString("%1s").arg(i), i * 1000);
    }
    exposureCombo->setCurrentIndex(0); // 默认1秒
    exposureLayout->addWidget(exposureLabel);
    exposureLayout->addWidget(exposureCombo);

    // 采集次数选择 - 修改这里：从5提升到10
    QHBoxLayout *countLayout = new QHBoxLayout();
    QLabel *countLabel = new QLabel("采集次数:", &dialog);
    QComboBox *countCombo = new QComboBox(&dialog);
    for (int i = 1; i <= 10; i++) {  // 修改：从5改为10
        countCombo->addItem(QString("%1次").arg(i), i);
    }
    countCombo->setCurrentIndex(0); // 默认1次
    countLayout->addWidget(countLabel);
    countLayout->addWidget(countCombo);

    // 当前设置显示
    QLabel *currentLabel = new QLabel("当前设置: 曝光1秒, 采集1次", &dialog);
    currentLabel->setAlignment(Qt::AlignCenter);

    // 按钮布局
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    QPushButton *confirmButton = new QPushButton("开始采集", &dialog);
    QPushButton *cancelButton = new QPushButton("取消", &dialog);
    buttonLayout->addWidget(confirmButton);
    buttonLayout->addWidget(cancelButton);

    // 添加到主布局
    layout.addWidget(titleLabel);
    layout.addSpacing(20);
    layout.addLayout(exposureLayout);
    layout.addLayout(countLayout);
    layout.addWidget(currentLabel);
    layout.addSpacing(20);
    layout.addLayout(buttonLayout);

    // 更新当前设置显示
    auto updateCurrentLabel = [exposureCombo, countCombo, currentLabel]() {
        QString exposureText = exposureCombo->currentText();
        QString countText = countCombo->currentText();
        currentLabel->setText(QString("当前设置: 曝光%1, 采集%2").arg(exposureText).arg(countText));
    };

    // 连接组合框变化信号
    connect(exposureCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [updateCurrentLabel](int index) { updateCurrentLabel(); });
    connect(countCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [updateCurrentLabel](int index) { updateCurrentLabel(); });

    // 连接按钮信号
    connect(confirmButton, &QPushButton::clicked, [&, exposureCombo, countCombo]() {
        int exposureTime = exposureCombo->currentData().toInt();
        int captureCount = countCombo->currentData().toInt();

        dialog.accept();
        performMultiCapture(exposureTime, captureCount);
    });

    connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);

    dialog.exec();
}

void MainWindow::updateMultiCaptureProgress()
{
    if (!m_multiCaptureData.isRunning) return;

    // 创建进度条对话框（如果不存在）
    if (!m_multiCaptureProgressDialog) {
        m_multiCaptureProgressDialog = new QProgressDialog(this);
        m_multiCaptureProgressDialog->setWindowTitle("多重采集进度");
        m_multiCaptureProgressDialog->setLabelText("采集中...");
        m_multiCaptureProgressDialog->setCancelButtonText("取消采集");
        m_multiCaptureProgressDialog->setRange(0, m_multiCaptureData.totalCount);
        m_multiCaptureProgressDialog->setMinimumDuration(0); // 立即显示
        m_multiCaptureProgressDialog->setWindowModality(Qt::WindowModal);

        // 连接取消按钮
        connect(m_multiCaptureProgressDialog, &QProgressDialog::canceled, this, [this]() {
            m_multiCaptureData.isRunning = false;
            hideMultiCaptureProgress();
            QMessageBox::information(this, "提示", "多重采集已被取消");
        });
    }

    // 更新进度
    m_multiCaptureProgressDialog->setValue(m_multiCaptureData.currentCount);
    m_multiCaptureProgressDialog->setLabelText(
                QString("采集中... %1/%2\n曝光时间: %3秒\n已完成: %4%")
                .arg(m_multiCaptureData.currentCount)
                .arg(m_multiCaptureData.totalCount)
                .arg(m_multiCaptureData.exposureTime / 1000.0)
                .arg((m_multiCaptureData.currentCount * 100) / m_multiCaptureData.totalCount));

    // 确保进度条可见
    if (!m_multiCaptureProgressDialog->isVisible()) {
        m_multiCaptureProgressDialog->show();
    }
}

void MainWindow::hideMultiCaptureProgress()
{
    if (m_multiCaptureProgressDialog) {
        m_multiCaptureProgressDialog->hide();
        m_multiCaptureProgressDialog->deleteLater();
        m_multiCaptureProgressDialog = nullptr;
    }
}

void MainWindow::performMultiCapture(int exposureTime, int captureCount)
{
    if (captureCount <= 0) return;

    // 保存原始状态
    m_silentMode = true;

    // 初始化多重采集数据
    m_multiCaptureData.allSpectra.clear();
    m_multiCaptureData.xData.clear();
    m_multiCaptureData.currentCount = 0;
    m_multiCaptureData.totalCount = captureCount;
    m_multiCaptureData.exposureTime = exposureTime;
    m_multiCaptureData.isRunning = true;

    // 显示进度条
    updateMultiCaptureProgress();

    // 使用专用信号连接
    static QMetaObject::Connection captureConn;
    if (captureConn) {
        disconnect(captureConn);
    }

    captureConn = connect(usbWorker, &UsbWorker::captureCompleted, this,
                          &MainWindow::onMultiCaptureDataReady, Qt::QueuedConnection);

    // 设置曝光时间并开始采集
    if (usbWorker) {
        QMetaObject::invokeMethod(usbWorker, "setExposureTime",
                                  Qt::QueuedConnection, Q_ARG(int, exposureTime));

        QTimer::singleShot(200, this, [this]() {
            QMetaObject::invokeMethod(usbWorker, "on_buttonExposure_clicked", Qt::QueuedConnection);

            QTimer::singleShot(500, this, [this]() {
                startNextMultiCapture();
            });
        });
    }
}

void MainWindow::displayAveragedSpectrum(const QVector<double>& spectrum, const QVector<double>& xData)
{
    if (spectrum.isEmpty() || xData.isEmpty()) {
        return;
    }

    QVector<QPointF> points;
    double miny = std::numeric_limits<double>::max();
    double maxy = -std::numeric_limits<double>::max();
    double minx = std::numeric_limits<double>::max();
    double maxx = -std::numeric_limits<double>::max();

    // 构建数据点
    for (int i = 0; i < xData.size(); i++) {
        double x = xData[i];
        double y = spectrum[i];
        // === 修正：835.932点使用前一个点的值 ===
        if (std::abs(x - 835.932) < 0.1 && i > 0) {
            y = spectrum[i-1];
        }
        points.append(QPointF(x, y));
        minx = std::min(minx, x);
        maxx = std::max(maxx, x);
        miny = std::min(miny, y);
        maxy = std::max(maxy, y);
    }

    // 更新图表
    series->replace(points);
    xAxis->setRange(minx, maxx);

    // 设置Y轴范围，确保有适当的边距
    if (maxy > miny) {
        yAxis->setRange(0, maxy * 1.1); // 最大值增加10%的边距
    } else {
        yAxis->setRange(0, 1000); // 备用范围
    }

    chartView->update();
}
// 新增：处理单次采集完成
void MainWindow::onMultiCaptureDataReady()
{
    // 添加状态检查
    if (!m_multiCaptureData.isRunning) {
        return;
    }

    // 防止超额采集的检查
    if (m_multiCaptureData.currentCount >= m_multiCaptureData.totalCount) {
        return;
    }

    // 保存当前光谱数据
    if (series->count() > 0) {
        QVector<double> currentSpectrum;
        for (int j = 0; j < series->count(); j++) {
            currentSpectrum.append(series->at(j).y());
        }
        m_multiCaptureData.allSpectra.append(currentSpectrum);

        // 保存X轴数据（只需要第一次）
        if (m_multiCaptureData.xData.isEmpty()) {
            for (int j = 0; j < series->count(); j++) {
                m_multiCaptureData.xData.append(series->at(j).x());
            }
        }
    }

    m_multiCaptureData.currentCount++;

    // 更新进度条
    updateMultiCaptureProgress();

    // 更新状态栏显示进度
    if (statusBar()) {
        statusBar()->showMessage(QString("多重采集中... %1/%2 (曝光:%3ms)")
                                 .arg(m_multiCaptureData.currentCount)
                                 .arg(m_multiCaptureData.totalCount)
                                 .arg(m_multiCaptureData.exposureTime));
    }


    // 检查是否完成所有采集
    if (m_multiCaptureData.currentCount >= m_multiCaptureData.totalCount) {
        QTimer::singleShot(100, this, &MainWindow::finishMultiCapture);
    } else {
        // 继续下一次采集，添加基于曝光时间的延迟
        int delay = qMax(500, m_multiCaptureData.exposureTime / 2);
        QTimer::singleShot(delay, this, &MainWindow::startNextMultiCapture);
    }
}

void MainWindow::finishMultiCapture()
{
    // 立即停止采集状态
    m_multiCaptureData.isRunning = false;
    m_silentMode = true;

    // 隐藏进度条
    hideMultiCaptureProgress();

    // 清空状态栏消息
    if (statusBar()) {
        statusBar()->clearMessage();
    }

    // 立即断开专用信号连接
    disconnect(usbWorker, &UsbWorker::captureCompleted, this, &MainWindow::onMultiCaptureDataReady);

    // 计算平均值并显示（只使用有效的前 totalCount 个光谱）
    int validSpectraCount = qMin(m_multiCaptureData.allSpectra.size(), m_multiCaptureData.totalCount);
    if (validSpectraCount > 0 && !m_multiCaptureData.xData.isEmpty()) {
        QVector<double> averagedSpectrum(m_multiCaptureData.xData.size(), 0.0);

        // 计算平均值（只使用有效的光谱）
        for (int i = 0; i < m_multiCaptureData.xData.size(); i++) {
            double sum = 0.0;
            for (int j = 0; j < validSpectraCount; j++) {
                if (i < m_multiCaptureData.allSpectra[j].size()) {
                    sum += m_multiCaptureData.allSpectra[j][i];
                }
            }
            averagedSpectrum[i] = sum / validSpectraCount;
        }
        // 显示平均值光谱
        displayAveragedSpectrum(averagedSpectrum, m_multiCaptureData.xData);

        QMessageBox::information(this, "完成",
                                 QString("多重采集完成!\n采集次数: %1次\n有效数据: %2个\n曝光时间: %3秒")
                                 .arg(m_multiCaptureData.totalCount)
                                 .arg(validSpectraCount)
                                 .arg(m_multiCaptureData.exposureTime / 1000.0));
    } else {
        QMessageBox::warning(this, "错误", "多重采集失败，未获取到有效数据");
    }

    // 重置曝光时间
    if (usbWorker) {
        QMetaObject::invokeMethod(usbWorker, "setExposureTime",
                                  Qt::QueuedConnection, Q_ARG(int, 1000));
    }
}

void MainWindow::startNextMultiCapture()
{

    // 添加防重复检查
    static bool isStartingCapture = false;
    if (isStartingCapture) {
        return;
    }

    if (!m_multiCaptureData.isRunning) {
        return;
    }

    if (m_multiCaptureData.currentCount >= m_multiCaptureData.totalCount) {
        finishMultiCapture();
        return;
    }

    isStartingCapture = true;

    // 设置超时保护
    QTimer::singleShot(m_multiCaptureData.exposureTime + 5000, this, [this]() {
        if (m_multiCaptureData.isRunning && m_multiCaptureData.currentCount < m_multiCaptureData.totalCount) {
            onMultiCaptureDataReady(); // 强制进入下一次采集
        }
    });

    // 执行单次采集
    QMetaObject::invokeMethod(usbWorker, "on_buttonUsbCapture_clicked", Qt::QueuedConnection);

    // 重置防重复标志（使用单次定时器确保不会立即重置）
    QTimer::singleShot(100, this, [&isStartingCapture]() {
        isStartingCapture = false;
    });
}



void MainWindow::on_buttonReset_clicked()
{
    // 调用 resetZAxis 方法重置 Z 轴和状态变量
    resetZAxis();

    // 断开所有与自动对焦相关的信号连接
    disconnect(this, &MainWindow::afDataReady, this, nullptr);
    disconnect(this, &MainWindow::dataProcessed, this, nullptr); // 修改：断开自己的信号

    // 清空 Mapping 数据缓存
    m_mappingSpectra.clear();
    m_mappingWave.clear();
    m_mappingPeaks.clear();
    m_mappingStep = 0;

    // 清空其他状态
    m_mappingRunning = false;
    m_mappingPoints = 0;

    // 清空图表中的所有文本标注
    for (QGraphicsTextItem* textItem : textItems) {
        if (textItem && chartView && chartView->scene()) {
            chartView->scene()->removeItem(textItem);
            delete textItem; // 确保释放内存
        }
    }
    textItems.clear();

    // 清空峰值散点系列
    if (peakSeries) {
        if (chart && chart->series().contains(peakSeries)) {
            chart->removeSeries(peakSeries);
        }
        delete peakSeries; // 确保释放内存
        peakSeries = nullptr;
    }

    // 重置图表范围
    xAxis->setRange(300, 2000);
    yAxis->setRange(0, 2000);

    // 强制更新图表
    chartView->update();

    // 在合适位置重新连接信号
    connect(this, &MainWindow::dataProcessed, this, &MainWindow::handleDataProcessed);

    // 确保 peakSeries 被重新初始化
    if (!peakSeries) {
        peakSeries = new QScatterSeries();
        peakSeries->setName("Peaks");
        peakSeries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
        peakSeries->setMarkerSize(10);
        peakSeries->setPen(QPen(Qt::black, 2));
        peakSeries->setBrush(QBrush(Qt::blue));
    }
}
void MainWindow::on_buttonProcessSpectrum_clicked() {
    QDialog dialog(this);
    dialog.setFixedSize(200, 400); // 增加高度以容纳新按钮

    dialog.setStyleSheet("QDialog { border: 2px solid black; background-color: white; }");

    QVBoxLayout layout(&dialog);

    // 标题
    QLabel *titleLabel = new QLabel("选择处理操作", &dialog);
    titleLabel->setAlignment(Qt::AlignCenter);
    QFont titleFont("Arial", 16, QFont::Bold);
    titleLabel->setFont(titleFont);

    // 大按钮样式
    QString bigButtonStyle =
            "QPushButton { "
        "   background-color: lightgreen; "
        "   color: black; "
        "   border: 2px solid green; "
        "   font-size: 14px; "
        "   font-weight: bold; "
        "   min-height: 50px; "
        "   min-width: 150px; "
        "}"
        "QPushButton:hover { "
        "   background-color: #90EE90; "
        "}"
        "QPushButton:pressed { "
        "   background-color: #32CD32; "
        "}";

    // ========== 新增：Mapping批量处理按钮 ==========
    QString mappingButtonStyle =
            "QPushButton { "
        "   background-color: lightblue; "
        "   color: black; "
        "   border: 2px solid blue; "
        "   font-size: 14px; "
        "   font-weight: bold; "
        "   min-height: 50px; "
        "   min-width: 150px; "
        "}"
        "QPushButton:hover { "
        "   background-color: #87CEFA; "
        "}"
        "QPushButton:pressed { "
        "   background-color: #1E90FF; "
        "}";

    // 原有按钮
    QPushButton* smoothButton = new QPushButton("平滑操作", &dialog);
    smoothButton->setStyleSheet(bigButtonStyle);
    connect(smoothButton, &QPushButton::clicked, this, &MainWindow::on_smooth_clicked);

    QPushButton* baselineButton = new QPushButton("去基线", &dialog);
    baselineButton->setStyleSheet(bigButtonStyle);
    connect(baselineButton, &QPushButton::clicked, this, &MainWindow::on_baseline_clicked);

    QPushButton* findPeakButton = new QPushButton("寻峰", &dialog);
    findPeakButton->setStyleSheet(bigButtonStyle);
    connect(findPeakButton, &QPushButton::clicked, this, &MainWindow::on_findpeak_clicked);

    // ========== 新增：Mapping批量处理按钮 ==========
    QPushButton* smoothMappingButton = new QPushButton("批量平滑Mapping", &dialog);
    smoothMappingButton->setStyleSheet(mappingButtonStyle);
    connect(smoothMappingButton, &QPushButton::clicked, this, &MainWindow::on_smooth_mapping_clicked);

    QPushButton* baselineMappingButton = new QPushButton("批量去基线Mapping", &dialog);
    baselineMappingButton->setStyleSheet(mappingButtonStyle);
    connect(baselineMappingButton, &QPushButton::clicked, this, &MainWindow::on_baseline_mapping_clicked);

    QPushButton* cancelButton = new QPushButton("取消", &dialog);
    cancelButton->setStyleSheet(bigButtonStyle);
    connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::close);

    // 添加到布局
    layout.addWidget(titleLabel);
    layout.addSpacing(15);

    // 普通处理
    QLabel *normalLabel = new QLabel("单光谱处理:", &dialog);
    normalLabel->setFont(QFont("Arial", 12, QFont::Bold));
    layout.addWidget(normalLabel);
    layout.addWidget(smoothButton);
    layout.addSpacing(10);
    layout.addWidget(baselineButton);
    layout.addSpacing(10);
    layout.addWidget(findPeakButton);

    layout.addSpacing(20);

    // Mapping批量处理（只在有Mapping数据时显示）
    if (!m_mappingSpectra.isEmpty()) {
        QLabel *mappingLabel = new QLabel("Mapping批量处理:", &dialog);
        mappingLabel->setFont(QFont("Arial", 12, QFont::Bold));
        layout.addWidget(mappingLabel);
        layout.addWidget(smoothMappingButton);
        layout.addSpacing(10);
        layout.addWidget(baselineMappingButton);
        layout.addSpacing(20);
    }

    layout.addWidget(cancelButton);

    dialog.exec();
}
void MainWindow::on_baseline_clicked()
{
    const int n = series->count();
    if (n == 0) return;
    QVector<double> xData, yData;
    xData.reserve(n);
    yData.reserve(n);
    for (int i = 0; i < n; ++i) {
        xData.append(series->at(i).x());
        yData.append(series->at(i).y());
    }
    removeBaseline(yData, xData);
}

void MainWindow::on_smooth_clicked()
{
    const int n = series->count();
    if (n == 0) return;
    QVector<double> xData, yData;
    xData.reserve(n);
    yData.reserve(n);
    for (int i = 0; i < n; ++i) {
        xData.append(series->at(i).x());
        yData.append(series->at(i).y());
    }

    smoothData(yData, xData);
}


double MainWindow::gaussLorentz(int x, double x0, double sigma, double gamma, double amplitude) {
    // 高斯部分
    double gauss = amplitude * exp(-0.5 * pow((x - x0) / sigma, 2));
    // 洛伦兹部分
    double lorentz = amplitude * gamma * 2 / (pow((x - x0) * gamma / (2 * sigma), 2) + 1);
    // 高斯-洛伦兹混合
    return gauss + lorentz;
}
void MainWindow::on_findpeak_clicked() // 寻峰
{
    // 确保主光谱可见
    if (series) {
        series->setVisible(true);
    }

    // 确保数据不为空
    if (series->count() == 0) { // 使用 count() 检查数据点数量
        QMessageBox::warning(this, "警告！", "没有数据可供分析。请加载数据。");
        return;
    }

    // 提取数据
    QVector<double> originalData(series->count());
    for (int i = 0; i < series->count(); ++i) {
        originalData[i] = series->at(i).y();
    }

    // 设置高斯-洛伦兹模型参数
    double sigma = 1; // 高斯标准差
    double gamma = 1; // 洛伦兹半高宽
    double amplitude = 20.0; // 振幅

    // 存储检测到的峰值索引
    QList<int> peaks;

    // 调用高斯-洛伦兹寻峰算法
    gaussLorentzDetectPeaks(originalData, peaks, sigma, gamma, amplitude);

    // 如果未检测到峰值，提示用户
    if (peaks.isEmpty()) {
        QMessageBox::warning(this, "警告！", "没有足够的数据点进行峰值检测。请收集更多数据。");
        return;
    }

    // 对检测到的峰值进行排序，按强度降序排列
    std::sort(peaks.begin(), peaks.end(), [this](int a, int b) {
        return series->at(a).y() > series->at(b).y();
    });

    // 限制只绘制前五个最强的峰
    QList<int> topFivePeaks = peaks.mid(0, 5);

    // 清除旧的峰值图形项
    for (QGraphicsTextItem* textItem : textItems) {
        if (textItem && chartView && chartView->scene()) {
            chartView->scene()->removeItem(textItem);
            delete textItem;
        }
    }
    textItems.clear();

    // 清除旧的峰值散点系列
    if (peakSeries) {
        if (chart && chart->series().contains(peakSeries)) {
            chart->removeSeries(peakSeries);
        }
        peakSeries->clear();
    } else {
        peakSeries = new QScatterSeries();
        peakSeries->setName("Peaks");
        peakSeries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
        peakSeries->setMarkerSize(10);
        peakSeries->setPen(QPen(Qt::black, 2));
        peakSeries->setBrush(QBrush(Qt::blue));
    }

    // 绘制峰值散点
    for (int pos : topFivePeaks) {
        QPointF point = series->at(pos);
        peakSeries->append(point);
        // 添加文本标注
        QGraphicsTextItem* textItem = chartView->scene()->addText(QString::number(point.x(), 'f', 2), QFont("Arial", 12));
        textItem->setPos(chartView->chart()->mapToPosition(point) + QPointF(5, -25)); // 调整文本位置
        textItems.append(textItem);
    }

    // 添加峰值系列到图表
    chart->addSeries(peakSeries);

    // 将峰值系列附加到现有的坐标轴
    peakSeries->attachAxis(xAxis);
    peakSeries->attachAxis(yAxis);

    // 重绘图表以反映更新
    chartView->update();
    chearflag2 = true;
}

void MainWindow::gaussLorentzDetectPeaks(const QVector<double>& originalData, QList<int>& peaks, double sigma, double gamma, double amplitude)
{
    const int dataSize = originalData.size();
    QVector<double> sortedData = originalData; // 复制原始数据
    std::sort(sortedData.begin(), sortedData.end()); // 对数据进行排序

    // 计算中位数
    double median;
    if (dataSize % 2 == 0) {
        median = (sortedData[dataSize / 2 - 1] + sortedData[dataSize / 2]) / 2.0;
    } else {
        median = sortedData[dataSize / 2];
    }

    for (int i = 1; i < dataSize - 1; ++i) {
        if(i<=30)
        {
            continue;
        }
        if (originalData[i] > originalData[i - 1] && originalData[i] > originalData[i + 1]) {
            double peakHeight = originalData[i];

            // 检查峰值是否大于中位数
            if (peakHeight <= 1.8 * median) {
                continue;
            }

            // 检查峰值的宽度
            int startWidthIndex = i;
            while (startWidthIndex > 0 && originalData[startWidthIndex - 1] > peakHeight - (peakHeight / 2)) {
                startWidthIndex--;
            }
            int endWidthIndex = i;
            while (endWidthIndex < dataSize - 1 && originalData[endWidthIndex + 1] > peakHeight - (peakHeight / 2)) {
                endWidthIndex++;
            }
            int peakWidth = endWidthIndex - startWidthIndex + 1;

            // 峰值宽度检查
            if (peakWidth > 3 * peakHeight) {
                continue;
            }

            // 检查是否相邻峰值太近或寻找更高的峰值
            bool isPeakTooCloseOrLower = false;
            for (int peakIndex : peaks) {
                if (std::abs(peakIndex - i) < 30 * sigma) {
                    if (originalData[peakIndex] > peakHeight) {
                        isPeakTooCloseOrLower = true;
                        break;
                    } else {
                        peaks.removeOne(peakIndex);
                    }
                }
            }
            if (!isPeakTooCloseOrLower) {
                peaks.append(i);
                // 拟合高斯-洛伦兹混合峰并计算拟合误差
                double x0 = i; // 峰值中心位置
                double sumOfResiduals = 0;
                int fitPointsCount = 0;
                for (int j = startWidthIndex; j <= endWidthIndex; ++j) {
                    double fitValue = gaussLorentz(j, x0, sigma, gamma, amplitude);
                    sumOfResiduals += std::abs(originalData[j] - fitValue);
                    fitPointsCount++;
                }
            }
        }
    }
}

void MainWindow::gaussLorentzDetectPeaksForAF(const QVector<double>& originalData, QList<int>& peaks)
{
    const int dataSize = originalData.size();
    if (dataSize < 10) return;

    QVector<double> sortedData = originalData;
    std::sort(sortedData.begin(), sortedData.end());

    // 计算中位数 - 使用更宽松的阈值
    double median;
    if (dataSize % 2 == 0) {
        median = (sortedData[dataSize / 2 - 1] + sortedData[dataSize / 2]) / 2.0;
    } else {
        median = sortedData[dataSize / 2];
    }

    // 自动对焦专用参数 - 更宽松的设置
    const double INTENSITY_THRESHOLD = 1.2;      // 降低强度阈值 (原1.8 → 1.2)
    const int MIN_PEAK_DISTANCE = 15;            // 减小最小间距 (原30 → 15)
    const int SKIP_POINTS = 10;                  // 减少跳过的点数 (原30 → 10)
    const double MAX_RELATIVE_WIDTH = 0.15;      // 放宽宽度限制

    for (int i = SKIP_POINTS; i < dataSize - 1; ++i) {
        // 基本峰值条件：局部最大值
        if (originalData[i] > originalData[i - 1] && originalData[i] > originalData[i + 1]) {
            double peakHeight = originalData[i];

            // 条件1：更宽松的强度阈值
            if (peakHeight <= INTENSITY_THRESHOLD * median) {
                continue;
            }

            // 条件2：计算半高宽（使用更宽松的阈值）
            int startWidthIndex = i;
            while (startWidthIndex > 0 &&
                   originalData[startWidthIndex - 1] > peakHeight * 0.3) { // 降低到30%高度
                startWidthIndex--;
            }
            int endWidthIndex = i;
            while (endWidthIndex < dataSize - 1 &&
                   originalData[endWidthIndex + 1] > peakHeight * 0.3) { // 降低到30%高度
                endWidthIndex++;
            }
            int peakWidth = endWidthIndex - startWidthIndex + 1;

            // 条件3：使用相对宽度判断，更宽松
            double relativeWidth = (double)peakWidth / dataSize;
            if (relativeWidth > MAX_RELATIVE_WIDTH) {
                continue;
            }

            // 条件4：更宽松的峰值间距检查
            bool isPeakTooClose = false;
            for (int peakIndex : peaks) {
                if (std::abs(peakIndex - i) < MIN_PEAK_DISTANCE) {
                    // 保留更强的峰值
                    if (originalData[peakIndex] >= peakHeight) {
                        isPeakTooClose = true;
                        break;
                    } else {
                        peaks.removeOne(peakIndex);
                    }
                }
            }

            if (!isPeakTooClose) {
                peaks.append(i);
            }
        }
    }

    // 如果没有找到峰值，尝试更宽松的策略
    if (peaks.isEmpty()) {

        // 备用策略：直接找最强的几个局部最大值
        QList<QPair<int, double>> candidatePeaks;
        for (int i = SKIP_POINTS; i < dataSize - 1; ++i) {
            if (originalData[i] > originalData[i - 1] && originalData[i] > originalData[i + 1]) {
                if (originalData[i] > median * 0.8) { // 进一步降低阈值
                    candidatePeaks.append(qMakePair(i, originalData[i]));
                }
            }
        }

        // 按强度排序，取前几个
        std::sort(candidatePeaks.begin(), candidatePeaks.end(),
                  [](const QPair<int, double>& a, const QPair<int, double>& b) {
            return a.second > b.second;
        });

        for (int i = 0; i < qMin(3, candidatePeaks.size()); ++i) {
            peaks.append(candidatePeaks[i].first);
        }
    }
}

void MainWindow::runAutoFocusStateMachine()
{
    // 安全检查：如果自动对焦已停止，直接返回
    if (!m_autoFocusRunning) {
        return;
    }

    // 检查迭代次数，防止无限循环
    if (m_afIteration++ >= MAX_AF_ITERATIONS) {
        m_afIteration = 0;
        QTimer::singleShot(100, this, &MainWindow::finishAutoFocus);
        return;
    }

    /* 1. 主线程copy X+Y */
    const int n = series->count();
    if (n < 3) {
        QTimer::singleShot(100, this, &MainWindow::finishAutoFocus);
        return;
    }

    QVector<double> xData, yData;
    xData.reserve(n);
    yData.reserve(n);
    for (int i = 0; i < n; ++i) {
        xData.append(series->at(i).x());
        yData.append(series->at(i).y());
    }

    /* 2. 首先检查832~839范围内是否存在全局最大值 */
    double globalMax = *std::max_element(yData.begin(), yData.end());
    bool maxInRange = false;

    // 使用二分查找优化查找速度
    // 找到832附近的位置
    auto startIt = std::lower_bound(xData.begin(), xData.end(), 832);
    auto endIt = std::upper_bound(xData.begin(), xData.end(), 839);

    if (startIt != xData.end() && endIt != xData.begin()) {
        int startIdx = startIt - xData.begin();
        int endIdx = endIt - xData.begin() - 1;

        for (int i = startIdx; i <= endIdx && i < xData.size(); ++i) {
            if (yData[i] == globalMax) {
                maxInRange = true;
                break;
            }
        }
    }

    // 如果832~839范围内存在全局最大值，直接快速上升并继续采集
    if (maxInRange) {
        // 快速上升 - 使用fastup.sh
        QProcess::execute("/home/initmoto.sh");
        QProcess::execute("/home/fastup.sh");
        g_zAxisPosition += 25 * 8;  // fastup.sh也是移动25单位
        zAxisLabel->setText(QString("Z轴：%1").arg(g_zAxisPosition));

        // 立即请求下一次采集
        QTimer::singleShot(0, this, [this]() {
            QMetaObject::invokeMethod(usbWorker, "on_buttonUsbCapture_clicked", Qt::QueuedConnection);
        });
        return;
    }

    /* 3. 如果832~839范围内不存在全局最大值，执行正常的平滑+去基线流程 */
    QVector<double> bl;

    // 修改：直接同步调用，不再需要事件循环和信号连接
    bl = performSmoothing(yData);  // 直接调用平滑函数
    if (bl.isEmpty()) {
        QTimer::singleShot(0, this, &MainWindow::finishAutoFocus);
        return;
    }

    // 修改：直接同步调用去基线函数
    bl = performBaselineCorrection(bl);  // 直接调用去基线函数
    if (bl.isEmpty()) {
        QTimer::singleShot(0, this, &MainWindow::finishAutoFocus);
        return;
    }

    /* 4. 寻峰 + 评分 */
    QList<int> peaks;
    gaussLorentzDetectPeaks(bl, peaks, 1, 1, 20.0);

    double score = 0.0;
    if (!peaks.isEmpty()) {
        QList<double> peakValues;
        for (int i : peaks) {
            if (i < 0 || i >= bl.size()) continue;
            peakValues << bl[i];
        }
        std::sort(peakValues.begin(), peakValues.end(), std::greater<double>());

        // 计算前3个最强峰的平均值作为评分
        int count = qMin(3, peakValues.size());
        for (int i = 0; i < count; ++i) {
            score += peakValues[i];
        }
        score /= count;
    } else {
        score = 0.0;
    }


    // 处理对焦逻辑 - 使用成员变量
    handleCoarseSearch(score, m_afContext);
    zAxisLabel->setText(QString("Z轴：%1").arg(g_zAxisPosition));

    // 继续采集数据（只有在对焦未完成时才继续）
    if (m_autoFocusRunning && !m_afContext.peakFound) {
        QTimer::singleShot(0, this, [this]() {
            QMetaObject::invokeMethod(usbWorker, "on_buttonUsbCapture_clicked", Qt::QueuedConnection);
        });
    }
}


void MainWindow::handleCoarseSearch(double score, AutoFocusContext& ctx)
{
    const double SIGNIFICANT_INCREASE_THRESHOLD = 1.05;  // 显著增加阈值 5%
    const double SIGNIFICANT_DECREASE_THRESHOLD = 0.85;  // 显著减少阈值 15%

    if (ctx.first) {
        // 第一次迭代
        ctx.first = false;
        ctx.lastScore = score;
        ctx.bestScore = score;
        ctx.bestPosition = g_zAxisPosition;
        ctx.peakFound = false;

        //qDebug() << "[AF] First iteration, moving up from Z =" << g_zAxisPosition;

        QProcess::execute("/home/initmoto.sh");
        QProcess::execute("/home/up.sh");
        g_zAxisPosition += 25;

        //qDebug() << "[AF] Now at Z =" << g_zAxisPosition;
        return;
    }

    // 更新最佳分数和位置
    if (score > ctx.bestScore) {
        // qDebug() << "[AF] New best score:" << score << "at Z:" << g_zAxisPosition;
        ctx.bestScore = score;
        ctx.bestPosition = g_zAxisPosition;
    }

    if (!ctx.peakFound) {
        double ratio = (ctx.lastScore > 0) ? score / ctx.lastScore : 1.0;
        //qDebug() << "[AF] Score ratio:" << ratio;

        // 阶段1：寻找峰值点（评分开始下降的点）
        if (score >= ctx.lastScore * SIGNIFICANT_INCREASE_THRESHOLD) {
            // 评分显著增加，继续向上
            //qDebug() << "[AF] Significant increase, continuing up";
            ctx.lastScore = score;
            QProcess::execute("/home/up.sh");
            g_zAxisPosition += 25;
            return;
        } else if (score <= ctx.lastScore * SIGNIFICANT_DECREASE_THRESHOLD) {
            // 评分显著下降，标记找到峰值点
            ctx.peakFound = true;

            // 回退到最佳位置
            int stepsBack = (g_zAxisPosition - ctx.bestPosition) / 25;


            for (int i = 0; i < stepsBack; ++i) {
                QProcess::execute("/home/down.sh");
                QThread::msleep(5);
                g_zAxisPosition -= 25;
                //qDebug() << "[AF] Step back" << (i+1) << "/" << stepsBack << "Z =" << g_zAxisPosition;
            }

            // 直接完成自动对焦
            QTimer::singleShot(5, this, &MainWindow::finishAutoFocus);
            return;
        } else {
            // 评分变化不大，继续向上移动
            //qDebug() << "[AF] No significant change, continuing up";
            ctx.lastScore = score;
            QProcess::execute("/home/up.sh");
            g_zAxisPosition += 25;
            QThread::msleep(5);
            return;
        }
    }

    // 如果已经找到峰值点但还在运行，直接停止（安全机制）
    qDebug() << "[AF] Peak already found but still running, finishing...";
    finishAutoFocus();
}


void MainWindow::moveToPosition(int targetPosition)
{
    int currentPos = g_zAxisPosition;
    int steps = abs(targetPosition - currentPos) / 25;

    if (targetPosition > currentPos) {
        for (int i = 0; i < steps; ++i) {
            QProcess::execute("/home/up.sh");
        }
    } else {
        for (int i = 0; i < steps; ++i) {
            QProcess::execute("/home/down.sh");
        }
    }
    g_zAxisPosition = targetPosition;
}

void MainWindow::calculateFinalBestPosition()
{
    if (m_fineScores.size() != 3) {
        qDebug() << "[AF Fine] Error: Expected 3 scores, got" << m_fineScores.size();
        return;
    }

    // 计算三个位置的平均评分
    double avgScore = (m_fineScores[0] + m_fineScores[1] + m_fineScores[2]) / 3.0;

    // 找出评分最高的位置
    int bestIndex = 0;
    double maxScore = m_fineScores[0];
    for (int i = 1; i < 3; ++i) {
        if (m_fineScores[i] > maxScore) {
            maxScore = m_fineScores[i];
            bestIndex = i;
        }
    }

    // 计算最终最佳位置
    int finalPosition = m_bestPosition;
    if (bestIndex == 1) {
        // 最佳点下方评分最高
        finalPosition = m_bestPosition - 25;
    } else if (bestIndex == 2) {
        // 最佳点上方评分最高
        finalPosition = m_bestPosition + 25;
    }

    // 移动到最终最佳位置
    moveToPosition(finalPosition);


}

void MainWindow::on_autoFocus_clicked()
{
    // 如果已有操作在进行中，直接返回
    if (m_operationInProgress) {
        QMessageBox::information(this, "提示", "当前有操作正在执行，请稍后再试");
        return;
    }

    resetZAxis();

    // === 清空图表数据 ===
    if (series) {
        series->clear();
    }
    if (peakSeries) {
        peakSeries->clear();
    }

    // 设置操作进行中标志
    m_operationInProgress = true;

    // 禁用所有按钮
    setAllButtonsEnabled(false);

    // 显示简单提示
    QMessageBox::information(this, "自动对焦", "自动对焦中...\n找到焦点后自动退出");

    // 重置自动对焦上下文
    m_afContext.first = true;
    m_afContext.up = true;
    m_afContext.lastScore = -1;
    m_afContext.bestScore = 0.0;
    m_afContext.bestPosition = 0;
    m_afContext.peakFound = false;

    // 重置状态
    m_silentMode = true;  // 静默模式，不绘图
    m_autoFocusRunning = true;
    m_afIteration = 0;
    m_fineSampling = false;
    m_samplingStep = 0;
    m_fineScores.clear();

    // 连接数据准备好信号
    disconnect(this, &MainWindow::afDataReady, this, nullptr); // 先断开所有连接

    connect(this, &MainWindow::afDataReady, this, [this]() {
        if (m_autoFocusRunning) {
            runAutoFocusStateMachine();
        }
    }, Qt::QueuedConnection);

    qDebug() << "[AF] Starting auto focus with fresh context...";

    // 开始自动对焦流程
    if (series->count() > 0) {
        QTimer::singleShot(0, this, &MainWindow::runAutoFocusStateMachine);
    } else {
        QTimer::singleShot(0, this, [this]() {
            QMetaObject::invokeMethod(usbWorker, "on_buttonUsbCapture_clicked", Qt::QueuedConnection);
        });
    }
}
void MainWindow::resetZAxis()
{
    // 先停止自动对焦
    m_autoFocusRunning = false;

    int steps = g_zAxisPosition / 25;
    qDebug() << "Resetting Z axis from" << g_zAxisPosition << "to 0, steps:" << steps;

    for (int i = 0; i < steps; ++i) {
        QProcess::execute("/home/down.sh");
        QThread::msleep(0);  // 减少延迟
        g_zAxisPosition -= 25;
        zAxisLabel->setText(QString("Z轴：%1").arg(g_zAxisPosition));
        qDebug() << "Auto focus reset: Moving down... Current Z:" << g_zAxisPosition;
    }

    g_zAxisPosition = 0;
    zAxisLabel->setText(QString("Z轴：%1").arg(g_zAxisPosition));

    // 重置所有状态变量
    m_silentMode = true;
    m_afIteration = 0;
    m_fineSampling = false;
    m_samplingStep = 0;
    m_fineScores.clear();
    m_bestScore = 0.0;
    m_bestPosition = 0;

    // 重置自动对焦上下文
    m_afContext.first = true;
    m_afContext.up = true;
    m_afContext.lastScore = -1;
    m_afContext.bestScore = 0.0;
    m_afContext.bestPosition = 0;
    m_afContext.peakFound = false;

    // 断开信号连接
    disconnect(this, &MainWindow::afDataReady, this, nullptr);

    qDebug() << "Auto focus reset complete. Z axis at 0, context reset.";
}


void MainWindow::finishAutoFocus()
{
    // 确保主光谱可见
    if (series) {
        series->setVisible(true);
    }

    m_autoFocusRunning = false;
    m_fineSampling = false;
    m_silentMode = false;  // 退出静默模式

    // 断开自动对焦信号连接
    disconnect(this, &MainWindow::afDataReady, this, nullptr);

    m_operationInProgress = false;

    // 重新启用所有按钮
    setAllButtonsEnabled(true);

    qDebug() << "[AF] Auto focus completed at Z =" << g_zAxisPosition;

    // 在最佳焦点位置执行一次采集并显示
    QMetaObject::invokeMethod(usbWorker, "on_buttonUsbCapture_clicked", Qt::QueuedConnection);

    // 等待数据采集完成
    QEventLoop loop;
    QMetaObject::Connection dataConn = connect(this, &MainWindow::mappingDataReady, &loop, &QEventLoop::quit);
    loop.exec();
    disconnect(dataConn);

    // 短暂延迟确保数据完全处理
    QThread::msleep(200);

    // 执行去基线处理并显示
    qDebug() << "[AF] Performing baseline removal on final spectrum";
    on_baseline_clicked();

    // 最终绘图
    QTimer::singleShot(300, this, [this]() {
        // 确保数据已经处理完成
        QVector<QPointF> points;
        for (int i = 0; i < series->count(); ++i) {
            points.append(series->at(i));
        }
        if (!points.isEmpty()) {
            // === 修正：835.932点使用前一个点的值 ===
            for (int i = 1; i < points.size(); ++i) {
                if (std::abs(points[i].x() - 835.932) < 0.1) {
                    points[i].setY(points[i-1].y());
                    break;  // 修改一个点后退出
                }
            }
            series->replace(points);
            chartView->update();
        }
    });
}

void MainWindow::on_mapping_clicked()
{
    // ==================== Mapping参数初始化复原 ====================

    // 1. 停止可能正在运行的Mapping
    m_mappingRunning = false;

    // 2. 清空所有Mapping数据缓存
    m_mappingSpectra.clear();
    m_mappingWave.clear();
    m_mappingPeaks.clear();
    m_mappingAvg.clear();
    m_mappingIntensities.clear();

    // 3. 重置Mapping状态变量
    m_mappingStep = 0;
    m_mappingPoints = 0;

    // 4. 清空临时批量处理数据
    m_tempBatchSpectra.clear();
    m_tempBatchXData.clear();
    m_isBatchDataReady = false;
    m_isBatchProcessing = false;
    m_shouldClearTempOnNextOp = true;

    // 5. 重置Mapping处理标志
    isMappingDataPresent = false;
    isMappingProcessed = false;

    // 6. 清空图表中的Mapping相关显示（保留主光谱）
    if (chart) {
        QList<QAbstractSeries*> allSeries = chart->series();
        for (QAbstractSeries* series : allSeries) {
            // 保留主光谱系列和峰值系列，移除其他系列
            if (series != this->series && series != peakSeries) {
                chart->removeSeries(series);
                delete series;
            }
        }
    }

    // 7. 清空峰值标注
    for (QGraphicsTextItem* textItem : textItems) {
        if (textItem && chartView && chartView->scene()) {
            chartView->scene()->removeItem(textItem);
            delete textItem;
        }
    }
    textItems.clear();

    // 8. 重置峰值系列
    if (peakSeries) {
        peakSeries->clear();
    }

    // 9. 重置图表显示范围
    if (xAxis && yAxis) {
        xAxis->setRange(300, 2000);
        yAxis->setRange(0, 2000);
    }

    // 10. 更新界面
    if (chartView) {
        chartView->update();
    }

    // 11. 清空状态栏消息
    if (statusBar()) {
        statusBar()->clearMessage();
    }

    qDebug() << "Mapping parameters reset complete, starting square mapping...";
    on_squareMapping_clicked();
}

void MainWindow::on_squareMapping_clicked()
{
    m_operationInProgress = true;

    // 禁用所有用户交互按钮
    setAllButtonsEnabled(false);

    // 显示简单提示
    QMessageBox::information(this, "Mapping开始", "Mapping采集中...\n完成后自动显示结果");

    // 开始Mapping
    QTimer::singleShot(100, this, &MainWindow::doSquare9Mapping);
}
void MainWindow::doSquare9Mapping()
{
    /* 1. 重置状态 */
    m_mappingSpectra.clear();
    m_mappingWave.clear();
    m_mappingPeaks.clear();
    m_mappingRunning = true;
    m_silentMode = true;  // 静默模式，不绘图

    /* 2. 依次执行9步移动并采谱（蛇形路径） */

    // 第1行：左→右（3步）
    executeMappingStep("right.sh", 800);
    if (!m_mappingRunning) return;

    executeMappingStep("right.sh", 800);
    if (!m_mappingRunning) return;

    executeMappingStep("right.sh", 800);
    if (!m_mappingRunning) return;

    // 第2行：右→左（3步）
    executeMappingStep("front.sh", 800);
    if (!m_mappingRunning) return;

    executeMappingStep("left.sh", 800);
    if (!m_mappingRunning) return;

    executeMappingStep("left.sh", 800);
    if (!m_mappingRunning) return;

    // 第3行：左→右（3步）
    executeMappingStep("front.sh", 800);
    if (!m_mappingRunning) return;

    executeMappingStep("right.sh", 800);
    if (!m_mappingRunning) return;

    executeMappingStep("right.sh", 800);
    if (!m_mappingRunning) return;

    /* 3. 归正：back 2 次 + left 2 次 */
    if (m_mappingRunning) {
        for (int i = 0; i < 2; ++i) {
            QProcess::execute("/home/back.sh");
            QThread::msleep(500);
        }
        for (int i = 0; i < 2; ++i) {
            QProcess::execute("/home/left.sh");
            QThread::msleep(500);
        }
    }

    /* 4. 统一绘图 */
    finishMapping();
}

void MainWindow::executeMappingStep(const QString& script, int msDelay)
{
    if (!m_mappingRunning) return;

    // 1. 移动平台
    QProcess::execute("/home/" + script);
    if (msDelay > 0)
        QThread::msleep(msDelay);

    // 2. 采谱
    QMetaObject::invokeMethod(usbWorker, "on_buttonUsbCapture_clicked", Qt::QueuedConnection);
    QEventLoop loop;
    QMetaObject::Connection conn =
            connect(this, &MainWindow::mappingDataReady, &loop, &QEventLoop::quit);
    loop.exec();
    disconnect(conn);

    // 3. 平滑+去基线（直接同步调用）
    const int n = series->count();
    if (n < 3) return;
    QVector<double> xData, yData;
    for (int i = 0; i < n; ++i) {
        xData.append(series->at(i).x());
        yData.append(series->at(i).y());
    }

    // 修改：直接同步调用，不再需要事件循环和信号连接
    QVector<double> bl = performSmoothing(yData);  // 直接调用平滑函数
    if (bl.isEmpty()) return;

    bl = performBaselineCorrection(bl);  // 直接调用去基线函数
    if (bl.isEmpty()) return;

    // 4. 记录标尺（第一张光谱）
    if (m_mappingWave.isEmpty()) {
        m_mappingWave = xData;
        QList<int> peaks;
        gaussLorentzDetectPeaks(bl, peaks, 1, 1, 20.0);
        m_mappingPeaks = peaks;
    }

    // 5. 存光谱
    m_mappingSpectra.append(bl);
}
void MainWindow::startMapping(const QString &scriptPath, int totalSteps)
{
    // 重置状态
    m_mappingSpectra.clear();
    m_mappingWave.clear();
    m_mappingPeaks.clear();
    m_mappingStep = 0;
    m_mappingRunning = true;
    m_silentMode = true;

    // 一次性执行完所有步进
    for (; m_mappingStep < totalSteps; ++m_mappingStep) {
        // 1. 移动平台
        QProcess::execute(scriptPath);
        QThread::msleep(1000);

        // 2. 采谱
        QMetaObject::invokeMethod(usbWorker, "on_buttonUsbCapture_clicked", Qt::QueuedConnection);
        QEventLoop loop;
        QMetaObject::Connection conn =
                connect(this, &MainWindow::mappingDataReady, &loop, &QEventLoop::quit);
        loop.exec();
        disconnect(conn);

        // 3. 平滑+去基线（直接同步调用）
        const int n = series->count();
        if (n < 3) continue;
        QVector<double> xData, yData;
        for (int i = 0; i < n; ++i) {
            xData.append(series->at(i).x());
            yData.append(series->at(i).y());
        }

        // 修改：直接同步调用，不再需要事件循环和信号连接
        QVector<double> bl = performSmoothing(yData);  // 直接调用平滑函数
        if (bl.isEmpty()) continue;

        bl = performBaselineCorrection(bl);  // 直接调用去基线函数
        if (bl.isEmpty()) continue;

        // 4. 记录标尺（第一张光谱）
        if (m_mappingWave.isEmpty()) {
            m_mappingWave = xData;
            QList<int> peaks;
            gaussLorentzDetectPeaks(bl, peaks, 1, 1, 20.0);
            m_mappingPeaks = peaks;
        }

        // 5. 存光谱
        m_mappingSpectra.append(bl);
    }

    finishMapping();   // 统一绘图
}
void MainWindow::finishMapping()
{
    m_mappingRunning = false;
    m_silentMode = false;
    m_operationInProgress = false;

    // 重新启用所有按钮
    setAllButtonsEnabled(true);

    if (m_mappingSpectra.isEmpty() || m_mappingWave.isEmpty()) {
        QMessageBox::warning(this, "错误", "Mapping采集失败，未获取到有效数据");
        return;
    }

    // 绘制所有光谱
    plotMappingSpectra();
}
void MainWindow::plotMappingSpectra()
{
    // === 修复：安全地移除Mapping相关系列，保留主光谱和峰值系列 ===
    QList<QAbstractSeries*> seriesToRemove;
    QList<QAbstractSeries*> allSeries = chart->series();

    // 只移除Mapping相关的系列，保留主光谱和峰值系列
    for (QAbstractSeries* s : allSeries) {
        if (s != series && s != peakSeries) {
            seriesToRemove.append(s);
        }
    }

    // 安全移除系列
    for (QAbstractSeries* s : seriesToRemove) {
        chart->removeSeries(s);
        s->deleteLater(); // 使用deleteLater安全删除
    }

    // 如果主光谱系列不在图表中，重新添加
    if (!allSeries.contains(series)) {
        chart->addSeries(series);
        series->attachAxis(xAxis);
        series->attachAxis(yAxis);
    }

    // === 关键修改：隐藏主光谱 ===
    if (series) {
        series->setVisible(false);
    }

    // 如果峰值系列不在图表中，重新添加
    if (peakSeries && !allSeries.contains(peakSeries)) {
        chart->addSeries(peakSeries);
        peakSeries->attachAxis(xAxis);
        peakSeries->attachAxis(yAxis);
    }

    // 隐藏峰值系列
    if (peakSeries) {
        peakSeries->setVisible(false);
    }

    // 计算所有数据的Y轴范围
    double globalMinY = std::numeric_limits<double>::max();
    double globalMaxY = std::numeric_limits<double>::lowest();

    // ========== 只绘制每个光谱 ==========
    for (int i = 0; i < m_mappingSpectra.size(); ++i) {
        if (i >= 9) break; // 只显示9条光谱

        const auto& spec = m_mappingSpectra[i];
        QLineSeries* mappingSeries = new QLineSeries();
        mappingSeries->setName(QString("Mapping %1").arg(i + 1));
        mappingSeries->setPen(QPen(Qt::red, 1.0));

        for (int j = 0; j < m_mappingWave.size(); ++j) {
            double x = m_mappingWave[j];
            double yValue = spec[j];

            // === 修正：835.932点使用前一个点的值 ===
            if (j > 0 && std::abs(x - 835.932) < 0.1) {
                yValue = spec[j - 1];  // 使用前一个点的值
            }

            mappingSeries->append(x, yValue);

            // 更新全局Y轴范围
            globalMinY = std::min(globalMinY, yValue);
            globalMaxY = std::max(globalMaxY, yValue);
        }

        chart->addSeries(mappingSeries);
        mappingSeries->attachAxis(xAxis);
        mappingSeries->attachAxis(yAxis);
    }

    // ========== 绘制去掉最大值和最小值后的平均值光谱 ==========
    QLineSeries* avgSeries = new QLineSeries();
    avgSeries->setName("mapping光谱(去极值)");
    avgSeries->setPen(QPen(Qt::red, 2.0));

    for (int j = 0; j < m_mappingWave.size(); ++j) {
        double x = m_mappingWave[j];

        // 收集所有光谱在当前波长点的值
        QVector<double> values;
        for (int i = 0; i < m_mappingSpectra.size(); ++i) {
            if (j < m_mappingSpectra[i].size()) {
                double val = m_mappingSpectra[i][j];
                // === 对每个光谱的835.932点都使用前一个点的值 ===
                if (j > 0 && std::abs(x - 835.932) < 0.1) {
                    val = m_mappingSpectra[i][j - 1];
                }
                values.append(val);
            }
        }

        // 去掉最大值和最小值后计算平均值
        double avg = 0.0;
        if (values.size() >= 3) {
            std::sort(values.begin(), values.end());
            values.removeFirst(); // 去掉最小值
            values.removeLast();  // 去掉最大值

            double sum = 0.0;
            for (double val : values) {
                sum += val;
            }
            avg = sum / values.size();
        }
        else if (!values.isEmpty()) {
            // 数据点不足，直接计算平均值
            double sum = 0.0;
            for (double val : values) {
                sum += val;
            }
            avg = sum / values.size();
        }

        // === 这里也需要再次修正，确保平均光谱的835.932点正确 ===
        if (j > 0 && std::abs(x - 835.932) < 0.1) {
            // 重新计算前一个点的平均值
            QVector<double> prevValues;
            for (int i = 0; i < m_mappingSpectra.size(); ++i) {
                if ((j - 1) < m_mappingSpectra[i].size()) {
                    prevValues.append(m_mappingSpectra[i][j - 1]);
                }
            }

            if (prevValues.size() >= 3) {
                std::sort(prevValues.begin(), prevValues.end());
                prevValues.removeFirst();
                prevValues.removeLast();
                double sumPrev = 0.0;
                for (double val : prevValues) {
                    sumPrev += val;
                }
                avg = sumPrev / prevValues.size();
            }
            else if (!prevValues.isEmpty()) {
                double sumPrev = 0.0;
                for (double val : prevValues) {
                    sumPrev += val;
                }
                avg = sumPrev / prevValues.size();
            }
        }

        avgSeries->append(x, avg);

        // 平均值也计入范围计算
        globalMinY = std::min(globalMinY, avg);
        globalMaxY = std::max(globalMaxY, avg);
    }

    chart->addSeries(avgSeries);
    avgSeries->attachAxis(xAxis);
    avgSeries->attachAxis(yAxis);

    // === 修改：设置Y轴范围，最小值为光谱最小值的80% ===
    if (globalMaxY > globalMinY) {
        double minYRange = globalMinY * 0.8;
        double maxYRange = globalMaxY * 1.2;
        yAxis->setRange(minYRange, maxYRange);
    }
    else if (globalMaxY > 0) {
        yAxis->setRange(0, globalMaxY * 1.2);
    }
    else {
        yAxis->setRange(-100, 100);
    }

    // 设置X轴范围
    xAxis->setRange(m_mappingWave.first(), m_mappingWave.last());
    yAxis->setTitleText("Intensity");

    chartView->update();
}
void MainWindow::setAllButtonsEnabled(bool enabled)
{
    // 禁用或启用所有主要功能按钮
    laserButton->setEnabled(enabled);
    buttonUsbCapture->setEnabled(enabled);
    multiCaptureButton->setEnabled(enabled);
    analysisButton->setEnabled(enabled);
    buttonSaveOpen->setEnabled(enabled);
    buttonProcessSpectrum->setEnabled(enabled);
    buttonClearChart->setEnabled(enabled);
    autoFocus->setEnabled(enabled);
    mappingButton->setEnabled(enabled);
    buttonReset->setEnabled(enabled);
}

// 修改批量处理函数，直接调用本地函数
void MainWindow::on_smooth_mapping_clicked()
{
    if (m_mappingSpectra.isEmpty()) {
        QMessageBox::information(this, "提示", "没有Mapping数据可供处理");
        return;
    }

    QProgressDialog progress("批量平滑中...", "取消", 0, m_mappingSpectra.size(), this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0); // 立即显示

    // 使用临时容器，避免直接修改原始数据
    QVector<QVector<double>> tempSpectra = m_mappingSpectra;

    for (int i = 0; i < tempSpectra.size(); ++i) {
        if (progress.wasCanceled()) break;

        QVector<double> yData = tempSpectra[i];
        QVector<double> xData = m_mappingWave;

        // 数据有效性检查
        if (yData.size() != xData.size() || yData.size() < 3) {
            qWarning() << "Invalid data at index" << i << ", skipping";
            continue;
        }

        // 直接调用本地平滑函数
        yData = performSmoothing(yData);

        // 修正835.932点
        for (int j = 0; j < xData.size(); ++j) {
            if (j > 0 && std::abs(xData[j] - 835.932) < 0.1) {
                yData[j] = yData[j - 1];
            }
        }

        tempSpectra[i] = yData;
        progress.setValue(i + 1);
        QCoreApplication::processEvents();
    }

    if (!progress.wasCanceled()) {
        // 处理完成后再替换原始数据
        m_mappingSpectra = tempSpectra;

        // 延迟绘图，确保UI线程安全
        QTimer::singleShot(100, this, [this]() {
            plotMappingSpectra();
        });
    }
}

void MainWindow::on_baseline_mapping_clicked()
{
    if (m_mappingSpectra.isEmpty()) {
        QMessageBox::information(this, "提示", "没有Mapping数据可供处理");
        return;
    }

    QProgressDialog progress("批量去基线中...", "取消", 0, m_mappingSpectra.size(), this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0);

    QVector<QVector<double>> tempSpectra = m_mappingSpectra;

    for (int i = 0; i < tempSpectra.size(); ++i) {
        if (progress.wasCanceled()) break;

        QVector<double> yData = tempSpectra[i];
        QVector<double> xData = m_mappingWave;

        if (yData.size() != xData.size() || yData.size() < 3) {
            qWarning() << "Invalid data at index" << i << ", skipping";
            continue;
        }

        // 直接调用本地去基线函数
        yData = performBaselineCorrection(yData);

        // 修正835.932点
        for (int j = 0; j < xData.size(); ++j) {
            if (j > 0 && std::abs(xData[j] - 835.932) < 0.1) {
                yData[j] = yData[j - 1];
            }
        }

        tempSpectra[i] = yData;
        progress.setValue(i + 1);
        QCoreApplication::processEvents();
    }

    if (!progress.wasCanceled()) {
        m_mappingSpectra = tempSpectra;

        QTimer::singleShot(100, this, [this]() {
            plotMappingSpectra();
        });
    }
}
void MainWindow::saveAFProcessData(int position, double score, const QVector<double>& spectrum)
{
    if (m_afLogFileName.isEmpty()) {
        // 如果文件名为空（比如刚开始对焦），创建一个以时间命名的文件
        QDir dir("/home/Datas");
        if (!dir.exists()) dir.mkpath(".");
        m_afLogFileName = QString("/home/Datas/AF_Log_%1.csv")
                          .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    }

    QFile file(m_afLogFileName);
    if (file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        QTextStream out(&file);
        // 如果是新文件，写入表头
        if (file.size() == 0) {
            out << "Step,Z_Position,Score,Raw_Data\n";
        }

        // 写入当前步骤、位置、分数
        static int stepCount = 0;
        out << stepCount++ << "," << position << "," << score << ",";

        // 写入光谱数据（用分号分隔，方便在Excel中展开）
        for(int i = 0; i < spectrum.size(); ++i) {
            out << spectrum[i] << (i == spectrum.size() - 1 ? "" : ";");
        }
        out << "\n";
        file.close();
    }
}
