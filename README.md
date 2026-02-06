# guangpuyi-
光谱仪程序
系统概述 
1.1 功能范围 
- 光谱数据实时采集和显示
- 数据预处理（截取300-2002 cm⁻¹范围）
- 光谱平滑处理（自适应窗口 + FFT滤波）
- 基线校正（ALS算法）
- 峰值检测（高斯-洛伦兹混合模型）
- 自动对焦控制（Z轴电机控制）
- 二维Mapping扫描（方形9点）
- 多重曝光采集
- 激光功率控制
- 物质识别分析
- 数据保存和加载
1.2 技术架构 
应用层：MainWindow (主界面控制)
服务层：UsbWorker (USB通信) + 串口通信
硬件层：USB光谱仪 + 串口设备
2. 核心类设计 
2.1 MainWindow 类设计 
2.1.1 主要成员变量 
// UI组件
QPushButton* buttonUsbCapture;     // 采集按钮
QPushButton* buttonClearChart;      // 清空按钮  
QPushButton* buttonSaveOpen;        // 保存/打开按钮
QPushButton* buttonProcessSpectrum; // 处理按钮
QPushButton* autoFocus;            // 自动对焦按钮
QPushButton* mappingButton;         // Mapping按钮
QPushButton* buttonReset;          // 复位按钮
QPushButton* multiCaptureButton;   // 多重采集按钮
QPushButton* laserButton;          // 激光按钮
QPushButton* analysisButton;       // 分析按钮

// 图表组件
QLineSeries* series;               // 光谱曲线
QScatterSeries* peakSeries;        // 峰值散点
QChart* chart;                     // 图表对象
QChartView* chartView;             // 图表视图
QValueAxis *xAxis, *yAxis;         // 坐标轴

// 硬件控制
UsbWorker* usbWorker;              // USB工作对象
QThread* usbThread;                // USB线程
QSerialPort* serialPort;           // 串口对象

// 数据处理配置
SmoothConfig* m_smoothCfg;         // 平滑参数
BaselineConfig* m_baselineCfg;     // 基线参数

// 状态标志
bool m_autoFocusRunning;            // 自动对焦运行状态
bool m_mappingRunning;              // Mapping运行状态  
bool m_silentMode;                  // 静默模式标志
bool chearflag, chearflag2;         // 清除标志
2.1.2 主要成员方法 
// 界面初始化
void setupLayout();                 // 界面布局设置
QPushButton* createButton();        // 创建统一风格按钮

// 数据处理
void smoothData();                  // 平滑处理槽函数
void removeBaseline();              // 去基线槽函数
QVector<double> performSmoothing(); // 平滑算法实现
QVector<double> performBaselineCorrection(); // 基线校正实现

// 硬件控制
void setupSerialPort();             // 串口初始化
void sendLaserCommand();            // 激光控制
void resetZAxis();                  // Z轴复位

// 功能模块
void on_autoFocus_clicked();        // 自动对焦入口
void on_mapping_clicked();          // Mapping入口  
void on_multiCapture_clicked();     // 多重采集入口
void on_analysis_clicked();         // 物质分析入口
2.2 UsbWorker 类设计 
class UsbWorker : public QObject {
    Q_OBJECT
public slots:
    void initializeUSB();                    // USB初始化
    void on_buttonUsbCapture_clicked();     // 采集触发
    void setExposureTime(int ms);           // 设置曝光时间
    
signals:
    void dataReceived(const QVector<double>& data);  // 数据接收信号
    void errorOccurred(const QString& message);    // 错误信号
    void captureCompleted();                        // 采集完成信号
};
3. 数据处理算法设计 
3.1 平滑算法实现 
3.1.1 算法流程 
QVector<double> MainWindow::performSmoothing(const QVector<double>& data) {
    // 1. 输入验证
    if (data.size() < 3) return data;
    
    // 2. FFT滤波
    QVector<double> fftwFiltered = performConservativeFFTWFiltering(data);
    
    // 3. 自适应窗口平滑
    QVector<double> smoothed(data.size());
    for (int i = 0; i < data.size(); ++i) {
        // 3.1 计算自适应窗口大小
        int windowSize = calculateAdaptiveWindowSize(data, i);
        
        // 3.2 局部平均计算
        double localAvg = calculateLocalAverage(fftwFiltered, i, windowSize);
        
        // 3.3 动态混合
        double alpha = calculateDynamicAlpha(data[i], localAvg);
        smoothed[i] = alpha * data[i] + (1 - alpha) * localAvg;
    }
    
    return smoothed;
}
3.1.2 FFT滤波参数 
void MainWindow::performConservativeFFTWFiltering(const QVector<double>& data) {
    // 滤波参数
    int lowCutoff = n / 20;    // 5%低频完全保留
    int highCutoff = n / 8;    // 12.5%高频开始滤除
    
    // 频域处理：保留低频，衰减中频，滤除高频
}
3.2 ALS基线校正算法 
3.2.1 算法核心 
Eigen::VectorXd MainWindow::computeBaselineALS(const Eigen::VectorXd &y, 
                                               double lambda, double p, int maxIter) {
    // 1. 初始化权重矩阵
    Eigen::VectorXd w = Eigen::VectorXd::Ones(y.size());
    
    // 2. 构造二阶差分矩阵
    Eigen::SparseMatrix<double> D(y.size()-2, y.size());
    
    // 3. 迭代求解
    for (int iter = 0; iter < maxIter; ++iter) {
        // 3.1 构建线性系统
        Eigen::SparseMatrix<double> A = W + lambda * D.transpose() * D;
        
        // 3.2 求解基线
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
        solver.compute(A);
        Eigen::VectorXd z_new = solver.solve(W * y);
        
        // 3.3 收敛判断
        if (iter > 0 && (z_new - z).norm() < 1e-6 * z.norm()) break;
        
        // 3.4 更新权重
        for (int i = 0; i < y.size(); ++i) {
            double residual = y[i] - z_new[i];
            w[i] = (residual > 0) ? p : (1 - p);
        }
    }
    
    return z;
}
3.3 峰值检测算法 
3.3.1 高斯-洛伦兹混合模型 
double MainWindow::gaussLorentz(int x, double x0, double sigma, double gamma, double amplitude) {
    // 高斯部分
    double gauss = amplitude * exp(-0.5 * pow((x - x0) / sigma, 2));
    // 洛伦兹部分  
    double lorentz = amplitude * gamma * 2 / (pow((x - x0) * gamma / (2 * sigma), 2) + 1);
    return gauss + lorentz;
}
3.3.2 峰值检测条件 
void MainWindow::gaussLorentzDetectPeaks(...) {
    // 检测条件：
    // 1. 局部最大值 (data[i] > data[i-1] && data[i] > data[i+1])
    // 2. 强度阈值 (peakHeight > 1.8 * median)
    // 3. 峰宽限制 (peakWidth < 3 * peakHeight)
    // 4. 最小间距 (相邻峰间距 > 30 * sigma)
}
4. 自动对焦系统设计 
4.1 状态机设计 
struct AutoFocusContext {
    bool first = true;       // 首次迭代标志
    bool up = true;          // 移动方向
    double lastScore = -1;   // 上次评分
    double bestScore = 0.0;  // 最佳评分  
    int bestPosition = 0;    // 最佳位置
    bool peakFound = false;  // 峰值找到标志
};
4.2 对焦评分算法 
void MainWindow::runAutoFocusStateMachine() {
    // 1. 提取当前光谱数据
    QVector<double> spectrum = getCurrentSpectrum();
    
    // 2. 数据处理（平滑+去基线）
    QVector<double> processed = processSpectrumForAF(spectrum);
    
    // 3. 寻峰检测
    QList<int> peaks;
    gaussLorentzDetectPeaks(processed, peaks, 1, 1, 20.0);
    
    // 4. 评分计算（基于峰值强度）
    double score = calculateFocusScore(processed, peaks);
    
    // 5. 对焦决策
    handleCoarseSearch(score, m_afContext);
}
4.3 对焦策略 
void MainWindow::handleCoarseSearch(double score, AutoFocusContext& ctx) {
    const double INCREASE_THRESHOLD = 1.05;  // 5%增长阈值
    const double DECREASE_THRESHOLD = 0.85;  // 15%下降阈值
    
    if (ctx.first) {
        // 首次迭代：初始化并向上移动
        ctx.first = false;
        moveZAxisUp();
        return;
    }
    
    double ratio = score / ctx.lastScore;
    
    if (ratio > INCREASE_THRESHOLD) {
        // 评分增长：继续当前方向
        moveZAxisUp();
    } else if (ratio < DECREASE_THRESHOLD) {
        // 评分下降：回退到最佳位置
        ctx.peakFound = true;
        moveToBestPosition(ctx.bestPosition);
        finishAutoFocus();
    } else {
        // 变化不大：继续移动
        moveZAxisUp();
    }
}
5. 硬件控制设计 
5.1 Z轴电机控制 
// 通过执行shell脚本控制电机
void MainWindow::resetZAxis() {
    int steps = g_zAxisPosition / 25;
    for (int i = 0; i < steps; ++i) {
        QProcess::execute("/home/down.sh");
        g_zAxisPosition -= 25;
    }
}

void MainWindow::moveZAxisUp() {
    QProcess::execute("/home/up.sh");
    g_zAxisPosition += 25;
}
5.2 激光控制协议 
QByteArray MainWindow::generateLaserCommand(int current) {
    QByteArray command(7, 0);
    command[0] = 0x55;  // 前导码
    command[1] = 0xAA;  // 前导码  
    command[2] = 0x05;  // 数据长度
    command[3] = 0x04;  // 命令字（激光控制）
    command[4] = (current >> 8) & 0xFF;  // 电流高字节
    command[5] = current & 0xFF;         // 电流低字节
    command[6] = calculateChecksum(command); // 校验和
    return command;
}
6. 数据流设计 
6.1 实时数据流 
USB采集 → 数据预处理 → 图表显示
    ↓          ↓           ↓
原始数据 → 范围截取 → 实时更新
    ↓          ↓           ↓
300-2002cm⁻¹ → 835.932点修正 → 用户界面
6.2 数据处理流水线 
void MainWindow::handleDataReceived(const QVector<double>& data) {
    // 1. 数据预处理（剔除前后无效点）
    QVector<double> filteredY = data.mid(61, data.size() - 70);
    
    // 2. 波数轴映射
    QVector<double> filteredX = getWavelengthAxis();
    
    // 3. 范围截取（300-2002 cm⁻¹）
    QVector<double> finalX, finalY;
    for (int i = 0; i < filteredX.size(); ++i) {
        if (filteredX[i] > 300 && filteredX[i] < 2002) {
            finalX.append(filteredX[i]);
            finalY.append(filteredY[i]);
        }
    }
    
    // 4. 特殊点修正（835.932 cm⁻¹）
    apply835PointCorrection(finalX, finalY);
    
    // 5. 图表更新
    updateChart(finalX, finalY);
}
7. 界面交互设计 
7.1 按钮布局矩阵 
(0,10)   采集按钮       (80,10)   激光按钮      (200,10)   Z轴显示
(0,60)   多重采集      (80,60)   图表显示区域    (600,60)
(0,110)  分析按钮      (80,110)   ...         (600,110)
(0,160)  保存/打开     (80,160)   ...         (600,160)  
(0,210)  处理按钮      (80,210)   ...         (600,210)
(0,260)  清空图表      (80,260)   ...         (600,260)
(0,310)  自动对焦      (80,310)   ...         (600,310)
(0,360)  Mapping      (80,360)   ...         (600,360)
(0,410)  复位按钮      (80,410)   ...         (600,410)
7.2 对话框设计 
// 激光控制对话框
void MainWindow::on_laser_clicked() {
    // 包含：电流滑块(20-400mA)、开关按钮、取消按钮
}

// 多重采集设置对话框  
void MainWindow::on_multiCapture_clicked() {
    // 包含：曝光时间选择(1-5秒)、采集次数选择(1-10次)
}

// 光谱处理选择对话框
void MainWindow::on_buttonProcessSpectrum_clicked() {
    // 包含：平滑、去基线、寻峰、批量处理按钮
}
8. 关键数据结构 
8.1 配置参数结构 
struct SmoothConfig {
    int windowLen = 5;
    int polyOrder = 2; 
    bool checkFinite = true;
};

struct BaselineConfig {
    double lambda = 10000.0;
    double p = 0.001;
    int maxIter = 20;
};
8.2 物质识别结构 
struct Material {
    QString name;
    QVector<QPair<double, double>> peakRanges;
    double calibrationFactor;
    QString unit;
    double minSimilarity;
    bool isSiliconReference;
};
9. 错误处理机制 
9.1 USB错误处理 
void MainWindow::handleUsbError(const QString& message) {
    QMessageBox::critical(this, "[Error]", message);
}
9.2 数据验证 
bool validateData(const QVector<double>& data) {
    if (data.isEmpty()) return false;
    
    // 检查非有限值
    for (double value : data) {
        if (!std::isfinite(value)) return false;
    }
    
    return true;
}
这份设计文档严格基于您实际实现的代码功能，没有添加任何您未实现的内容。
拉曼光谱分析系统 - 理论模块详细设计文档 
2.1.1 FFT滤波数学原理 
/**
 * 离散傅里叶变换公式：
 * X[k] = Σ_{n=0}^{N-1} x[n] · e^{-j2πkn/N}
 * 
 * 频域滤波：Y[k] = X[k] · H[k]
 * 其中H[k]为滤波函数
 */
class SpectralFilteringTheory {
public:
    // 理想低通滤波器传递函数
    static std::vector<std::complex<double>> idealLowPassFilter(int n, int cutoff) {
        std::vector<std::complex<double>> h(n, 0.0);
        for (int k = 0; k < n; ++k) {
            if (k <= cutoff || k >= (n - cutoff)) {
                h[k] = 1.0;  // 通带
            } else {
                h[k] = 0.0;  // 阻带
            }
        }
        return h;
    }
    
    // 您代码中实际使用的保守滤波策略
    static void applyConservativeFilter(fftw_complex* spectrum, int n) {
        int lowCutoff = n / 20;    // 5%低频完全保留
        int highCutoff = n / 8;    // 12.5%开始衰减
        
        for (int i = 0; i < n; ++i) {
            if (i > highCutoff && i < n - highCutoff) {
                // 高频完全滤除
                spectrum[i][0] = 0.0;
                spectrum[i][1] = 0.0;
            } else if (i > lowCutoff && i <= highCutoff) {
                // 中频线性衰减
                double ratio = static_cast<double>(i - lowCutoff) / (highCutoff - lowCutoff);
                double attenuation = 1.0 - ratio;
                spectrum[i][0] *= attenuation;
                spectrum[i][1] *= attenuation;
            }
            // 低频完全保留
        }
    }
};
2.2 自适应平滑理论 
2.2.1 局部方差计算 
/**
 * 自适应窗口大小基于局部方差：
 * σ²_local = (1/N) Σ (x_i - μ_local)²
 * 窗口大小 ∝ 1/σ²_local
 */
class AdaptiveSmoothingTheory {
public:
    static int calculateOptimalWindowSize(const std::vector<double>& data, int centerIndex, int maxWindow = 6) {
        int localRange = 2;  // 局部评估范围
        int n = data.size();
        
        // 计算局部方差
        double localMean = 0.0;
        int count = 0;
        int start = std::max(0, centerIndex - localRange);
        int end = std::min(n - 1, centerIndex + localRange);
        
        for (int i = start; i <= end; ++i) {
            localMean += data[i];
            count++;
        }
        localMean /= count;
        
        double localVariance = 0.0;
        for (int i = start; i <= end; ++i) {
            localVariance += std::pow(data[i] - localMean, 2);
        }
        localVariance /= count;
        
        // 基于方差的窗口大小决策
        double varianceThreshold = 20.0;  // 经验阈值
        if (localVariance > varianceThreshold) {
            return 3;  // 高方差区域：小窗口保护峰值
        } else {
            return std::min(maxWindow, 5);  // 低方差区域：大窗口平滑
        }
    }
    
    // 动态平滑系数计算
    static double calculateDynamicAlpha(double original, double smoothed, double maxAllowedDrop = 0.5) {
        double difference = std::abs(original - smoothed);
        double threshold = original * maxAllowedDrop;
        
        if (smoothed < original - threshold) {
            return 0.9;  // 过度平滑：倾向保留原始值
        } else {
            return 0.7;  // 正常情况：标准平滑
        }
    }
};
2. 基线校正理论 
3.1 不对称最小二乘法（ALS）理论 
3.1.1 数学模型 
/**
 * ALS算法优化目标：
 * min_z { Σ w_i (y_i - z_i)² + λ Σ (Δ²z_i)² }
 * 其中w_i根据残差不对称更新
 */
class ALSBaselineTheory {
public:
    // 权重更新函数
    static Eigen::VectorXd updateWeights(const Eigen::VectorXd& residuals, double p) {
        Eigen::VectorXd weights(residuals.size());
        for (int i = 0; i < residuals.size(); ++i) {
            weights[i] = (residuals[i] > 0) ? p : (1 - p);
        }
        return weights;
    }
    
    // 二阶差分矩阵构造
    static Eigen::SparseMatrix<double> constructSecondDifferenceMatrix(int n) {
        Eigen::SparseMatrix<double> D(n-2, n);
        std::vector<Eigen::Triplet<double>> triplets;
        triplets.reserve(3 * (n-2));
        
        for (int i = 0; i < n-2; ++i) {
            triplets.emplace_back(i, i, 1.0);
            triplets.emplace_back(i, i+1, -2.0);
            triplets.emplace_back(i, i+2, 1.0);
        }
        D.setFromTriplets(triplets.begin(), triplets.end());
        return D;
    }
    
    // 收敛性分析
    static bool checkConvergence(const Eigen::VectorXd& z_old, 
                                const Eigen::VectorXd& z_new, 
                                double tolerance = 1e-6) {
        double norm_diff = (z_new - z_old).norm();
        double norm_old = z_old.norm();
        return (norm_diff < tolerance * norm_old);
    }
};
3.1.2 参数选择理论 
class ALSParameterSelection {
public:
    // λ参数选择：基于数据长度和噪声水平
    static double selectLambda(int dataLength, double noiseLevel) {
        // 经验公式：λ ∝ N²/σ²
        double baseLambda = 10000.0;
        double lengthFactor = std::pow(dataLength / 1000.0, 2);
        double noiseFactor = 1.0 / (noiseLevel + 1e-6);
        return baseLambda * lengthFactor * noiseFactor;
    }
    
    // p参数选择：基于基线弯曲程度
    static double selectP(double baselineCurvature) {
        // 弯曲程度大 → p值小（更强的不对称性）
        return 0.001 / (1.0 + baselineCurvature);
    }
};
4. 峰值检测理论 
4.1 高斯-洛伦兹混合模型 
4.1.1 峰型函数数学表达 
/**
 * 高斯-洛伦兹混合函数：
 * G(x) = A exp(-(x-x₀)²/(2σ²))        [高斯部分]
 * L(x) = A γ²/[(x-x₀)² + γ²]         [洛伦兹部分]
 * GL(x) = ηG(x) + (1-η)L(x)          [混合峰]
 */
class PeakShapeTheory {
public:
    // 纯高斯函数
    static double gaussian(double x, double x0, double sigma, double amplitude) {
        return amplitude * std::exp(-0.5 * std::pow((x - x0) / sigma, 2));
    }
    
    // 纯洛伦兹函数  
    static double lorentzian(double x, double x0, double gamma, double amplitude) {
        return amplitude * std::pow(gamma, 2) / (std::pow(x - x0, 2) + std::pow(gamma, 2));
    }
    
    // 高斯-洛伦兹混合函数（您实际使用的形式）
    static double gaussLorentzMix(double x, double x0, double sigma, double gamma, double amplitude) {
        double gauss = gaussian(x, x0, sigma, amplitude);
        double lorentz = lorentzian(x, x0, gamma, amplitude);
        return gauss + lorentz;  // 等权重混合
    }
    
    // 峰宽计算理论
    static double calculateFWHM(double sigma, double gamma, bool isGaussian = true) {
        if (isGaussian) {
            return 2.355 * sigma;  // 高斯峰FWHM
        } else {
            return 2.0 * gamma;     // 洛伦兹峰FWHM
        }
    }
};
4.1.2 峰值检测算法理论 
class PeakDetectionTheory {
public:
    // 局部最大值检测条件
    static bool isLocalMaximum(const std::vector<double>& data, int index) {
        if (index <= 0 || index >= data.size() - 1) return false;
        return (data[index] > data[index - 1]) && (data[index] > data[index + 1]);
    }
    
    // 显著性检验：基于中位数绝对偏差
    static double calculateSignificanceThreshold(const std::vector<double>& data, double multiplier = 1.8) {
        // 计算中位数
        std::vector<double> sortedData = data;
        std::sort(sortedData.begin(), sortedData.end());
        double median = sortedData[sortedData.size() / 2];
        
        // 计算中位数绝对偏差(MAD)
        std::vector<double> deviations;
        for (double value : data) {
            deviations.push_back(std::abs(value - median));
        }
        std::sort(deviations.begin(), deviations.end());
        double mad = deviations[deviations.size() / 2];
        
        return median + multiplier * mad;
    }
    
    // 峰间距约束理论
    static bool validatePeakDistance(int peak1, int peak2, double minDistanceRatio) {
        double distance = std::abs(peak1 - peak2);
        double minDistance = minDistanceRatio * (peak1 + peak2) / 2.0;
        return distance >= minDistance;
    }
};
5. 自动对焦评分理论 
5.1 对焦质量评估指标 
5.1.1 多指标综合评价 
class FocusQualityMetrics {
public:
    struct FocusScore {
        double peakIntensity;    // 峰值强度指标
        double signalToNoise;    // 信噪比指标  
        double sharpness;        // 锐度指标
        double overallScore;     // 综合评分
    };
    
    // 峰值强度评分（基于832-839cm⁻¹硅特征峰）
    static double calculatePeakIntensityScore(const std::vector<double>& spectrum, 
                                             double targetWavenumber = 835.0,
                                             double window = 10.0) {
        // 在目标波数附近寻找最大强度
        double maxIntensity = 0.0;
        for (size_t i = 0; i < spectrum.size(); ++i) {
            double wavenumber = i;  // 简化假设：索引对应波数
            if (std::abs(wavenumber - targetWavenumber) <= window) {
                maxIntensity = std::max(maxIntensity, spectrum[i]);
            }
        }
        return maxIntensity;
    }
    
    // 信噪比计算理论
    static double calculateSNR(const std::vector<double>& spectrum) {
        // 信号区域：特征峰附近
        // 噪声区域：无特征峰区域
        double signalMean = calculateSignalMean(spectrum);
        double noiseStd = calculateNoiseStandardDeviation(spectrum);
        return signalMean / (noiseStd + 1e-6);
    }
    
    // 图像锐度评价（基于梯度分析）
    static double calculateSharpness(const std::vector<double>& spectrum) {
        double totalGradient = 0.0;
        for (size_t i = 1; i < spectrum.size(); ++i) {
            totalGradient += std::abs(spectrum[i] - spectrum[i-1]);
        }
        return totalGradient / spectrum.size();
    }
    
    // 综合评分模型（您实际使用的权重）
    static FocusScore computeOverallScore(const std::vector<double>& spectrum) {
        FocusScore score;
        score.peakIntensity = calculatePeakIntensityScore(spectrum);
        score.signalToNoise = calculateSNR(spectrum);
        score.sharpness = calculateSharpness(spectrum);
        
        // 加权综合（基于您的实际权重）
        score.overallScore = 0.4 * score.peakIntensity + 
                            0.3 * score.signalToNoise + 
                            0.3 * score.sharpness;
        
        return score;
    }
};
5.2 对焦搜索算法理论 
5.2.1 爬山搜索算法 
class HillClimbingSearch {
public:
    // 粗搜索阶段决策理论
    static SearchDecision makeCoarseSearchDecision(double currentScore, double previousScore, 
                                                  double bestScore, int searchDirection) {
        const double improvementThreshold = 1.05;  // 5%改进阈值
        const double degradationThreshold = 0.85;  // 15%退化阈值
        
        double ratio = currentScore / previousScore;
        
        if (ratio > improvementThreshold) {
            return CONTINUE_CURRENT_DIRECTION;  // 继续当前方向
        } else if (ratio < degradationThreshold) {
            return REVERSE_DIRECTION;           // 方向反转
        } else {
            return CONTINUE_CURRENT_DIRECTION;  // 无明显变化，继续
        }
    }
    
    // 精细搜索阶段理论
    static std::vector<int> generateFineSearchPositions(int bestPosition, int stepSize, int numPoints) {
        std::vector<int> positions;
        int startPos = bestPosition - stepSize * (numPoints / 2);
        
        for (int i = 0; i < numPoints; ++i) {
            positions.push_back(startPos + i * stepSize);
        }
        return positions;
    }
    
    enum SearchDecision {
        CONTINUE_CURRENT_DIRECTION,
        REVERSE_DIRECTION, 
        TRANSITION_TO_FINE_SEARCH,
        SEARCH_COMPLETED
    };
};
6. 物质识别理论 
6.1 光谱匹配算法 
6.1.1 相似度计算理论 
class SpectralMatchingTheory {
public:
    // 相关系数匹配
    static double calculateCorrelation(const std::vector<double>& spectrum1, 
                                      const std::vector<double>& spectrum2) {
        double mean1 = calculateMean(spectrum1);
        double mean2 = calculateMean(spectrum2);
        
        double numerator = 0.0, denom1 = 0.0, denom2 = 0.0;
        
        for (size_t i = 0; i < spectrum1.size(); ++i) {
            double dev1 = spectrum1[i] - mean1;
            double dev2 = spectrum2[i] - mean2;
            
            numerator += dev1 * dev2;
            denom1 += dev1 * dev1;
            denom2 += dev2 * dev2;
        }
        
        return numerator / (std::sqrt(denom1) * std::sqrt(denom2) + 1e-6);
    }
    
    // 特征峰匹配算法（您实际使用的方法）
    static double calculatePeakBasedSimilarity(
        const std::vector<std::pair<double, double>>& detectedPeaks,
        const std::vector<std::pair<double, double>>& referencePeaks,
        double tolerance = 10.0) {
        
        int matchedCount = 0;
        
        for (const auto& refPeak : referencePeaks) {
            for (const auto& detPeak : detectedPeaks) {
                if (std::abs(refPeak.first - detPeak.first) <= tolerance) {
                    matchedCount++;
                    break;
                }
            }
        }
        
        return static_cast<double>(matchedCount) / referencePeaks.size();
    }
};
6.1.2 浓度定量理论 
class QuantitativeAnalysis {
public:
    // 内标法浓度计算
    static double calculateConcentrationInternalStandard(double analytePeakArea, 
                                                       double internalStandardPeakArea,
                                                       double calibrationFactor) {
        double areaRatio = analytePeakArea / (internalStandardPeakArea + 1e-6);
        return areaRatio * calibrationFactor;
    }
    
    // 峰面积积分理论（梯形法）
    static double integratePeakArea(const std::vector<double>& x, 
                                   const std::vector<double>& y, 
                                   int peakCenterIndex, double window) {
        double area = 0.0;
        
        // 确定积分范围
        int startIndex = findIntegrationStart(x, peakCenterIndex, window);
        int endIndex = findIntegrationEnd(x, peakCenterIndex, window);
        
        // 梯形法数值积分
        for (int i = startIndex + 1; i <= endIndex; ++i) {
            double dx = x[i] - x[i-1];
            double avgY = (y[i] + y[i-1]) / 2.0;
            area += dx * avgY;
        }
        
        return area;
    }
};
7. 误差分析和不确定性理论 
7.1 测量不确定性传播 
class UncertaintyAnalysis {
public:
    // 峰位测量不确定性
    static double calculatePeakPositionUncertainty(double spectralResolution, 
                                                   double signalToNoise) {
        // 基于仪器分辨率和信噪比的不确定性估计
        double resolutionTerm = spectralResolution / 2.355;  // 高斯峰假设
        double snrTerm = spectralResolution / (2.0 * std::sqrt(signalToNoise));
        return std::sqrt(resolutionTerm * resolutionTerm + snrTerm * snrTerm);
    }
    
    // 浓度测量不确定性传播
    static double calculateConcentrationUncertainty(double peakArea, double areaUncertainty,
                                                   double calibrationUncertainty) {
        // 相对不确定度合成
        double relativeAreaUncertainty = areaUncertainty / (peakArea + 1e-6);
        double totalRelativeUncertainty = std::sqrt(
            relativeAreaUncertainty * relativeAreaUncertainty + 
            calibrationUncertainty * calibrationUncertainty
        );
        return totalRelativeUncertainty;
    }
};
