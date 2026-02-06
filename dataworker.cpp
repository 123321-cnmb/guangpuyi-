#include "dataworker.h"


// 平滑的辅助函数
static int nextPowerOfTwo(int x) {
    x--;
    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;
    x++;
    return x;
}
static bool containsNonFinite(const QVector<double> &v)
{
    return std::any_of(v.constBegin(), v.constEnd(),[](double x) { return !std::isfinite(x); });
}

DataWorker::DataWorker(QObject *parent) : QObject(parent)
{
    m_smoothCfg   = new SmoothConfig;
    m_baselineCfg = new BaselineConfig;
}

DataWorker::~DataWorker()
{
    delete m_smoothCfg;
    delete m_baselineCfg;
}


// 平滑接口
void DataWorker::smoothData(const QVector<double>& y, const QVector<double>& x)
{

    if (y.size() < 3) {
        emit dataProcessed(y, x);   // 数据太短，原样返回
        return;
    }
    QVector<double> sm = performSmoothing(y);
    emit dataProcessed(sm, x);
}

QVector<double> DataWorker::performSmoothing(const QVector<double>& data) {
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

int DataWorker::calculateAdaptiveWindowSize(const QVector<double>& data, int index) {
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
QVector<double> DataWorker::performConservativeFFTWFiltering(const QVector<double>& data) {
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


// 更新原有的removeBaseline方法，改为使用多项式拟合
void DataWorker::removeBaseline(const QVector<double> &data, const QVector<double>& x)
{
    QMutexLocker lock(&m_mutex);
    QElapsedTimer t;
    t.start();
    if (data.isEmpty()) {
        emit errorOccurred("Empty data for baseline removal.");
        return;
    }
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

QVector<double> DataWorker::ensureNonZeroResult(const QVector<double>& original, const QVector<double>& corrected)
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

QVector<double> DataWorker::performBaselineCorrection(const QVector<double> &data)
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

Eigen::VectorXd DataWorker::computeBaselineALS(const Eigen::VectorXd &y, double lambda, double p, int maxIter)
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


SmoothConfig DataWorker::smoothConfig() const {
    QMutexLocker l(&m_mutex);
    return *m_smoothCfg;
}
void DataWorker::setBaselineConfig(const BaselineConfig &cfg) {
    QMutexLocker l(&m_mutex);
    *m_baselineCfg = cfg;
}
BaselineConfig DataWorker::baselineConfig() const {
    QMutexLocker l(&m_mutex);
    return *m_baselineCfg;
}
