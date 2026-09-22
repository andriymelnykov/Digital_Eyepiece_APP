#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>
#include <cmath>
#include <limits>
#include <algorithm>

#include <iostream>

struct SigmaClippedStats
{
    double mean = std::numeric_limits<double>::quiet_NaN();
    double stddev = std::numeric_limits<double>::quiet_NaN();
    int count = 0;
};

static void computeMeanStddev(const std::vector<int>& vals, double& mean, double& stddev)
{
    if (vals.empty()) {
        mean = stddev = std::numeric_limits<double>::quiet_NaN();
        return;
    }

    double sum = 0.0;
    for (int v : vals) sum += v;
    mean = sum / vals.size();

    double sq = 0.0;
    for (int v : vals) {
        double d = v - mean;
        sq += d * d;
    }

    stddev = std::sqrt(sq / vals.size());   // population stddev
    // use (vals.size() - 1) instead if you want sample stddev

    //std::cout << mean << " " << stddev << std::endl;
}

SigmaClippedStats sigmaClippedMeanStddev(
    const cv::Mat& img,
    double sigma = 3.0,
    int maxIters = 10)
{
    CV_Assert(img.type() == CV_32SC1);
    CV_Assert(!img.empty());
    CV_Assert(sigma > 0.0);

    std::vector<int> vals;
    vals.reserve(img.total());

    if (img.isContinuous()) {
        const int* p = img.ptr<int>(0);
        vals.assign(p, p + img.total());
    }
    else {
        for (int r = 0; r < img.rows; ++r) {
            const int* row = img.ptr<int>(r);
            vals.insert(vals.end(), row, row + img.cols);
        }
    }

    double mean = 0.0, stddev = 0.0;

    for (int iter = 0; iter < maxIters; ++iter) {
        computeMeanStddev(vals, mean, stddev);
        if (vals.empty() || stddev == 0.0)
            break;

        double lo = mean - sigma * stddev;
        double hi = mean + sigma * stddev;

        std::vector<int> clipped;
        clipped.reserve(vals.size());

        for (int v : vals) {
            if (v >= lo && v <= hi)
                clipped.push_back(v);
        }

        if (clipped.size() == vals.size()) {
            // converged: nothing rejected
            vals.swap(clipped);
            break;
        }

        if (clipped.empty()) {
            vals.clear();
            mean = stddev = std::numeric_limits<double>::quiet_NaN();
            break;
        }

        vals.swap(clipped);
    }

    computeMeanStddev(vals, mean, stddev);

    SigmaClippedStats out;
    out.mean = mean;
    out.stddev = stddev;
    out.count = static_cast<int>(vals.size());
    return out;
}





struct MedianMadStats
{
    double median = std::numeric_limits<double>::quiet_NaN();
    double mad = std::numeric_limits<double>::quiet_NaN();          // raw MAD
    double sigmaMad = std::numeric_limits<double>::quiet_NaN();     // 1.4826 * MAD
    int count = 0;
};

static double medianOfVector(std::vector<double> vals)
{
    if (vals.empty())
        return std::numeric_limits<double>::quiet_NaN();

    const size_t n = vals.size();
    const size_t mid = n / 2;

    std::nth_element(vals.begin(), vals.begin() + mid, vals.end());
    double med = vals[mid];

    if ((n % 2) == 0) {
        std::nth_element(vals.begin(), vals.begin() + mid - 1, vals.end());
        med = 0.5 * (vals[mid - 1] + vals[mid]);
    }

    return med;
}

MedianMadStats medianMadStats(const cv::Mat& img)
{
    CV_Assert(img.type() == CV_32SC1);
    CV_Assert(!img.empty());

    std::vector<double> vals;
    vals.reserve(img.total());

    if (img.isContinuous()) {
        const int* p = img.ptr<int>(0);
        for (size_t i = 0; i < img.total(); ++i)
            vals.push_back(static_cast<double>(p[i]));
    }
    else {
        for (int r = 0; r < img.rows; ++r) {
            const int* row = img.ptr<int>(r);
            for (int c = 0; c < img.cols; ++c)
                vals.push_back(static_cast<double>(row[c]));
        }
    }

    MedianMadStats out;
    out.count = static_cast<int>(vals.size());

    out.median = medianOfVector(vals);

    std::vector<double> absDev;
    absDev.reserve(vals.size());
    for (double v : vals)
        absDev.push_back(std::abs(v - out.median));

    out.mad = medianOfVector(absDev);
    out.sigmaMad = 1.4826 * out.mad;   // normal-consistent robust sigma estimate

    return out;
}


float medianCenterHalfCrop(const cv::Mat& img)
{
    CV_Assert(img.type() == CV_32FC1);

    cv::Mat crop = img(cv::Rect(
        img.cols / 4,
        img.rows / 4,
        img.cols / 2,
        img.rows / 2
    ));

    std::vector<double> vals;
    vals.reserve(img.total());

    if (img.isContinuous()) {
        const float* p = img.ptr<float>(0);
        for (size_t i = 0; i < img.total(); ++i)
            vals.push_back(static_cast<double>(p[i]));
    }
    else {
        for (int r = 0; r < img.rows; ++r) {
            const float* row = img.ptr<float>(r);
            for (int c = 0; c < img.cols; ++c)
                vals.push_back(static_cast<double>(row[c]));
        }
    }

    float median = medianOfVector(vals);
    return median;
}