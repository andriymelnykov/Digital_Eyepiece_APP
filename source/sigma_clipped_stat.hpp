#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>
#include <cmath>
#include <limits>

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