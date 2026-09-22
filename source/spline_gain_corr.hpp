#include <opencv2/core.hpp>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace RadialSplineCorrection
{
    struct Spline1D
    {
        std::vector<float> x; // knot radii in [0..1]
        std::vector<float> y; // amplitudes
        std::vector<float> m; // tangents dy/dx at knots
    };

    static void ValidateInputs(
        const cv::Mat& img,
        const std::vector<float>& radius,
        const std::vector<float>& rValues,
        const std::vector<float>& gValues,
        const std::vector<float>& bValues)
    {
        if (img.empty())
            throw std::invalid_argument("Image is empty.");

        if (img.type() != CV_32FC3)
            throw std::invalid_argument("Image must have type CV_32FC3.");

        const size_t n = radius.size();

        if (n < 3 || n > 10)
            throw std::invalid_argument("Radius point count must be in range 3..10.");

        if (rValues.size() != n || gValues.size() != n || bValues.size() != n)
            throw std::invalid_argument("RGB value arrays must have same size as radius array.");

        if (std::abs(radius.front()) > 1e-6f)
            throw std::invalid_argument("First radius point must be 0.0.");

        if (std::abs(radius.back() - 1.0f) > 1e-6f)
            throw std::invalid_argument("Last radius point must be 1.0.");

        for (size_t i = 0; i < n; ++i)
        {
            if (radius[i] < 0.0f || radius[i] > 1.0f)
                throw std::invalid_argument("Radius points must be in [0..1].");

            if (i > 0 && !(radius[i] > radius[i - 1]))
                throw std::invalid_argument("Radius points must be strictly increasing.");
        }
    }

    static Spline1D BuildSpline(
        const std::vector<float>& x,
        const std::vector<float>& y)
    {
        const size_t n = x.size();

        Spline1D s;
        s.x = x;
        s.y = y;
        s.m.resize(n, 0.0f);

        // Derivative at center forced to 0.0
        s.m[0] = 0.0f;

        // Interior tangents
        for (size_t i = 1; i + 1 < n; ++i)
        {
            const float dx = x[i + 1] - x[i - 1];
            s.m[i] = (dx > 0.0f) ? (y[i + 1] - y[i - 1]) / dx : 0.0f;
        }

        // Last tangent
        {
            const float dx = x[n - 1] - x[n - 2];
            s.m[n - 1] = (dx > 0.0f) ? (y[n - 1] - y[n - 2]) / dx : 0.0f;
        }

        return s;
    }

    static inline float EvalHermiteSegment(
        float x0, float x1,
        float y0, float y1,
        float m0, float m1,
        float x)
    {
        const float h = x1 - x0;
        if (h <= 0.0f)
            return y0;

        const float t = (x - x0) / h;
        const float t2 = t * t;
        const float t3 = t2 * t;

        const float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
        const float h10 = t3 - 2.0f * t2 + t;
        const float h01 = -2.0f * t3 + 3.0f * t2;
        const float h11 = t3 - t2;

        return h00 * y0 + h10 * h * m0 + h01 * y1 + h11 * h * m1;
    }

    static float EvalSpline(const Spline1D& s, float xr)
    {
        const size_t n = s.x.size();

        // Left extrapolation by first segment
        if (xr <= s.x[0])
        {
            return EvalHermiteSegment(
                s.x[0], s.x[1],
                s.y[0], s.y[1],
                s.m[0], s.m[1],
                xr);
        }

        // Right extrapolation by last segment
        if (xr >= s.x[n - 1])
        {
            return EvalHermiteSegment(
                s.x[n - 2], s.x[n - 1],
                s.y[n - 2], s.y[n - 1],
                s.m[n - 2], s.m[n - 1],
                xr);
        }

        // Find segment
        size_t i = 0;
        for (; i + 1 < n; ++i)
        {
            if (xr >= s.x[i] && xr <= s.x[i + 1])
                break;
        }

        return EvalHermiteSegment(
            s.x[i], s.x[i + 1],
            s.y[i], s.y[i + 1],
            s.m[i], s.m[i + 1],
            xr);
    }

    // Writes gain factors into img (CV_32FC3).
    // Input arrays are RGB order.
    // OpenCV image channels are stored as BGR.
    void BuildRadialGainImage(
        cv::Mat& img,
        const std::vector<float>& radius,
        const std::vector<float>& rValues,
        const std::vector<float>& gValues,
        const std::vector<float>& bValues)
    {
        ValidateInputs(img, radius, rValues, gValues, bValues);

        const Spline1D splineR = BuildSpline(radius, rValues);
        const Spline1D splineG = BuildSpline(radius, gValues);
        const Spline1D splineB = BuildSpline(radius, bValues);

        const float cx = 0.5f * float(img.cols - 1);
        const float cy = 0.5f * float(img.rows - 1);

        // Inscribed circle radius
        const float rMax = 0.5f * float(std::min(img.cols, img.rows));
        if (rMax <= 0.0f)
            throw std::invalid_argument("Invalid inscribed-circle radius.");

        for (int y = 0; y < img.rows; ++y)
        {
            cv::Vec3f* row = img.ptr<cv::Vec3f>(y);

            for (int x = 0; x < img.cols; ++x)
            {
                const float dx = float(x) - cx;
                const float dy = float(y) - cy;
                const float r = std::sqrt(dx * dx + dy * dy) / rMax;

                const float gainR = EvalSpline(splineR, r);
                const float gainG = EvalSpline(splineG, r);
                const float gainB = EvalSpline(splineB, r);

                // OpenCV channel order is BGR
                row[x][0] = gainB;
                row[x][1] = gainG;
                row[x][2] = gainR;
            }
        }
    }
}