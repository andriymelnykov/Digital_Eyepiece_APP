// Background compensation for linear astro EAA frames (CV_32FC3 BGR 0..1)
// - Annulus sampling + robust local background estimates (median or lower-quantile)
// - Optional structure mask (DoG) to exclude stars/halos/structured nebulosity
// - Thin-Plate Spline (TPS) RBF regression with smoothing lambda
// - Additive or multiplicative correction
//
// Build: OpenCV only
// Note: For speed, TPS is evaluated on a coarse grid and upsampled.

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

enum class BgMode { Additive, Multiplicative };
enum class BgEstimator { Median, LowerQuantile };

struct BgCompParams {
    // Annulus (fractions of min(w,h))
    float outerFrac = 1.0f;
    float innerFrac = 0.6f;

    // Sampling density: N samples across outer diameter (2*r_out)
    int   N = 16;

    // Local window radius for robust estimation (pixels)
    int   winR = 32;

    // Robust clipping
    int   clipIters = 3;
    float kLow = 3.5f;
    float kHigh = 2.0f;

    // Local estimator selection
    BgEstimator estimator = BgEstimator::Median;
    float q = 0.2f;                 // for LowerQuantile (0..1)

    // Optional structure mask (DoG on luminance)
    bool  useMask = true;
    float maskSigma1 = 2.0f;
    float maskSigma2 = 15.0f;
    float maskThresh = 3.0f;        // threshold in robust-sigma units
    int   maskDilate = 6;           // dilation radius in pixels (covers halos)

    // TPS smoothing
    double lambda = 1e-3;

    // Evaluate background on evalGrid x evalGrid then upsample
    int evalGrid = 256;

    // Numeric safety
    float eps = 1e-6f;

    // Minimum number of pixels kept in window after masking/clipping
    int minKept = 200;
};

// ---------- Robust stats helpers ----------

static float median_inplace(std::vector<float>& v) {
    if (v.empty()) return 0.0f;
    size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    float m = v[mid];
    if (v.size() % 2 == 0) {
        std::nth_element(v.begin(), v.begin() + (mid - 1), v.end());
        m = 0.5f * (m + v[mid - 1]);
    }
    return m;
}

static float quantile_inplace(std::vector<float>& v, float q) {
    if (v.empty()) return 0.0f;
    q = std::min(1.0f, std::max(0.0f, q));
    size_t k = (size_t)std::floor(q * (v.size() - 1));
    std::nth_element(v.begin(), v.begin() + k, v.end());
    return v[k];
}

static float mad_sigma(const std::vector<float>& v, float med) {
    if (v.empty()) return 0.0f;
    std::vector<float> d(v.size());
    for (size_t i = 0; i < v.size(); ++i) d[i] = std::fabs(v[i] - med);
    float mad = median_inplace(d);
    return 1.4826f * mad;
}

// ---------- Structure mask (Option 3) ----------
// Mask pixels likely belonging to stars/halos/structured nebulosity.
// Returns CV_8U mask, 255 = exclude from local background stats.

static cv::Mat buildStructureMask(const cv::Mat& imgBGR32F, const BgCompParams& p) {
    CV_Assert(imgBGR32F.type() == CV_32FC3);

    std::vector<cv::Mat> ch(3);
    cv::split(imgBGR32F, ch);

    // Luminance-ish (BGR weights)
    cv::Mat L = 0.114f * ch[0] + 0.587f * ch[1] + 0.299f * ch[2];

    cv::Mat g1, g2, dog;
    cv::GaussianBlur(L, g1, cv::Size(0, 0), p.maskSigma1, p.maskSigma1, cv::BORDER_REPLICATE);
    cv::GaussianBlur(L, g2, cv::Size(0, 0), p.maskSigma2, p.maskSigma2, cv::BORDER_REPLICATE);
    dog = g1 - g2; // positive = small/medium bright structure

    // Robust normalize DoG by MAD (global)
    std::vector<float> vals;
    vals.reserve((size_t)dog.total());
    for (int y = 0; y < dog.rows; ++y) {
        const float* r = dog.ptr<float>(y);
        vals.insert(vals.end(), r, r + dog.cols);
    }
    std::vector<float> tmp = vals;
    float med = median_inplace(tmp);
    float sig = mad_sigma(vals, med);
    if (sig < 1e-12f) sig = 1e-12f;

    cv::Mat Z = (dog - med) * (1.0f / sig);

    cv::Mat mask;
    cv::threshold(Z, mask, p.maskThresh, 255.0, cv::THRESH_BINARY);
    mask.convertTo(mask, CV_8U);

    if (p.maskDilate > 0) {
        int r = p.maskDilate;
        cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * r + 1, 2 * r + 1));
        cv::dilate(mask, mask, se);
    }
    return mask;
}

// ---------- Robust local background estimator (Option 2 + masking) ----------

static bool robust_local_bg(
    const cv::Mat& ch, int cx, int cy, int winR,
    int clipIters, float kLow, float kHigh, int minKept,
    BgEstimator estimator, float q,
    const cv::Mat* excludeMask, // optional CV_8U (same size). nonzero => exclude
    float& outBg)
{
    int x0 = std::max(0, cx - winR);
    int x1 = std::min(ch.cols - 1, cx + winR);
    int y0 = std::max(0, cy - winR);
    int y1 = std::min(ch.rows - 1, cy + winR);

    std::vector<float> vals;
    vals.reserve((size_t)(x1 - x0 + 1) * (size_t)(y1 - y0 + 1));

    for (int y = y0; y <= y1; ++y) {
        const float* row = ch.ptr<float>(y);
        const uchar* mrow = excludeMask ? excludeMask->ptr<uchar>(y) : nullptr;
        for (int x = x0; x <= x1; ++x) {
            if (mrow && mrow[x]) continue;
            vals.push_back(row[x]);
        }
    }

    if ((int)vals.size() < minKept) return false;

    // Iterative asymmetric sigma-clipping around median
    for (int it = 0; it < clipIters; ++it) {
        std::vector<float> tmp = vals;
        float med = median_inplace(tmp);
        float sig = mad_sigma(vals, med);
        if (sig < 1e-12f) { outBg = med; return true; }

        float lo = med - kLow * sig;
        float hi = med + kHigh * sig;

        std::vector<float> kept;
        kept.reserve(vals.size());
        for (float v : vals) if (v >= lo && v <= hi) kept.push_back(v);

        if ((int)kept.size() < minKept) return false;
        vals.swap(kept);
    }

    // Final statistic after clipping
    if (estimator == BgEstimator::Median) {
        std::vector<float> tmp = vals;
        outBg = median_inplace(tmp);
    }
    else {
        std::vector<float> tmp = vals;
        outBg = quantile_inplace(tmp, q);
    }
    return true;
}

// ---------- TPS (Thin Plate Spline) ----------

static inline double tps_phi(double r) {
    const double e = 1e-12;
    double rr = r * r;
    return rr * std::log(r + e);
}

// Solve TPS coefficients for one channel.
// pts: N points (x,y), vals: N values.
// coeff: (N+3)x1 with [w0..wN-1, a0, a1, a2]
static bool fit_tps(
    const std::vector<cv::Point2d>& pts,
    const std::vector<double>& vals,
    double lambda,
    cv::Mat& coeff)
{
    int N = (int)pts.size();
    if (N < 6) return false;

    int M = N + 3;
    cv::Mat A = cv::Mat::zeros(M, M, CV_64F);
    cv::Mat b = cv::Mat::zeros(M, 1, CV_64F);

    // K + lambda I
    for (int i = 0; i < N; ++i) {
        A.at<double>(i, i) = lambda;
        for (int j = i + 1; j < N; ++j) {
            double dx = pts[i].x - pts[j].x;
            double dy = pts[i].y - pts[j].y;
            double r = std::sqrt(dx * dx + dy * dy);
            double kij = tps_phi(r);
            A.at<double>(i, j) = kij;
            A.at<double>(j, i) = kij;
        }
        b.at<double>(i, 0) = vals[i];
    }

    // P block
    for (int i = 0; i < N; ++i) {
        A.at<double>(i, N + 0) = 1.0;
        A.at<double>(i, N + 1) = pts[i].x;
        A.at<double>(i, N + 2) = pts[i].y;

        A.at<double>(N + 0, i) = 1.0;
        A.at<double>(N + 1, i) = pts[i].x;
        A.at<double>(N + 2, i) = pts[i].y;
    }

    cv::Mat x;
    bool ok = cv::solve(A, b, x, cv::DECOMP_SVD);
    if (!ok || x.empty()) return false;
    coeff = x;
    return true;
}

static inline double eval_tps(
    const std::vector<cv::Point2d>& pts,
    const cv::Mat& coeff,
    double x, double y)
{
    int N = (int)pts.size();
    const double* c = coeff.ptr<double>(0);

    // affine
    double sum = c[N + 0] + c[N + 1] * x + c[N + 2] * y;

    // rbf
    for (int i = 0; i < N; ++i) {
        double dx = x - pts[i].x;
        double dy = y - pts[i].y;
        double r = std::sqrt(dx * dx + dy * dy);
        sum += c[i] * tps_phi(r);
    }
    return sum;
}

static float median_of_mat32f(const cv::Mat& m) {
    CV_Assert(m.type() == CV_32F);
    std::vector<float> v;
    v.reserve((size_t)m.total());
    for (int y = 0; y < m.rows; ++y) {
        const float* row = m.ptr<float>(y);
        v.insert(v.end(), row, row + m.cols);
    }
    return median_inplace(v);
}

// ---------- Main callable ----------
//
// img: CV_32FC3 linear BGR 0..1
// outBackground: optional full-res CV_32FC3 background model
//   - in Additive mode: background in linear space
//   - in Multiplicative mode: background in LOG space (per channel)

inline cv::Mat compensateBackgroundTPS(
    const cv::Mat& img, const BgCompParams& p, BgMode mode,
    cv::Mat* outBackground = nullptr)
{
    CV_Assert(img.type() == CV_32FC3);

    const int w = img.cols, h = img.rows;
    const double m = std::min(w, h);
    const double cx = 0.5 * (w - 1), cy = 0.5 * (h - 1);

    const double rOut = 0.5 * p.outerFrac * m;
    const double rIn = 0.5 * p.innerFrac * m;
    const double rOut2 = rOut * rOut;
    const double rIn2 = rIn * rIn;

    const double step = (2.0 * rOut) / std::max(2, p.N);

    // Optional structure mask
    cv::Mat structureMask;
    const cv::Mat* maskPtr = nullptr;
    if (p.useMask) {
        structureMask = buildStructureMask(img, p);
        maskPtr = &structureMask;
    }

    // Split channels
    std::vector<cv::Mat> ch(3);
    cv::split(img, ch);

    // Collect samples in the annulus
    std::vector<cv::Point2d> pts;
    std::vector<double> vB, vG, vR;
    pts.reserve(512); vB.reserve(512); vG.reserve(512); vR.reserve(512);

    const int margin = p.winR + 2;

    for (double yy = margin; yy < h - margin; yy += step) {
        for (double xx = margin; xx < w - margin; xx += step) {
            double dx = xx - cx, dy = yy - cy;
            double d2 = dx * dx + dy * dy;
            if (d2 < rIn2 || d2 > rOut2) continue;

            int ix = (int)std::round(xx);
            int iy = (int)std::round(yy);

            float b = 0, g = 0, r = 0;
            bool okB = robust_local_bg(ch[0], ix, iy, p.winR,
                p.clipIters, p.kLow, p.kHigh, p.minKept,
                p.estimator, p.q, maskPtr, b);
            bool okG = robust_local_bg(ch[1], ix, iy, p.winR,
                p.clipIters, p.kLow, p.kHigh, p.minKept,
                p.estimator, p.q, maskPtr, g);
            bool okR = robust_local_bg(ch[2], ix, iy, p.winR,
                p.clipIters, p.kLow, p.kHigh, p.minKept,
                p.estimator, p.q, maskPtr, r);
            if (!(okB && okG && okR)) continue;

            if (mode == BgMode::Multiplicative) {
                b = std::log(std::max(b, p.eps));
                g = std::log(std::max(g, p.eps));
                r = std::log(std::max(r, p.eps));
            }

            pts.emplace_back(xx, yy);
            vB.push_back((double)b);
            vG.push_back((double)g);
            vR.push_back((double)r);
        }
    }

    // Fit TPS per channel
    cv::Mat cB, cG, cR;
    if (!fit_tps(pts, vB, p.lambda, cB) ||
        !fit_tps(pts, vG, p.lambda, cG) ||
        !fit_tps(pts, vR, p.lambda, cR))
    {
        if (outBackground) *outBackground = cv::Mat::zeros(img.size(), img.type());
        return img.clone();
    }

    // Evaluate on coarse grid then upsample
    const int G = std::max(32, p.evalGrid);
    cv::Mat bgSmall(G, G, CV_32FC3);

    for (int gy = 0; gy < G; ++gy) {
        double y = (double)gy * (h - 1) / (G - 1);
        cv::Vec3f* row = bgSmall.ptr<cv::Vec3f>(gy);
        for (int gx = 0; gx < G; ++gx) {
            double x = (double)gx * (w - 1) / (G - 1);
            double bb = eval_tps(pts, cB, x, y);
            double gg = eval_tps(pts, cG, x, y);
            double rr = eval_tps(pts, cR, x, y);
            row[gx] = cv::Vec3f((float)bb, (float)gg, (float)rr);
        }
    }

    cv::Mat bgFull;
    cv::resize(bgSmall, bgFull, img.size(), 0, 0, cv::INTER_CUBIC);

    // Robust reference for normalization (median over the evaluated background field)
    std::vector<cv::Mat> bgCh(3);
    cv::split(bgFull, bgCh);
    float refB = median_of_mat32f(bgCh[0]);
    float refG = median_of_mat32f(bgCh[1]);
    float refR = median_of_mat32f(bgCh[2]);

    cv::Mat out = img.clone();

    if (mode == BgMode::Additive) {
        // subtract only variation: (bg - ref)
        for (int y = 0; y < h; ++y) {
            cv::Vec3f* o = out.ptr<cv::Vec3f>(y);
            const cv::Vec3f* bgr = bgFull.ptr<cv::Vec3f>(y);
            for (int x = 0; x < w; ++x) {
                o[x][0] -= (bgr[x][0] - refB);
                o[x][1] -= (bgr[x][1] - refG);
                o[x][2] -= (bgr[x][2] - refR);
            }
        }
    }
    else {
        // bgFull is in log space; divide by exp(bg - ref) => median correction ~ 1
        for (int y = 0; y < h; ++y) {
            cv::Vec3f* o = out.ptr<cv::Vec3f>(y);
            const cv::Vec3f* L = bgFull.ptr<cv::Vec3f>(y);
            for (int x = 0; x < w; ++x) {
                float cb = std::exp(L[x][0] - refB);
                float cg = std::exp(L[x][1] - refG);
                float cr = std::exp(L[x][2] - refR);
                o[x][0] = o[x][0] / std::max(cb, p.eps);
                o[x][1] = o[x][1] / std::max(cg, p.eps);
                o[x][2] = o[x][2] / std::max(cr, p.eps);
            }
        }
    }

    if (outBackground) *outBackground = bgFull;
    return out;
}
