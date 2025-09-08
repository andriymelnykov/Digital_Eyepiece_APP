#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

namespace popfx {

    // -------- Params ----------
    struct Params {
        float strength = 0.6f;   // overall amount; -0.3..1.0
        int   radius = 24;     // local contrast scale (px @ full-res)
        float eps = 1.5e-4f;// maps to bilateral sigmaColor
        float micro = 0.6f;   // micro-contrast relative gain
        float midWidth = 0.35f;  // midtone bell width (0.2..0.5)
        float base_ds = 0.5f;   // downsample factor for base (0.33..0.75); 1.0 = off
        bool  use_opencl = true;  // try OpenCL via UMat if available
    };

    // ---------- helpers (work for Mat or UMat via templates) ----------
    template<class Arr>
    static Arr clamp01(const Arr& m) {
        Arr tmp, out;
        cv::max(m, cv::Scalar::all(0), tmp);   // <-- Scalar for UMat compatibility
        cv::min(tmp, cv::Scalar::all(1), out); // <-- Scalar for UMat compatibility
        return out;
    }

    template<class Arr>
    static Arr luminance709(const Arr& bgr32f) {
        std::vector<Arr> ch; cv::split(bgr32f, ch); // B,G,R
        Arr y, t;
        cv::multiply(ch[2], 0.2126, y);      // R
        cv::multiply(ch[1], 0.7152, t); cv::add(y, t, y); // G
        cv::multiply(ch[0], 0.0722, t); cv::add(y, t, y); // B
        return y;
    }

    template<class Arr>
    static Arr midtoneMask(const Arr& gray, float width) {
        Arr gCentered, sq, ex;
        cv::subtract(gray, 0.5, gCentered);
        cv::multiply(gCentered, gCentered, sq);                // (x-0.5)^2
        double k = -1.0 / (2.0 * width * width);
        cv::multiply(sq, k, ex);                               // -(..)/(2w^2)
        Arr mask; cv::exp(ex, mask);                           // exp(...)

        //imshow("mask", mask);
        //cv::waitKey(1);

        return mask;
    }

    template<class Arr>
    static Arr highpassGauss(const Arr& gray, double sigma) {
        Arr blur; cv::GaussianBlur(gray, blur, cv::Size(), sigma, sigma, cv::BORDER_REPLICATE);
        Arr hp; cv::subtract(gray, blur, hp);
        return hp;
    }

    template<class Arr>
    static void bilateralBase(const Arr& gray, Arr& base, int radius, float eps, float ds) {
        // map eps (~1e-4..3e-4) to a reasonable sigmaColor on [0..1] images
        double sigmaColor = std::max(0.02, std::sqrt((double)eps) * 8.0);
        double sigmaSpace_full = std::max(1.0, (double)radius);

        if (ds < 1.0) {
            cv::Size smallSz(std::max(2, int(gray.cols * ds)),
                std::max(2, int(gray.rows * ds)));
            Arr small, smallBase;
            cv::resize(gray, small, smallSz, 0, 0, cv::INTER_AREA);
            double sigmaSpace = sigmaSpace_full * ds;
            cv::bilateralFilter(small, smallBase, -1, sigmaColor, sigmaSpace, cv::BORDER_REPLICATE);
            cv::resize(smallBase, base, gray.size(), 0, 0, cv::INTER_LINEAR);
        }
        else {
            cv::bilateralFilter(gray, base, -1, sigmaColor, sigmaSpace_full, cv::BORDER_REPLICATE);
        }
    }

    template<class Arr>
    static Arr applyRatioToBGR(const Arr& bgr, const Arr& ratio) {
        std::vector<Arr> ch; cv::split(bgr, ch);
        for (int i = 0; i < 3; ++i) cv::multiply(ch[i], ratio, ch[i]);
        Arr out; cv::merge(ch, out);
        return out;
    }

    template<class Arr>
    static Arr popEffectCore(const Arr& srcBGR, const Params& p) {
        Arr src = clamp01(srcBGR);

        // Luminance
        Arr Y = luminance709(src);

        // Edge-preserving base & detail
        Arr base; bilateralBase(Y, base, p.radius, p.eps, p.base_ds);
        Arr detail; cv::subtract(Y, base, detail);

        // Midtone emphasis
        Arr m = midtoneMask(base, p.midWidth);

        Arr detailBoost; cv::multiply(detail, m, detailBoost);
        Arr hp = highpassGauss(Y, 1.0);
        Arr hpBoost; cv::multiply(hp, m, hpBoost);

        // Enhanced luminance
        Arr tmp, Y_enh;
        cv::multiply(detailBoost, p.strength, tmp);
        Arr hpScaled; cv::multiply(hpBoost, p.strength * p.micro, hpScaled);
        cv::add(Y, tmp, Y_enh);
        cv::add(Y_enh, hpScaled, Y_enh);

        // ratio clamp (use Scalar for UMat compatibility)
        const double epsRatio = 1e-6;
        Arr denom, num, ratio;
        cv::add(Y, epsRatio, denom);
        cv::add(Y_enh, epsRatio, num);
        cv::divide(num, denom, ratio);
        cv::min(ratio, cv::Scalar(2.5), ratio); // <-- Scalar
        cv::max(ratio, cv::Scalar(0.4), ratio); // <-- Scalar

        Arr out = applyRatioToBGR(src, ratio);
        return clamp01(out);
    }

    // ---------- public API ----------

    /**
     * Pop effect. Input: CV_32FC3 BGR in [0,1]. Output: same.
     * Will run on OpenCL (UMat) when available & use_opencl==true, else CPU (Mat).
     */
    inline cv::Mat popEffect(const cv::Mat& src32fBGR, const Params& params = {}) {
        CV_Assert(src32fBGR.type() == CV_32FC3);
        Params p = params;

        bool canOCL = p.use_opencl && cv::ocl::haveOpenCL();
        if (canOCL) cv::ocl::setUseOpenCL(true);

        if (canOCL) {
            cv::UMat uSrc, uOut;
            src32fBGR.copyTo(uSrc);
            uOut = popEffectCore<cv::UMat>(uSrc, p);
            cv::Mat out; uOut.copyTo(out);
            return out;
        }
        else {
            // CPU path (OpenCV is internally multi-threaded/vec optimized)
            return popEffectCore<cv::Mat>(src32fBGR, p);
        }
    }

} // namespace popfx
