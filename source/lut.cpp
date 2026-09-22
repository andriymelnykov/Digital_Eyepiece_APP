float minFloat(float a, float b) {
    if (a < b)
        return a;
    else
        return b;
}


void compute_LUT(float gamma) {

    if (debug_flag == 1) {
        cout << "Computing LUT" << endl;
        logfile << "Computing LUT" << endl;
    }

    if (gamma < 1.0)
        gamma_dark = gamma;
    else if (gamma < 15)
        gamma_dark = 0.286 * gamma + 0.714;
    else
        gamma_dark = 0.118 * gamma + 3.235;


    //cout << "gamma: " << gamma << " " << gamma_dark << endl;

    LUT_max_y = atan(gamma);

    LUT_dark_max_y = atan(gamma_dark);

    //LUT_star_max_y = atan(gamma);

    for (int i = 0; i < LUT_size; i++) {
        LUT_in[i] = i / (float)(LUT_size);

        float m = 10;
        float a = 9 / 4 / m / m;
        float c = 4 * m * m * m / 27;
        float f;
        if (LUT_in[i] < a)
            f = m * LUT_in[i] - c * LUT_in[i] * LUT_in[i];
        else
            f = sqrt(LUT_in[i]);

        LUT_out[i] = (atan(LUT_in[i] * gamma) / LUT_max_y * star_protection_factor) + (f * (1 - star_protection_factor));
        LUT_dark_out[i] = (atan(LUT_in[i] * gamma_dark) / LUT_dark_max_y * star_protection_factor) + (f * (1 - star_protection_factor));
        //LUT_star_out[i] = minFloat(
        //    (atan(LUT_in[i] * gamma) / LUT_star_max_y * star_factor) + (f * (1 - star_factor)),
        //    LUT_out[i]);

        //LUT_out[i] = LUT_in[i];  // no stretch
        //LUT_out[i] = gamma * LUT_in[i];  // simple liniear

        if (blkp_mode == 1)
        {
            if (LUT_in[i] < blkp_x1_monitor)
                LUT_blkp_monitor[i] = blkp_y1_monitor / blkp_x1_monitor * LUT_in[i];
            else
                LUT_blkp_monitor[i] = ((1 - blkp_y1_monitor) * LUT_in[i] + (blkp_y1_monitor - blkp_x1_monitor)) / (1 - blkp_x1_monitor);

            if (LUT_in[i] < blkp_x1_eyepiece)
                LUT_blkp_eyepiece[i] = blkp_y1_eyepiece / blkp_x1_eyepiece * LUT_in[i];
            else
                LUT_blkp_eyepiece[i] = ((1 - blkp_y1_eyepiece) * LUT_in[i] + (blkp_y1_eyepiece - blkp_x1_eyepiece)) / (1 - blkp_x1_eyepiece);
        }
    }

    //printf("lut %f %f %f\n %f %f %f\n", LUT_in[0], LUT_in[1], LUT_in[LUT_size - 1], LUT_out[0], LUT_out[1], LUT_out[LUT_size - 1]);

    //Show LUT
    /*
    Mat LUTImage(500, 500, CV_8UC1, Scalar(0));

    for (int i = 1; i < LUT_size; i++)
    {
        //line(LUTImage, Point(0,250), Point(500, 250),  Scalar(128), 1, 8, 0);
        //line(LUTImage, Point(250, 0), Point(250, 500), Scalar(128), 1, 8, 0);
        //line(LUTImage, Point(cvRound(500 * LUT_in[i - 1]), cvRound(500 - 500* LUT_blkp_eyepiece[i - 1])),
        //               Point(cvRound(500 * LUT_in[i]), cvRound(500 - 500 * LUT_blkp_eyepiece[i])),
        //               Scalar(255), 1, 8, 0);
        line(LUTImage, Point(cvRound(500 * LUT_in[i - 1]), cvRound(500 - 500 * LUT_dark_out[i - 1])),
            Point(cvRound(500 * LUT_in[i]), cvRound(500 - 500 * LUT_dark_out[i])),
            Scalar(128), 1, 8, 0);
        line(LUTImage, Point(cvRound(500 * LUT_in[i - 1]), cvRound(500 - 500 * LUT_out[i - 1])),
            Point(cvRound(500 * LUT_in[i]), cvRound(500 - 500 * LUT_out[i])),
            Scalar(200), 1, 8, 0);
        line(LUTImage, Point(cvRound(500 * LUT_in[i - 1]), cvRound(500 - 500 * LUT_star_out[i - 1])),
            Point(cvRound(500 * LUT_in[i]), cvRound(500 - 500 * LUT_star_out[i])),
            Scalar(80), 1, 8, 0);

    }
    imshow("LUT", LUTImage);
    /**/
}


void compute_LUT_noise(float gamma_n) {

    if (debug_flag == 1) {
        cout << "Computing LUT for noise filter" << endl;
        logfile << "Computing LUT for noise filter" << endl;
    }

    LUT_noise_max_y = atan(gamma_n);

    for (int i = 0; i < LUT_size_noise; i++) {
        LUT_noise_in[i] = i / (float)(LUT_size_noise);

        //LUT_noise_out[i] = atan(LUT_noise_in[i] * gamma_n) / LUT_noise_max_y;

        //LUT_noise_inv[i] = tan(LUT_noise_in[i] * LUT_noise_max_y) / gamma_n;

        float m = 10;
        float a = 9 / 4 / m / m;
        float c = 4 * m * m * m / 27;
        float f;
        if (LUT_noise_in[i] < a)
            f = m * LUT_noise_in[i] - c * LUT_noise_in[i] * LUT_noise_in[i];
        else
            f = sqrt(LUT_noise_in[i]);

        LUT_noise_out[i] = (atan(LUT_noise_in[i] * gamma_n) / LUT_noise_max_y * star_protection_factor) + (f * (1 - star_protection_factor));

    }

    // Sweep once to build inverse by linear interpolation
    int i = 0; // index into forward LUT
    for (int j = 0; j < LUT_size_noise; ++j) {
        float y = j / (float)LUT_size_noise;

        // Advance i until LUT_out[i] <= y <= LUT_out[i+1]
        while (i + 1 < LUT_size_noise && LUT_noise_out[i + 1] < y) {
            ++i;
        }

        if (i + 1 >= LUT_size_noise) {
            // y is at/above the last sample (numerical edge)
            LUT_noise_inv[j] = LUT_noise_in[LUT_size_noise - 1];
            continue;
        }

        float y0 = LUT_noise_out[i];
        float y1 = LUT_noise_out[i + 1];
        float x0 = LUT_noise_in[i];
        float x1 = LUT_noise_in[i + 1];

        // Protect against division by zero in flat segments
        float denom = (y1 - y0);
        float t = (denom > 1e-12f) ? (y - y0) / denom : 0.0f;

        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        LUT_noise_inv[j] = x0 + t * (x1 - x0);
    }
}


void compute_LUT_star(float gamma) { 

    if (debug_flag == 1) {
        cout << "Computing LUT for bright stars" << endl;
        logfile << "Computing LUT for bright stars" << endl;
    }

    LUT_star_max_y = atan(gamma);

    for (int i = 0; i < LUT_size; i++) {

        float m = 10;
        float a = 9 / 4 / m / m;
        float c = 4 * m * m * m / 27;
        float f;
        if (LUT_in[i] < a)
            f = m * LUT_in[i] - c * LUT_in[i] * LUT_in[i];
        else
            f = sqrt(LUT_in[i]);

        LUT_star_out[i] = minFloat(
            (atan(LUT_in[i] * gamma) / LUT_star_max_y * star_factor) + (f * (1 - star_factor)),
            LUT_out[i]);
    }

    //printf("lut %f %f %f\n %f %f %f\n", LUT_in[0], LUT_in[1], LUT_in[LUT_size - 1], LUT_out[0], LUT_out[1], LUT_out[LUT_size - 1]);

    //Show LUT
    /*
    Mat LUTImage(500, 500, CV_8UC1, Scalar(0));

    for (int i = 1; i < LUT_size; i++)
    {
        //line(LUTImage, Point(0,250), Point(500, 250),  Scalar(128), 1, 8, 0);
        //line(LUTImage, Point(250, 0), Point(250, 500), Scalar(128), 1, 8, 0);
        //line(LUTImage, Point(cvRound(500 * LUT_in[i - 1]), cvRound(500 - 500* LUT_blkp_eyepiece[i - 1])),
        //               Point(cvRound(500 * LUT_in[i]), cvRound(500 - 500 * LUT_blkp_eyepiece[i])),
        //               Scalar(255), 1, 8, 0);
        line(LUTImage, Point(cvRound(500 * LUT_in[i - 1]), cvRound(500 - 500 * LUT_dark_out[i - 1])),
            Point(cvRound(500 * LUT_in[i]), cvRound(500 - 500 * LUT_dark_out[i])),
            Scalar(128), 1, 8, 0);
        line(LUTImage, Point(cvRound(500 * LUT_in[i - 1]), cvRound(500 - 500 * LUT_out[i - 1])),
            Point(cvRound(500 * LUT_in[i]), cvRound(500 - 500 * LUT_out[i])),
            Scalar(200), 1, 8, 0);
        line(LUTImage, Point(cvRound(500 * LUT_in[i - 1]), cvRound(500 - 500 * LUT_star_out[i - 1])),
            Point(cvRound(500 * LUT_in[i]), cvRound(500 - 500 * LUT_star_out[i])),
            Scalar(80), 1, 8, 0);

    }
    imshow("LUT", LUTImage);
    waitKey(1);
    /**/
}


float LookUpTable(float x, float LookUp_in[], float LookUp_out[], unsigned int LookUp_length) {

    // old linear interpolation
    /*
    float out = 0;

    //liniear look-up table
    if (x <= LookUp_in[0]) out = LookUp_out[0];
    else  if (x >= LookUp_in[LookUp_length - 1]) out = LookUp_out[LookUp_length - 1];
    else {
        unsigned int i = 0;
        while (x >= LookUp_in[i + 1]) i++;
        out = (float)LookUp_out[i] + ((float)LookUp_out[i + 1] - (float)LookUp_out[i]) * ((float)x - (float)LookUp_in[i]) / ((float)LookUp_in[i + 1] - (float)LookUp_in[i]);
    }

    return out;
    /**/

    // faster linear interpolation
    /*
    float index = x * (LookUp_length - 1);

    unsigned int indexLower = static_cast<unsigned int>(index);
    unsigned int indexUpper = indexLower + 1;

    //if (indexUpper >= 1000) {
    //    indexUpper = 999; // Ensure we don't go out of bounds
    //}
    if (x <= 0) return LookUp_out[0];
    else  if (x >= 1) return LookUp_out[LookUp_length - 1];
    else {
        float weightUpper = index - indexLower;
        float weightLower = 1.0f - weightUpper;

        return (weightLower * LookUp_out[indexLower]) + (weightUpper * LookUp_out[indexUpper]);
        //return (LookUp_out[indexLower]);
    }
    /**/

    // fast without linear interpolation
    /**/
    float index = x * (LookUp_length - 1);
    //float index = x/2 * (LookUp_length - 1);

    int indexLower = static_cast<unsigned int>(index);

    if (indexLower < 0) return LookUp_out[0];
    else if (indexLower >= LookUp_length) return LookUp_out[LookUp_length - 1];
    else return (1.0 * LookUp_out[indexLower]);
    /**/


}




#include <algorithm>
#include <stdexcept>

cv::Mat rgbBlendClip(
    const cv::Mat& image_lum_stretched,
    const cv::Mat& image_rgb,
    bool clampOutput = true
) {
    if (image_lum_stretched.empty() || image_rgb.empty()) {
        throw std::runtime_error("Input images must not be empty.");
    }

    if (image_lum_stretched.size() != image_rgb.size()) {
        throw std::runtime_error("Input images must have the same size.");
    }

    if (image_lum_stretched.type() != CV_32FC3 || image_rgb.type() != CV_32FC3) {
        throw std::runtime_error("Input images must be CV_32FC3.");
    }

    cv::Mat output(image_lum_stretched.size(), CV_32FC3);

    constexpr float maxValue = 1.0f;
    constexpr float eps = 1e-12f;

    for (int y = 0; y < image_lum_stretched.rows; ++y) {
        const cv::Vec3f* lumPtr = image_lum_stretched.ptr<cv::Vec3f>(y);
        const cv::Vec3f* rgbPtr = image_rgb.ptr<cv::Vec3f>(y);
        cv::Vec3f* outPtr = output.ptr<cv::Vec3f>(y);

        for (int x = 0; x < image_lum_stretched.cols; ++x) {
            const cv::Vec3f& lum = lumPtr[x];  // r', g', b' or BGR channels
            const cv::Vec3f& rgb = rgbPtr[x];  // r'', g'', b''

            float k = 1.0f;

            for (int c = 0; c < 3; ++c) {
                const float a = lum[c];
                const float b = rgb[c];

                const float diff = a - b;

                // We only need to limit k if moving from rgb -> lum increases
                // the channel above 1.0.
                if (diff > eps && a > maxValue) {
                    float kc = (maxValue - b) / diff;
                    k = std::min(k, kc);
                }
            }

            // Keep k in the valid blend range.
            k = std::clamp(k, 0.0f, 1.0f);

            cv::Vec3f out;
            for (int c = 0; c < 3; ++c) {
                out[c] = k * lum[c] + (1.0f - k) * rgb[c];

                if (clampOutput) {
                    out[c] = std::clamp(out[c], 0.0f, 1.0f);
                }
            }

            outPtr[x] = out;
        }
    }

    return output;
}

Mat blurChromaYCrCb(const cv::Mat& bgr32f, double sigma = 1.5)
{
    CV_Assert(bgr32f.type() == CV_32FC3);

    // Important: bgr32f should normally be in range [0, 1].
    cv::Mat ycrcb;
    cv::cvtColor(bgr32f, ycrcb, cv::COLOR_BGR2YCrCb);

    std::vector<cv::Mat> ch;
    cv::split(ycrcb, ch);

    // ch[0] = Y  : keep sharp
    // ch[1] = Cr : blur color
    // ch[2] = Cb : blur color
    cv::GaussianBlur(ch[1], ch[1], Size(0, 0), sigma, sigma, cv::BORDER_DEFAULT);
    cv::GaussianBlur(ch[2], ch[2], Size(0, 0), sigma, sigma, cv::BORDER_DEFAULT);

    cv::merge(ch, ycrcb);

    cv::Mat out;
    cv::cvtColor(ycrcb, out, cv::COLOR_YCrCb2BGR);

    // Optional, useful if later code assumes valid display range.
    //cv::min(cv::max(out, 0.0f), 1.0f, out);

    return out;
}




inline float lut_lookup(float value, const float* lut_out, int lut_size) {
    if (!std::isfinite(value)) {
        return lut_out[0];
    }

    float index = value * static_cast<float>(lut_size - 1);
    //int indexLower = static_cast<int>(std::floor(index));
    int indexLower = static_cast<int>(index);

    indexLower = std::clamp(indexLower, 0, lut_size - 1);

    return lut_out[indexLower];
}



void _apply_lut_to_mat(cv::Mat& image, const float* lut_out, int lut_size) {
    CV_Assert(image.depth() == CV_32F);

    const int channels = image.channels();

    for (int y = 0; y < image.rows; ++y) {
        float* row = image.ptr<float>(y);

        for (int x = 0; x < image.cols * channels; ++x) {
            row[x] = lut_lookup(row[x], lut_out, lut_size);
        }
    }
}

void apply_lut_to_mat(cv::Mat& image, const float* lut_out, int lut_size) {
    CV_Assert(image.depth() == CV_32F);

    const int channels = image.channels();

    if (image.isContinuous()) {
        const int total_values = image.rows * image.cols * channels;

        float* p = image.ptr<float>(0);

        for (int i = 0; i < total_values; ++i) {
            p[i] = lut_lookup(p[i], lut_out, lut_size);
        }
    }
    else {
        const int row_values = image.cols * channels;

        for (int y = 0; y < image.rows; ++y) {
            float* row = image.ptr<float>(y);

            for (int x = 0; x < row_values; ++x) {
                row[x] = lut_lookup(row[x], lut_out, lut_size);
            }
        }
    }
}

void apply_lut_stretch(cv::Mat& image, const float* lut_out, int lut_size, float factor, const std::string& debug_message) {
    CV_Assert(image.depth() == CV_32F);

    //factor:
    //0 - rgb separate channel stretch
    //1 - luminance stretch preserving rgb
    //0.02..0.98 - mix of two
    factor = std::clamp(factor, 0.0f, 1.0f);

    if (debug_flag == 1) {
        std::cout << debug_message << std::endl;
        logfile << debug_message << std::endl;
    }

    const bool do_lum_stretch = factor > 0.01f;

    cv::Mat image_original = image.clone();

    cv::Mat image_lum_stretched;

    if (do_lum_stretch && image.channels() == 3) {
        std::vector<cv::Mat> bgr_planes;
        cv::split(image_original, bgr_planes);

        // OpenCV is BGR:
        // bgr_planes[0] = B
        // bgr_planes[1] = G
        // bgr_planes[2] = R
        cv::Mat grey_linear =
            0.333f * bgr_planes[2] +
            0.333f * bgr_planes[1] +
            0.333f * bgr_planes[0];

        cv::Mat grey_stretched = grey_linear.clone();

        apply_lut_to_mat(grey_stretched, lut_out, lut_size);

        cv::Mat grey_safe;
        cv::max(grey_linear, cv::Scalar::all(0.00001f), grey_safe);

        cv::Mat scale;
        cv::divide(grey_stretched, grey_safe, scale);

        cv::multiply(bgr_planes[0], scale, bgr_planes[0]);
        cv::multiply(bgr_planes[1], scale, bgr_planes[1]);
        cv::multiply(bgr_planes[2], scale, bgr_planes[2]);

        cv::merge(bgr_planes, image_lum_stretched);
    }

    // Always create the per-channel RGB-stretched version.
    image = image_original.clone();
    apply_lut_to_mat(image, lut_out, lut_size);

    // If requested, create luminance result protected by RGB blend clipping.
    if (do_lum_stretch && image.channels() == 3) {
        //image_lum_stretched = image_lum_stretched / 1.2;
        //image = image / 1.2;

        //double minVal, maxVal;
        //minMaxLoc(image_lum_stretched.reshape(1), &minVal, &maxVal);
        //cout << "max = " << maxVal << std::endl;

        image_lum_stretched = blurChromaYCrCb(image_lum_stretched, 2.0);

        //minMaxLoc(image_lum_stretched.reshape(1), &minVal, &maxVal);
        //cout << "max = " << maxVal << std::endl;

        image_lum_stretched = rgbBlendClip(image_lum_stretched, image);

        image = image * (1.0f - factor) + image_lum_stretched * factor;
    }

    // For grayscale images, luminance mode does not apply.
    // image is already per-channel/per-pixel LUT-stretched.
}


void gamma_correction(cv::Mat& image, float factor) {
    apply_lut_stretch(image, LUT_out, LUT_size, factor, "Apply stretch");
}

void gamma_dark_correction(cv::Mat& image, float factor) {
    apply_lut_stretch(image, LUT_dark_out, LUT_size, factor, "Apply dark stretch");
}

void gamma_star_correction(cv::Mat& image, float factor) {
    apply_lut_stretch(image, LUT_star_out, LUT_size, factor, "Apply star stretch");
}

void gamma_noise_correction(cv::Mat& image, float factor) {
    apply_lut_stretch(image, LUT_noise_out, LUT_size_noise, factor, "Apply noise stretch");
}

void gamma_noise_inv_correction(cv::Mat& image, float factor) {
    apply_lut_stretch(image, LUT_noise_inv, LUT_size_noise, factor, "Apply noise inv stretch");
}

void blkp_monitor_correction(cv::Mat& image, float factor) {
    apply_lut_stretch(image, LUT_blkp_monitor, LUT_size, factor, "Apply black point monitor correction");
}

void blkp_eyepiece_correction(cv::Mat& image, float factor) {
    apply_lut_stretch(image, LUT_blkp_eyepiece, LUT_size, factor, "Apply black point eyepiece correction");
}











void _gamma_correction(Mat& image, float factor) {

    // Gamma correction
    //image = gamma * image;

    //factor:
    //0 - rgb separate channel stretch
    //1 - luminance stretch preserving rgb
    //0.02..0.98 - mix of two


    if (debug_flag == 1) {
        cout << "Apply stretch" << endl;
        logfile << "Apply stretch" << endl;
    }

    // old algorithm
    /*
    vector<Mat> bgr_planes;
    split(image, bgr_planes);
    for (int i = 0; i < image.rows; i++)
        for (int j = 0; j < image.cols; j++) {
            bgr_planes[0].at<float>(i, j) = LookUpTable(bgr_planes[0].at<float>(i, j), LUT_in, LUT_out, LUT_size);
            bgr_planes[1].at<float>(i, j) = LookUpTable(bgr_planes[1].at<float>(i, j), LUT_in, LUT_out, LUT_size);
            bgr_planes[2].at<float>(i, j) = LookUpTable(bgr_planes[2].at<float>(i, j), LUT_in, LUT_out, LUT_size);
        }
    merge(bgr_planes, image);
    /**/

    //bool do_rgb_stretch = factor < 0.99;
    bool do_rgb_stretch = true;
    bool do_lum_stretch = factor > 0.01;

    //luminance stretch part
    Mat image_linear;
    vector<Mat> bgr_planes;
    Mat grey_linear, grey_stretched;
    Mat scale;
    Mat image_lum_stretched;

    if (do_lum_stretch) {
        image_linear = image.clone();

        split(image_linear, bgr_planes);

        Mat grey_linear = 0.3 * bgr_planes[2] + 0.5 * bgr_planes[1] + 0.2 * bgr_planes[0];
        Mat grey_stretched = grey_linear.clone();

        if (grey_stretched.isContinuous()) // check, if gaps in memory
        {
            // using point arithmetics
            int channels = grey_stretched.channels();
            int nrows = grey_stretched.rows;
            int ncols = grey_stretched.cols * channels;

            float* p = (float*)grey_stretched.data;
            for (unsigned int i = 0; i < ncols * nrows; ++i) {
                float index = *p * (LUT_size - 1);

                int indexLower = static_cast<unsigned int>(index);

                if (indexLower < 0) *p = LUT_out[0];
                else if (indexLower >= LUT_size) *p = LUT_out[LUT_size - 1];
                else *p = (1.0 * LUT_out[indexLower]);

                p++;
            }
        }
        else {
            // using iterators - safe, if gaps in memory
            MatIterator_<float> it, end;
            for (it = grey_stretched.begin<float>(), end = grey_stretched.end<float>(); it != end; ++it)
            {
                (*it) = LookUpTable((*it), LUT_in, LUT_out, LUT_size);
            }
        }

        imshow("grey_stretched", grey_stretched);
        waitKey(1);

        max(grey_linear, Scalar::all(0.00001f), grey_linear);
        divide(grey_stretched, grey_linear, scale);

        multiply(bgr_planes[2], scale, bgr_planes[2]);
        multiply(bgr_planes[1], scale, bgr_planes[1]);
        multiply(bgr_planes[0], scale, bgr_planes[0]);

        merge(bgr_planes, image_lum_stretched);

        //imshow("image_lum_stretched", image_lum_stretched);
        //waitKey(1);

        //double minVal, maxVal;
        //minMaxLoc(image_lum_stretched.reshape(1), &minVal, &maxVal);
        //cout << "max = " << maxVal << std::endl;
    }

    if (do_rgb_stretch) {
        if (image.isContinuous()) // check, if gaps in memory
        {
            // using point arithmetics
            int channels = image.channels();
            int nrows = image.rows;
            int ncols = image.cols * channels;

            //cout << "continuous" << endl;
            float* p = (float*)image.data;
            for (unsigned int i = 0; i < ncols * nrows; ++i) {
                //*p++ = LookUpTable(*p, LUT_in, LUT_out, LUT_size);

                float index = *p * (LUT_size - 1);

                int indexLower = static_cast<unsigned int>(index);

                if (indexLower < 0) *p = LUT_out[0];
                else if (indexLower >= LUT_size) *p = LUT_out[LUT_size - 1];
                else *p = (1.0 * LUT_out[indexLower]);

                p++;
            }
        }
        else {
            // using iterators - safe, if gaps in memory
            if (image.channels() == 3) {
                MatIterator_<Vec3f> it, end;
                for (it = image.begin<Vec3f>(), end = image.end<Vec3f>(); it != end; ++it)
                {
                    (*it)[0] = LookUpTable((*it)[0], LUT_in, LUT_out, LUT_size);
                    (*it)[1] = LookUpTable((*it)[1], LUT_in, LUT_out, LUT_size);
                    (*it)[2] = LookUpTable((*it)[2], LUT_in, LUT_out, LUT_size);
                }
            }
            else {
                MatIterator_<float> it, end;
                for (it = image.begin<float>(), end = image.end<float>(); it != end; ++it)
                {
                    (*it) = LookUpTable((*it), LUT_in, LUT_out, LUT_size);
                }
            }
        }
    }
    
    if (do_lum_stretch)
        image_lum_stretched = rgbBlendClip(image_lum_stretched, image);
    
    if (do_rgb_stretch && do_lum_stretch) {
        image = image * (1.0 - factor) + image_lum_stretched * factor;
    }
    else if (do_lum_stretch) {
        image = image_lum_stretched.clone();
    }
    else {
        //do nothing
    }

}


void _gamma_dark_correction(Mat& image, float gamma) {
    // Gamma correction
    //image = 0.1 * image;



    /**/
    if (debug_flag == 1) {
        cout << "Apply dark stretch" << endl;
        logfile << "Apply dark stretch" << endl;
    }

    if (image.isContinuous()) // check, if gaps in memory
    {
        // using point arithmetics
        int channels = image.channels();
        int nrows = image.rows;
        int ncols = image.cols * channels;

        //cout << "continuous" << endl;
        float* p = (float*)image.data;
        for (unsigned int i = 0; i < ncols * nrows; ++i) {
            //*p++ = LookUpTable(*p, LUT_in, LUT_out, LUT_size);

            float index = *p * (LUT_size - 1);

            int indexLower = static_cast<unsigned int>(index);

            if (indexLower < 0) *p = LUT_dark_out[0];
            else if (indexLower >= LUT_size) *p = LUT_dark_out[LUT_size - 1];
            else *p = (1.0 * LUT_dark_out[indexLower]);

            p++;
        }
    }
    else {
        // using iterators - safe, if gaps in memory
        if (image.channels() == 3) {
            MatIterator_<Vec3f> it, end;
            for (it = image.begin<Vec3f>(), end = image.end<Vec3f>(); it != end; ++it)
            {
                (*it)[0] = LookUpTable((*it)[0], LUT_in, LUT_dark_out, LUT_size);
                (*it)[1] = LookUpTable((*it)[1], LUT_in, LUT_dark_out, LUT_size);
                (*it)[2] = LookUpTable((*it)[2], LUT_in, LUT_dark_out, LUT_size);
            }
        }
        else {
            MatIterator_<float> it, end;
            for (it = image.begin<float>(), end = image.end<float>(); it != end; ++it)
            {
                (*it) = LookUpTable((*it), LUT_in, LUT_dark_out, LUT_size);
            }
        }
    }
    /**/
}


void _gamma_star_correction(Mat& image, float gamma) {

    /**/
    if (debug_flag == 1) {
        cout << "Apply star stretch" << endl;
        logfile << "Apply star stretch" << endl;
    }

    if (image.isContinuous()) // check, if gaps in memory
    {
        // using point arithmetics
        int channels = image.channels();
        int nrows = image.rows;
        int ncols = image.cols * channels;

        //cout << "continuous" << endl;
        float* p = (float*)image.data;
        for (unsigned int i = 0; i < ncols * nrows; ++i) {
            float index = *p * (LUT_size - 1);

            int indexLower = static_cast<unsigned int>(index);

            if (indexLower < 0) *p = LUT_star_out[0];
            else if (indexLower >= LUT_size) *p = LUT_star_out[LUT_size - 1];
            else *p = (1.0 * LUT_star_out[indexLower]);

            p++;
        }
    }
    else {
        // using iterators - safe, if gaps in memory
        if (image.channels() == 3) {
            MatIterator_<Vec3f> it, end;
            for (it = image.begin<Vec3f>(), end = image.end<Vec3f>(); it != end; ++it)
            {
                (*it)[0] = LookUpTable((*it)[0], LUT_in, LUT_star_out, LUT_size);
                (*it)[1] = LookUpTable((*it)[1], LUT_in, LUT_star_out, LUT_size);
                (*it)[2] = LookUpTable((*it)[2], LUT_in, LUT_star_out, LUT_size);
            }
        }
        else {
            MatIterator_<float> it, end;
            for (it = image.begin<float>(), end = image.end<float>(); it != end; ++it)
            {
                (*it) = LookUpTable((*it), LUT_in, LUT_star_out, LUT_size);
            }
        }
    }
    /**/
}


void _gamma_noise_correction(Mat& image, float gamma) {
    // Gamma correction
    //image = 0.1 * image;



    /**/
    if (debug_flag == 1) {
        cout << "Apply noise stretch" << endl;
        logfile << "Apply noise stretch" << endl;
    }

    if (image.isContinuous()) // check, if gaps in memory
    {
        // using point arithmetics
        int channels = image.channels();
        int nrows = image.rows;
        int ncols = image.cols * channels;

        //cout << "continuous" << endl;
        float* p = (float*)image.data;
        for (unsigned int i = 0; i < ncols * nrows; ++i) {
            //*p++ = LookUpTable(*p, LUT_noise_in, LUT_noise_out, LUT_size_noise);

            float index = *p * (LUT_size_noise - 1);

            int indexLower = static_cast<unsigned int>(index);

            if (indexLower < 0) *p = LUT_noise_out[0];
            else if (indexLower >= LUT_size_noise) *p = LUT_noise_out[LUT_size_noise - 1];
            else *p = (1.0 * LUT_noise_out[indexLower]);

            p++;
        }
    }
    else {
        // using iterators - safe, if gaps in memory
        MatIterator_<Vec3f> it, end;
        for (it = image.begin<Vec3f>(), end = image.end<Vec3f>(); it != end; ++it)
        {
            (*it)[0] = LookUpTable((*it)[0], LUT_noise_in, LUT_noise_out, LUT_size_noise);
            (*it)[1] = LookUpTable((*it)[1], LUT_noise_in, LUT_noise_out, LUT_size_noise);
            (*it)[2] = LookUpTable((*it)[2], LUT_noise_in, LUT_noise_out, LUT_size_noise);
        }
    }
    /**/
}


void _gamma_noise_inv_correction(Mat& image, float gamma) {
    // Gamma correction
    //image = 0.1 * image;



    /**/
    if (debug_flag == 1) {
        cout << "Apply noise inv stretch" << endl;
        logfile << "Apply noise inv stretch" << endl;
    }

    if (image.isContinuous()) // check, if gaps in memory
    {
        // using point arithmetics
        int channels = image.channels();
        int nrows = image.rows;
        int ncols = image.cols * channels;

        //cout << "continuous" << endl;
        float* p = (float*)image.data;
        for (unsigned int i = 0; i < ncols * nrows; ++i) {
            //*p++ = LookUpTable(*p, LUT_noise_in, LUT_noise_out, LUT_size_noise);

            float index = *p * (LUT_size_noise - 1);

            int indexLower = static_cast<unsigned int>(index);

            if (indexLower < 0) *p = LUT_noise_inv[0];
            else if (indexLower >= LUT_size_noise) *p = LUT_noise_inv[LUT_size_noise - 1];
            else *p = (1.0 * LUT_noise_inv[indexLower]);

            p++;
        }
    }
    else {
        // using iterators - safe, if gaps in memory
        MatIterator_<Vec3f> it, end;
        for (it = image.begin<Vec3f>(), end = image.end<Vec3f>(); it != end; ++it)
        {
            (*it)[0] = LookUpTable((*it)[0], LUT_noise_in, LUT_noise_inv, LUT_size_noise);
            (*it)[1] = LookUpTable((*it)[1], LUT_noise_in, LUT_noise_inv, LUT_size_noise);
            (*it)[2] = LookUpTable((*it)[2], LUT_noise_in, LUT_noise_inv, LUT_size_noise);
        }
    }
    /**/
}


void _blkp_monitor_correction(Mat& image) {
    if (debug_flag == 1) {
        cout << "Apply black point monitor correction" << endl;
        logfile << "Apply black point monitor correction" << endl;
    }

    if (image.isContinuous()) // check, if gaps in memory
    {
        // using point arithmetics
        int channels = image.channels();
        int nrows = image.rows;
        int ncols = image.cols * channels;

        //cout << "continuous" << endl;
        float* p = (float*)image.data;
        for (unsigned int i = 0; i < ncols * nrows; ++i) {
            //*p++ = LookUpTable(*p, LUT_in, LUT_out, LUT_size);

            float index = *p * (LUT_size - 1);

            int indexLower = static_cast<unsigned int>(index);

            if (indexLower < 0) *p = LUT_blkp_monitor[0];
            else if (indexLower >= LUT_size) *p = LUT_blkp_monitor[LUT_size - 1];
            else *p = (1.0 * LUT_blkp_monitor[indexLower]);

            p++;
        }
    }
    else {
        // using iterators - safe, if gaps in memory
        if (image.channels() == 3) {
            MatIterator_<Vec3f> it, end;
            for (it = image.begin<Vec3f>(), end = image.end<Vec3f>(); it != end; ++it)
            {
                (*it)[0] = LookUpTable((*it)[0], LUT_in, LUT_blkp_monitor, LUT_size);
                (*it)[1] = LookUpTable((*it)[1], LUT_in, LUT_blkp_monitor, LUT_size);
                (*it)[2] = LookUpTable((*it)[2], LUT_in, LUT_blkp_monitor, LUT_size);
            }
        }
        else {
            MatIterator_<float> it, end;
            for (it = image.begin<float>(), end = image.end<float>(); it != end; ++it)
            {
                (*it) = LookUpTable((*it), LUT_in, LUT_blkp_monitor, LUT_size);
            }
        }
    }
    /**/
}


void _blkp_eyepiece_correction(Mat& image) {
    if (debug_flag == 1) {
        cout << "Apply black point eyepiece correction" << endl;
        logfile << "Apply black point eyepiece correction" << endl;
    }

    if (image.isContinuous()) // check, if gaps in memory
    {
        // using point arithmetics
        int channels = image.channels();
        int nrows = image.rows;
        int ncols = image.cols * channels;

        //cout << "continuous" << endl;
        float* p = (float*)image.data;
        for (unsigned int i = 0; i < ncols * nrows; ++i) {
            //*p++ = LookUpTable(*p, LUT_in, LUT_out, LUT_size);

            float index = *p * (LUT_size - 1);

            int indexLower = static_cast<unsigned int>(index);

            if (indexLower < 0) *p = LUT_blkp_eyepiece[0];
            else if (indexLower >= LUT_size) *p = LUT_blkp_eyepiece[LUT_size - 1];
            else *p = (1.0 * LUT_blkp_eyepiece[indexLower]);

            p++;
        }
    }
    else {
        // using iterators - safe, if gaps in memory
        if (image.channels() == 3) {
            MatIterator_<Vec3f> it, end;
            for (it = image.begin<Vec3f>(), end = image.end<Vec3f>(); it != end; ++it)
            {
                (*it)[0] = LookUpTable((*it)[0], LUT_in, LUT_blkp_eyepiece, LUT_size);
                (*it)[1] = LookUpTable((*it)[1], LUT_in, LUT_blkp_eyepiece, LUT_size);
                (*it)[2] = LookUpTable((*it)[2], LUT_in, LUT_blkp_eyepiece, LUT_size);
            }
        }
        else {
            MatIterator_<float> it, end;
            for (it = image.begin<float>(), end = image.end<float>(); it != end; ++it)
            {
                (*it) = LookUpTable((*it), LUT_in, LUT_blkp_eyepiece, LUT_size);
            }
        }
    }
    /**/
}