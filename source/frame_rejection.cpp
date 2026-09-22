// ---for satellite rejection

void get_delta_image(Mat& first_image, Mat& current_image, Mat& delta_image, int& diff_pixels)
{
    // get linear delta image
    Mat delta_im;
    delta_im = current_image - first_image;

    // get linear delta, 1-ch with some blur
    Mat delta_image_gray;
    if (delta_im.channels() == 3)
        cvtColor(delta_im, delta_image_gray, cv::COLOR_BGR2GRAY);
    else
        delta_im.copyTo(delta_image_gray);
    GaussianBlur(delta_image_gray, delta_image_gray, Size(11, 11), 0);

    // get 16u full mask
    Mat delta_image_gray_32sc1;
    delta_image_gray.convertTo(delta_image_gray_32sc1, CV_32SC1, 65535.0);
    MedianMadStats stat = medianMadStats(delta_image_gray_32sc1);
    //cout << stat.median << "  " << stat.sigmaMad << endl;
    Mat delta_image_gray_16uc1;
    delta_image_gray_32sc1.convertTo(delta_image_gray_16uc1, CV_16UC1);
    threshold(delta_image_gray_16uc1, delta_image_gray_16uc1, 4 * stat.sigmaMad, 65535, THRESH_BINARY);

    //imshow("delta", delta_image_gray_16uc1);
    //waitKey(1);

    // get 8u only lines mask
    Mat delta_image_gray_8uc1;
    Mat LineMask;
    delta_image_gray_16uc1.convertTo(delta_image_gray_8uc1, CV_8UC1);
    LineMask = keepOnlyLines(delta_image_gray_8uc1);
    //LineMask = keepOnlyLinesHoughP(delta_image_gray_8uc1);

    //imshow("lines", LineMask);
    //waitKey(1);

    // get number of pixels in detected lines
    diff_pixels = cv::countNonZero(LineMask);



    // get delta linear image without background and noise
    delta_im.copyTo(delta_image);
    GaussianBlur(delta_image, delta_image, Size(5, 5), 0);
    delta_image = delta_image - Scalar::all(5 * (float)stat.sigmaMad / 65535);
    max(delta_image, Scalar::all(0.0f), delta_image);
    //min(delta_image, Scalar::all(1.0f), delta_image);

    //imshow("delta_image", delta_image * 500);
    //waitKey(1);

    //cout << "diff pixels: " << diff_pixels << endl;

}

// ---for shaky image rejection

struct ShakyFrameParams {
    int max_shift = 10;
    int shift_step = 1;
    int pixel_step = 2;
    float threshold_sigma = 1.5f;
    float clamp_sigma = 400.0f;  //400.0f;  //20.0f; // <= 0 disables clamp
    bool suppress_large_objects = true;
    int large_object_kernel = 31;      // downsampled pixels, rounded to odd
    float large_object_sigma = 6.0f;   // threshold above background in sigma units
    int large_object_dilate = 9;       // downsampled pixels, rounded to odd
};

struct ShakyFrameMetrics {
    bool valid = false;
    float background = 0.0f;
    float sigma = 0.0f;
    float corr_floor = 0.0f;
    float lambda1 = 0.0f;
    float lambda2 = 0.0f;
    float size = 0.0f;
    float elongation = 0.0f;
    float peak = 0.0f;
    float quality = 0.0f;
};

ShakyFrameMetrics calculate_shaky_frame_metrics(const Mat& input_image, const ShakyFrameParams& params)
{
    ShakyFrameMetrics metrics;

    if (input_image.empty())
        return metrics;
    if ((input_image.channels() != 1) && (input_image.channels() != 3))
        return metrics;
    if ((params.max_shift < 1) || (params.shift_step < 1) || (params.pixel_step < 1))
        return metrics;
    if ((input_image.cols < 16) || (input_image.rows < 16))
        return metrics;

    Rect roi(input_image.cols / 4, input_image.rows / 4, input_image.cols / 2, input_image.rows / 2);
    Mat cropped = input_image(roi);

    Mat gray;
    if (cropped.channels() == 3)
        cvtColor(cropped, gray, cv::COLOR_BGR2GRAY);
    else
        cropped.copyTo(gray);

    if (gray.depth() != CV_32F)
        gray.convertTo(gray, CV_32F);

    Mat small;
    resize(gray, small, Size(gray.cols / 2, gray.rows / 2), 0, 0, INTER_AREA);
    if ((small.cols <= 2 * params.max_shift) || (small.rows <= 2 * params.max_shift))
        return metrics;

    //imshow("small gray", small); waitKey(1);

    Mat small_32sc1;
    small.convertTo(small_32sc1, CV_32SC1, 65535.0);
    MedianMadStats stat = medianMadStats(small_32sc1);
    if ((stat.count <= 0) || !std::isfinite(stat.median) || !std::isfinite(stat.sigmaMad))
        return metrics;

    metrics.background = (float)(stat.median / 65535.0);
    metrics.sigma = (float)(stat.sigmaMad / 65535.0);
    if (metrics.sigma <= 0.0f)
        return metrics;

    //cout << metrics.background << " " << metrics.sigma << endl;

    Mat work = small - (metrics.background + params.threshold_sigma * metrics.sigma);
    max(work, Scalar::all(0.0f), work);
    if (params.clamp_sigma > 0.0f) {
        float clamp_value = params.clamp_sigma * metrics.sigma;
        min(work, Scalar::all(clamp_value), work);
    }

    //imshow("work", work); waitKey(1);

    if (params.suppress_large_objects && (params.large_object_kernel > 1) && (params.large_object_sigma > 0.0f)) {
        int large_kernel_size = params.large_object_kernel;
        if ((large_kernel_size % 2) == 0)
            large_kernel_size++;

        Mat large_kernel = getStructuringElement(MORPH_ELLIPSE, Size(large_kernel_size, large_kernel_size));
        Mat large_objects;
        morphologyEx(work, large_objects, MORPH_OPEN, large_kernel);

        Mat large_mask;
        threshold(large_objects, large_mask, params.large_object_sigma * metrics.sigma, 255.0, THRESH_BINARY);
        large_mask.convertTo(large_mask, CV_8U);

        if (params.large_object_dilate > 0) {
            int dilate_kernel_size = params.large_object_dilate;
            if ((dilate_kernel_size % 2) == 0)
                dilate_kernel_size++;

            Mat dilate_kernel = getStructuringElement(MORPH_ELLIPSE, Size(dilate_kernel_size, dilate_kernel_size));
            dilate(large_mask, large_mask, dilate_kernel);
        }

        work.setTo(0.0f, large_mask);

        //imshow("mask", large_mask); waitKey(1);
    }

    //imshow("work_masked", work); waitKey(1);

    std::vector<int> shifts;
    for (int shift = -params.max_shift; shift <= params.max_shift; shift += params.shift_step)
        shifts.push_back(shift);

    int shift_count = static_cast<int>(shifts.size());
    Mat corr(shift_count, shift_count, CV_32FC1, Scalar::all(0));

    for (int cy = 0; cy < shift_count; cy++) {
        int dy = shifts[cy];
        int y0 = std::max(0, -dy);
        int y1 = std::min(work.rows, work.rows - dy);
        for (int cx = 0; cx < shift_count; cx++) {
            int dx = shifts[cx];
            int x0 = std::max(0, -dx);
            int x1 = std::min(work.cols, work.cols - dx);

            double sum = 0.0;
            for (int y = y0; y < y1; y += params.pixel_step) {
                const float* row1 = work.ptr<float>(y);
                const float* row2 = work.ptr<float>(y + dy);
                for (int x = x0; x < x1; x += params.pixel_step)
                    sum += (double)row1[x] * (double)row2[x + dx];
            }

            corr.at<float>(cy, cx) = (float)sum;
        }
    }

    //imshow("corr", corr); waitKey(1);

    std::vector<double> border_values;
    border_values.reserve((size_t)(corr.cols * 2 + corr.rows * 2));
    for (int y = 0; y < corr.rows; y++) {
        const float* row = corr.ptr<float>(y);
        for (int x = 0; x < corr.cols; x++) {
            if ((x == 0) || (x == corr.cols - 1) || (y == 0) || (y == corr.rows - 1))
                border_values.push_back(row[x]);
        }
    }

    metrics.corr_floor = (float)medianOfVector(border_values);
    corr = corr - metrics.corr_floor;
    max(corr, Scalar::all(0.0f), corr);

    //imshow("corr", corr / (cv::norm(corr, cv::NORM_INF) + 1e-30f)); waitKey(1);

    double sum_corr = 0.0;
    double mxx = 0.0;
    double myy = 0.0;
    double mxy = 0.0;
    double peak = 0.0;

    for (int cy = 0; cy < corr.rows; cy++) {
        const float* row = corr.ptr<float>(cy);
        int dy = shifts[cy];
        for (int cx = 0; cx < corr.cols; cx++) {
            int dx = shifts[cx];
            double c = row[cx];
            sum_corr += c;
            mxx += c * dx * dx;
            myy += c * dy * dy;
            mxy += c * dx * dy;
            if (c > peak)
                peak = c;
        }
    }

    if (sum_corr <= 0.0)
        return metrics;

    mxx /= sum_corr;
    myy /= sum_corr;
    mxy /= sum_corr;

    double trace = mxx + myy;
    double det_term = (mxx - myy) * (mxx - myy) + 4.0 * mxy * mxy;
    if (det_term < 0.0)
        det_term = 0.0;
    double root = sqrt(det_term);
    double lambda1 = 0.5 * (trace + root);
    double lambda2 = 0.5 * (trace - root);

    if (lambda1 < 0.0)
        lambda1 = 0.0;
    if (lambda2 < 0.0)
        lambda2 = 0.0;

    metrics.lambda1 = (float)lambda1;
    metrics.lambda2 = (float)lambda2;
    metrics.size = (float)(lambda1 + lambda2);
    metrics.peak = (float)peak;
    metrics.quality = (float)(peak / (sum_corr + 1e-30));
    if (lambda2 > 1e-12)
        metrics.elongation = (float)sqrt(lambda1 / lambda2);
    else
        metrics.elongation = 0.0f;

    cout << "Calculated size: " << metrics.size << " elongation: " << metrics.elongation << endl;

    metrics.valid = true;
    return metrics;
}

// ---for cloud rejection
