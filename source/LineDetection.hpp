#include <opencv2/opencv.hpp>
#include <iostream>

#include <vector>
#include <cmath>

cv::Mat keepOnlyLines(const cv::Mat& src8u)
{
    CV_Assert(src8u.type() == CV_8UC1);

    // 4. Close small gaps in lines
    cv::Mat closed;
    cv::morphologyEx(
        src8u,
        closed,
        cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11))
    );

    // 5. Connected components
    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(
        closed,
        labels,
        stats,
        centroids,
        8,
        CV_32S
    );

    cv::Mat lineMask = cv::Mat::zeros(src8u.size(), CV_8U);

    // Tune these for your image scale
    const int minArea = 20;
    const float minLength = 30.0f;
    const float minAspectRatio = 6.0f;

    for (int label = 1; label < n; ++label) // label 0 is background
    {
        int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < minArea)
            continue;

        cv::Mat componentMask = labels == label;

        std::vector<cv::Point> pts;
        cv::findNonZero(componentMask, pts);

        if (pts.size() < 5)
            continue;

        cv::RotatedRect box = cv::minAreaRect(pts);

        float w = box.size.width;
        float h = box.size.height;

        float longSide = std::max(w, h);
        float shortSide = std::max(1.0f, std::min(w, h));

        float aspect = longSide / shortSide;

        if (longSide >= minLength && aspect >= minAspectRatio)
        {
            lineMask.setTo(255, componentMask);
        }
    }

    // 6. Keep original float values only where lines were detected
    //cv::Mat result = cv::Mat::zeros(src32f.size(), src32f.type());
    //src32f.copyTo(result, lineMask);

    return lineMask;
}





cv::Mat keepOnlyLinesHoughP(
    const cv::Mat& src8u,
    double cannyLow = 50.0,
    double cannyHigh = 150.0,
    int houghThreshold = 50,
    double minLineLength = 40.0,
    double maxLineGap = 10.0,
    int outputLineThickness = 5
)
{
    CV_Assert(src8u.type() == CV_8UC1);

    // 2. Suppress noise a bit.
    //cv::Mat blurred;
    //cv::GaussianBlur(src8u, blurred, cv::Size(3, 3), 0);

    // 3. Edge image for HoughLinesP.
    cv::Mat edges;
    //cv::Canny(blurred, edges, cannyLow, cannyHigh, 3);
    cv::Canny(src8u, edges, cannyLow, cannyHigh, 3);

    // Optional: join tiny gaps in edge map before Hough.
    cv::morphologyEx(
        edges,
        edges,
        cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11))
    );

    // 4. Probabilistic Hough line detection.
    // OpenCV docs note that the input image may be modified, so use a clone.
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(
        edges.clone(),
        lines,
        1,                 // rho resolution in pixels
        CV_PI / 180.0,     // theta resolution in radians
        houghThreshold,    // accumulator threshold
        minLineLength,     // reject short segments
        maxLineGap         // join gaps on the same line
    );

    // 5. Draw detected line segments into a mask.
    cv::Mat lineMask = cv::Mat::zeros(src8u.size(), CV_8U);

    for (const cv::Vec4i& l : lines)
    {
        cv::Point p1(l[0], l[1]);
        cv::Point p2(l[2], l[3]);

        cv::line(
            lineMask,
            p1,
            p2,
            cv::Scalar(255),
            outputLineThickness,
            cv::LINE_AA
        );
    }

    // Optional: dilate slightly to cover thick line interiors.
    // Useful if Hough detected only the edges of thick bright lines.
    if (outputLineThickness <= 2)
    {
        cv::dilate(
            lineMask,
            lineMask,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11))
        );
    }

    // 6. Keep original float values only near detected lines.
    //cv::Mat result = cv::Mat::zeros(src32f.size(), src32f.type());
    //src32f.copyTo(result, lineMask);

    return lineMask;
}