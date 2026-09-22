void plot_cut(Mat& display_image) {

    //int hist_w, hist_h;
    //hist_w = display_image.cols; hist_h = display_image.rows;

    vector<Mat> bgr_planes;
    split(display_image, bgr_planes);

    for (int i = 1; i < display_image.cols; i++)
    {
        line(display_image, Point(i - 1, cvRound(display_image.cols * (1 - bgr_planes[1].at<float>(cvRound(display_image.rows / 2), i - 1)))),
            Point(i, cvRound(display_image.cols * (1 - bgr_planes[1].at<float>(cvRound(display_image.rows / 2), i)))),
            Scalar(200, 200, 200), 2, 8, 0);
    }

}

void plot_cut2(Mat& display_image) {

    vector<Mat> bgr_planes;
    split(display_image, bgr_planes);

    Mat src8u, dst8u;
    bgr_planes[1].convertTo(src8u, CV_8U, 255.0);
    cv::medianBlur(src8u, dst8u, 41);
    dst8u.convertTo(bgr_planes[1], CV_32F, 1.0 / 255.0);

    for (int i = 1; i < display_image.cols; i++)
    {
        line(display_image, Point(i - 1, cvRound(display_image.cols * (1 - bgr_planes[1].at<float>(cvRound(display_image.rows / 2), i - 1)))),
            Point(i, cvRound(display_image.cols * (1 - bgr_planes[1].at<float>(cvRound(display_image.rows / 2), i)))),
            Scalar(200, 200, 200), 2, 8, 0);
    }

}