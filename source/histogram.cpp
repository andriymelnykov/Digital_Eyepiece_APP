void show_histogram(Mat& image) {
    vector<Mat> bgr_planes;
    split(image, bgr_planes);
    int histSize = 1000;
    float range[] = { 0, 1 }; //the upper boundary is exclusive
    const float* histRange[] = { range };
    bool uniform = true, accumulate = false;
    Mat b_hist, g_hist, r_hist;
    calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, histRange, uniform, accumulate);
    calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, histRange, uniform, accumulate);
    calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, histRange, uniform, accumulate);
    int hist_w = 1000, hist_h = 400;
    int bin_w = cvRound((double)hist_w / histSize);
    Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
    normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    for (int i = 1; i < histSize; i++)
    {
        line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
            Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))),
            Scalar(255, 0, 0), 2, 8, 0);
        line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
            Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
            Scalar(0, 255, 0), 2, 8, 0);
        line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
            Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
            Scalar(0, 0, 255), 2, 8, 0);
    }
    imshow("Histogram", histImage);
}



void show_RAW_histogram(Mat& image) {

    int histSize = 1000;
    float range[] = { 0, 65536 }; //the upper boundary is exclusive
    const float* histRange[] = { range };
    bool uniform = true, accumulate = false;
    Mat hist;
    calcHist(&image, 1, 0, Mat(), hist, 1, &histSize, histRange, uniform, accumulate);

    int hist_w = display_height, hist_h = display_height / 2;
    //int hist_w = 1000, hist_h = 300;
    //int bin_w = cvRound((double)hist_w / histSize);
    float bin_w = (float)hist_w / histSize;
    Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
    normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    for (int i = 1; i < histSize; i++)
    {
        line(histImage, Point(cvRound(bin_w * (i - 1)), hist_h - cvRound(hist.at<float>(i - 1))),
            Point(cvRound(bin_w * (i)), hist_h - cvRound(hist.at<float>(i))),
            Scalar(0, 255, 0), 2, 8, 0);
    }
    imshow("RAW Histogram", histImage);
}


void plot_RAW_histogram(Mat& display_image, Mat& RAW_image) {

    int histSize = 1000;
    float range[] = { 0, 65536 }; //the upper boundary is exclusive
    const float* histRange[] = { range };
    bool uniform = true, accumulate = false;
    Mat hist;
    calcHist(&RAW_image, 1, 0, Mat(), hist, 1, &histSize, histRange, uniform, accumulate);

    int hist_w, hist_h;

    if (special_setup_01 == 1) {
        hist_w = display_image.cols / 2; hist_h = display_image.rows / 2;
    }
    else {
        hist_w = display_image.cols; hist_h = display_image.rows / 2;
    }

    float bin_w = (float)hist_w / histSize;


    //Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

    normalize(hist, hist, 0, hist_h, NORM_MINMAX, -1, Mat());
    for (int i = 1; i < histSize; i++)
    {
        //line(histImage, Point(cvRound(bin_w * (i - 1)), hist_h - cvRound(hist.at<float>(i - 1))),
        //    Point(cvRound(bin_w * (i)), hist_h - cvRound(hist.at<float>(i))),
        //    Scalar(0, 255, 0), 2, 8, 0);

        if (special_setup_01 == 1) {
            line(display_image, Point(display_image.cols / 4 + cvRound(bin_w * (i - 1)), display_image.rows / 4 * 3 - cvRound(hist.at<float>(i - 1))),
                Point(display_image.cols / 4 + cvRound(bin_w * (i)), display_image.rows / 4 * 3 - cvRound(hist.at<float>(i))),
                Scalar(200, 200, 200), 2, 8, 0);
        }
        else {
            line(display_image, Point(cvRound(bin_w * (i - 1)), display_image.rows - cvRound(hist.at<float>(i - 1))),
                Point(cvRound(bin_w * (i)), display_image.rows - cvRound(hist.at<float>(i))),
                Scalar(200, 200, 200), 2, 8, 0);
        }
    }

    if (special_setup_01 == 1) {
        rectangle(display_image, Rect(display_image.cols / 4, display_image.rows / 4 * 3 - hist_h, hist_w, hist_h), Scalar(200, 200, 200), 2);
    }
    else {
        rectangle(display_image, Rect(0, display_image.rows - hist_h, hist_w, hist_h), Scalar(200, 200, 200), 2);
    }
}