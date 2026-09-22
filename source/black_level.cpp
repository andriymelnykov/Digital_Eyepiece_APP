void black_level(Mat& image, float b_level) {

    if (debug_flag == 1) {
        cout << "Apply black level correction" << endl;
        logfile << "Apply black level correction" << endl;
    }

    // Black level correction, histogram threshold from left - b_level

    Mat image2;
    //image.copyTo(image2);
    double resize_factor = 300.0 / image.rows;

    if ((stack_from_file == 2) && (state == foto_state))  // special dataset generation mode
        resize(sub_base_image, image2, Size(0, 0), resize_factor, resize_factor, INTER_AREA);
    else
        resize(image, image2, Size(0, 0), resize_factor, resize_factor, INTER_AREA);

    int sz = 3; //7;
    blur(image2, image2, Size(sz, sz));

    // apply circular mask for background compensation
    //Scalar m = mean(image2);
    if (circular_mask_background_flag == 1) {
        //Prepare circular mask for background
        Mat circular_mask_b(image2.rows, image2.cols, CV_32FC3, Scalar(2, 2, 2));
        circle(circular_mask_b, Point(circular_mask_b.cols / 2, circular_mask_b.rows / 2), round(0.5 * circular_mask_b.rows * circular_mask_background_size), Scalar(0, 0, 0), FILLED, LINE_AA);

        image2 = max(image2, circular_mask_b);
    }
    //image2.copyTo(image);  // test

    vector<Mat> bgr_planes;
    split(image2, bgr_planes);

    int histSize = 10000;
    float range[] = { 0.000001, 1 }; //the upper boundary is exclusive
    const float* histRange[] = { range };
    bool uniform = true, accumulate = false;
    Mat b_hist, g_hist, r_hist;
    calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, histRange, uniform, accumulate);
    calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, histRange, uniform, accumulate);
    calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, histRange, uniform, accumulate);
    //int hist_w = 1000, hist_h = 400;
    //int bin_w = cvRound((double)hist_w / histSize);
    //Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
    normalize(b_hist, b_hist, 0, 1, NORM_MINMAX, -1, Mat());
    normalize(g_hist, g_hist, 0, 1, NORM_MINMAX, -1, Mat());
    normalize(r_hist, r_hist, 0, 1, NORM_MINMAX, -1, Mat());

    int r_offs = 0, g_offs = 0, b_offs = 0;

    for (int i = 1; i < histSize; i++) {
        if (r_offs == 0)
            if (r_hist.at<float>(i) > b_level)
                r_offs = i;
        if (g_offs == 0)
            if (g_hist.at<float>(i) > b_level)
                g_offs = i;
        if (b_offs == 0)
            if (b_hist.at<float>(i) > b_level)
                b_offs = i;
    }

    float b = b_offs / (float)histSize;
    float g = g_offs / (float)histSize;
    float r = r_offs / (float)histSize;

    //printf("black level: %f %f %f\n", b, g, r);
    //printf("black level: %d %d %d\n", b_offs, g_offs, r_offs);

    split(image, bgr_planes);

    bgr_planes[0] = bgr_planes[0] - b;
    bgr_planes[0] = bgr_planes[0] * (1 / (1 - b));
    bgr_planes[1] = bgr_planes[1] - g;
    bgr_planes[1] = bgr_planes[1] * (1 / (1 - g));
    bgr_planes[2] = bgr_planes[2] - r;
    bgr_planes[2] = bgr_planes[2] * (1 / (1 - r));

    merge(bgr_planes, image);
}



void black_level_mono(Mat& image, float b_level) {

    if (debug_flag == 1) {
        cout << "Apply black level mono correction" << endl;
        logfile << "Apply black level mono correction" << endl;
    }

    // Black level correction, histogram threshold from left - b_level

    Mat image2;
    //image.copyTo(image2);
    double resize_factor = 300.0 / image.rows;

    if ((stack_from_file == 2) && (state == foto_state))  // special dataset generation mode
        resize(sub_base_image, image2, Size(0, 0), resize_factor, resize_factor, INTER_LINEAR);
    else
        resize(image, image2, Size(0, 0), resize_factor, resize_factor, INTER_LINEAR);

    int sz = 3; //7;
    blur(image2, image2, Size(sz, sz));

    // apply circular mask for background compensation
    if (circular_mask_background_flag == 1) {
        //Prepare circular mask for background
        Mat circular_mask_b(image2.rows, image2.cols, CV_32FC1, 2.0);
        circle(circular_mask_b, Point(circular_mask_b.cols / 2, circular_mask_b.rows / 2), round(0.5 * circular_mask_b.rows * circular_mask_background_size), 0.0, FILLED, LINE_AA);

        image2 = max(image2, circular_mask_b);
    }
    //image2.copyTo(image);  // test

    //vector<Mat> bgr_planes;
    //split(image2, bgr_planes);

    int histSize = 10000;
    float range[] = { 0.000001, 1 }; //the upper boundary is exclusive
    const float* histRange[] = { range };
    bool uniform = true, accumulate = false;
    Mat m_hist; // b_hist, g_hist, r_hist;
    calcHist(&image2, 1, 0, Mat(), m_hist, 1, &histSize, histRange, uniform, accumulate);
    //calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, histRange, uniform, accumulate);
    //calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, histRange, uniform, accumulate);

    //int hist_w = 1000, hist_h = 400;
    //int bin_w = cvRound((double)hist_w / histSize);
    //Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

    normalize(m_hist, m_hist, 0, 1, NORM_MINMAX, -1, Mat());
    //normalize(g_hist, g_hist, 0, 1, NORM_MINMAX, -1, Mat());
    //normalize(r_hist, r_hist, 0, 1, NORM_MINMAX, -1, Mat());

    int m_offs = 0; // r_offs = 0, g_offs = 0, b_offs = 0;

    for (int i = 1; i < histSize; i++) {
        if (m_offs == 0)
            if (m_hist.at<float>(i) > b_level)
                m_offs = i;
        //if (g_offs == 0)
        //    if (g_hist.at<float>(i) > b_level)
        //        g_offs = i;
        //if (b_offs == 0)
        //    if (b_hist.at<float>(i) > b_level)
        //        b_offs = i;
    }

    float m = m_offs / (float)histSize;
    //float g = g_offs / (float)histSize;
    //float r = r_offs / (float)histSize;

    //printf("black level: %f %f %f\n", b, g, r);
    //printf("black level: %d %d %d\n", b_offs, g_offs, r_offs);

    //split(image, bgr_planes);

    image = image - m;
    image = image * (1 / (1 - m));
    //bgr_planes[0] = bgr_planes[0] - b;
    //bgr_planes[0] = bgr_planes[0] * (1 / (1 - b));
    //bgr_planes[1] = bgr_planes[1] - g;
    //bgr_planes[1] = bgr_planes[1] * (1 / (1 - g));
    //bgr_planes[2] = bgr_planes[2] - r;
    //bgr_planes[2] = bgr_planes[2] * (1 / (1 - r));

    //merge(bgr_planes, image);
}



void black_level_calculate(Mat& image, float b_level, float& b_offs, float& g_offs, float& r_offs) {

    if (debug_flag == 1) {
        cout << "Calculate black level" << endl;
        logfile << "Calculate black level" << endl;
    }

    // Black level calculation, histogram threshold from left - b_level

    vector<Mat> bgr_planes;
    split(image, bgr_planes);
    int histSize = 10000;
    float range[] = { 0.000001, 1 }; //the upper boundary is exclusive
    const float* histRange[] = { range };
    bool uniform = true, accumulate = false;
    Mat b_hist, g_hist, r_hist;
    calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, histRange, uniform, accumulate);
    calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, histRange, uniform, accumulate);
    calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, histRange, uniform, accumulate);
    //int hist_w = 1000, hist_h = 400;
    //int bin_w = cvRound((double)hist_w / histSize);
    //Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
    normalize(b_hist, b_hist, 0, 1, NORM_MINMAX, -1, Mat());
    normalize(g_hist, g_hist, 0, 1, NORM_MINMAX, -1, Mat());
    normalize(r_hist, r_hist, 0, 1, NORM_MINMAX, -1, Mat());

    r_offs = 0, g_offs = 0, b_offs = 0;

    for (int i = 1; i < histSize; i++) {
        if (r_offs == 0)
            if (r_hist.at<float>(i) > b_level)
                r_offs = i;
        if (g_offs == 0)
            if (g_hist.at<float>(i) > b_level)
                g_offs = i;
        if (b_offs == 0)
            if (b_hist.at<float>(i) > b_level)
                b_offs = i;
    }

    b_offs = b_offs / (float)histSize;
    g_offs = g_offs / (float)histSize;
    r_offs = r_offs / (float)histSize;


    //printf("black level: %f %f %f\n", b_offs, g_offs, r_offs);


}



void black_level_calculate_mono(Mat& image, float b_level, float& m_offs) {

    if (debug_flag == 1) {
        cout << "Calculate black level mono" << endl;
        logfile << "Calculate black level mono" << endl;
    }

    // Black level calculation, histogram threshold from left - b_level

    //vector<Mat> bgr_planes;
    //split(image, bgr_planes);
    int histSize = 10000;
    float range[] = { 0.000001, 1 }; //the upper boundary is exclusive
    const float* histRange[] = { range };
    bool uniform = true, accumulate = false;
    Mat m_hist; // b_hist, g_hist, r_hist;
    calcHist(&image, 1, 0, Mat(), m_hist, 1, &histSize, histRange, uniform, accumulate);
    //calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, histRange, uniform, accumulate);
    //calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, histRange, uniform, accumulate);

    //int hist_w = 1000, hist_h = 400;
    //int bin_w = cvRound((double)hist_w / histSize);
    //Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

    normalize(m_hist, m_hist, 0, 1, NORM_MINMAX, -1, Mat());
    //normalize(g_hist, g_hist, 0, 1, NORM_MINMAX, -1, Mat());
    //normalize(r_hist, r_hist, 0, 1, NORM_MINMAX, -1, Mat());

    m_offs = 0;  // r_offs = 0, g_offs = 0, b_offs = 0;

    for (int i = 1; i < histSize; i++) {
        if (m_offs == 0)
            if (m_hist.at<float>(i) > b_level)
                m_offs = i;
        //if (g_offs == 0)
        //    if (g_hist.at<float>(i) > b_level)
        //        g_offs = i;
        //if (b_offs == 0)
        //    if (b_hist.at<float>(i) > b_level)
        //        b_offs = i;
    }

    m_offs = m_offs / (float)histSize;
    //b_offs = b_offs / (float)histSize;
    //g_offs = g_offs / (float)histSize;
    //r_offs = r_offs / (float)histSize;


    //printf("black level: %f %f %f\n", b_offs, g_offs, r_offs);
}





void plane_equation(float x1, float y1, float z1,
    float x2, float y2, float z2,
    float x3, float y3, float z3,
    float& a, float& b, float& c, float& d) {

    float a1 = x2 - x1;
    float b1 = y2 - y1;
    float c1 = z2 - z1;
    float a2 = x3 - x1;
    float b2 = y3 - y1;
    float c2 = z3 - z1;
    a = b1 * c2 - b2 * c1;
    b = a2 * c1 - a1 * c2;
    c = a1 * b2 - b1 * a2;
    d = (-a * x1 - b * y1 - c * z1);
    //printf("equation of plane is %.2f x + %.2f"
    //    " y + %.2f z + %.2f = 0.", a, b, c, d);
}


void black_level_gradient(Mat& image, float b_level) {

    if (debug_flag == 1) {
        cout << "Apply black level gradient correction" << endl;
        logfile << "Apply black level gradient correction" << endl;
    }

    // Black level correction with gradient, based on 4 corner pictures, histogram threshold from left - b_level

    float b_offs1, g_offs1, r_offs1; //offsets for left upper corner
    float b_offs2, g_offs2, r_offs2; //offsets for right upper corner
    float b_offs3, g_offs3, r_offs3; //offsets for right down corner
    float b_offs4, g_offs4, r_offs4; //offsets for left down corner

    Mat image2;
    //image.copyTo(image2);
    double resize_factor = 500.0 / image.rows;    //300.0 / image.rows;

    if (stack_from_file == 2)  // special dataset generation mode
        resize(sub_base_image, image2, Size(0, 0), resize_factor, resize_factor, INTER_AREA);
    else
        resize(image, image2, Size(0, 0), resize_factor, resize_factor, INTER_AREA);

    int sz = 3; //7;
    blur(image2, image2, Size(sz, sz));

    // apply circular mask for background compensation
    if (circular_mask_background_flag == 1) {
        //Prepare circular mask for background
        Mat circular_mask_b(image2.rows, image2.cols, CV_32FC3, Scalar(2, 2, 2));
        circle(circular_mask_b, Point(circular_mask_b.cols / 2, circular_mask_b.rows / 2), round(0.5 * circular_mask_b.rows * circular_mask_background_size), Scalar(0, 0, 0), FILLED, LINE_AA);

        image2 = max(image2, circular_mask_b);
    }
    //image2.copyTo(image);  // test

    int half_height = round(image2.rows / 10);  // dimensions of 1/5 tile
    int half_width = round(image2.cols / 10);

    //int row1 = round(image.rows / 10);     int col1 = round(image.cols / 10);   // points for plane equation, centers of corner 1/5 tiles
    //int row2 = round(image.rows / 10);     int col2 = round(image.cols / 10 * 9);
    //int row3 = round(image.rows / 10 * 9); int col3 = round(image.cols / 10 * 9);
    //int row4 = round(image.rows / 10 * 9); int col4 = round(image.cols / 10);
    int row1 = half_height;                     int col1 = half_width;
    int row2 = half_height;                     int col2 = image2.cols - 1 - half_width;
    int row3 = image2.rows - 1 - half_height;   int col3 = image2.cols - 1 - half_width;
    int row4 = image2.rows - 1 - half_height;   int col4 = half_width;
    //cout << row1 << " " << col1 << endl;

    //Mat crop1 = image2(Range(0, round(image2.rows / 5)), Range(0, round(image2.cols / 5)));   // left upper corner, 1/5 tile
    //Mat crop2 = image2(Range(0, round(image2.rows / 5)), Range(round(image2.cols / 5 * 4), image2.cols - 1));   // right upper corner, 1/5 tile
    //Mat crop3 = image2(Range(round(image2.rows / 5 * 4), image2.rows - 1), Range(round(image2.cols / 5 * 4), image2.cols - 1));   // right down corner, 1/5 tile
    //Mat crop4 = image2(Range(round(image2.rows / 5 * 4), image2.rows - 1), Range(0, round(image2.cols / 5)));   // left down corner, 1/5 tile

    if (circular_mask_background_flag == 1) {
        //Mat crop1 = image2(Range(0, round(image2.rows / 2)), Range(0, round(image2.cols / 2)));   // left upper corner, 1/2 tile
        //Mat crop2 = image2(Range(0, round(image2.rows / 2)), Range(round(image2.cols / 2), image2.cols - 1));   // right upper corner, 1/2 tile
        //Mat crop3 = image2(Range(round(image2.rows / 2), image2.rows - 1), Range(round(image2.cols / 2), image2.cols - 1));   // right down corner, 1/2 tile
        //Mat crop4 = image2(Range(round(image2.rows / 2), image2.rows - 1), Range(0, round(image2.cols / 2)));   // left down corner, 1/2 tile

        row1 = round(image2.rows / 2 - (circular_mask_background_size * image2.rows * 0.35));   // points for plane equation, boundary of circular mask
        col1 = round(image2.cols / 2 - (circular_mask_background_size * image2.rows * 0.35));

        row2 = round(image2.rows / 2 - (circular_mask_background_size * image2.rows * 0.35));
        col2 = round(image2.cols / 2 + (circular_mask_background_size * image2.rows * 0.35));

        row3 = round(image2.rows / 2 + (circular_mask_background_size * image2.rows * 0.35));
        col3 = round(image2.cols / 2 + (circular_mask_background_size * image2.rows * 0.35));

        row4 = round(image2.rows / 2 + (circular_mask_background_size * image2.rows * 0.35));
        col4 = round(image2.cols / 2 - (circular_mask_background_size * image2.rows * 0.35));

        if ((row1 - half_height) < 0) row1 = half_height;
        if ((col1 - half_width) < 0) col1 = half_width;
        if ((row2 - half_height) < 0) row2 = half_height;
        if ((col2 + half_width) > (image2.cols - 1)) col2 = image2.cols - 1 - half_width;
        if ((row3 + half_height) > (image2.rows - 1)) row3 = image2.rows - 1 - half_height;
        if ((col3 + half_width) > (image2.cols - 1)) col3 = image2.cols - 1 - half_width;
        if ((row4 + half_height) > (image2.rows - 1)) row4 = image2.rows - 1 - half_height;
        if ((col4 - half_width) < 0) col4 = half_width;
    }

    Mat crop1 = image2(Range(row1 - half_height, row1 + half_height), Range(col1 - half_width, col1 + half_width));   // left upper corner, center on circle, 1/5 tile
    Mat crop2 = image2(Range(row2 - half_height, row2 + half_height), Range(col2 - half_width, col2 + half_width));   // right upper corner, center on circle, 1/5 tile
    Mat crop3 = image2(Range(row3 - half_height, row3 + half_height), Range(col3 - half_width, col3 + half_width));   // right down corner, center on circle, 1/5 tile
    Mat crop4 = image2(Range(row4 - half_height, row4 + half_height), Range(col4 - half_width, col4 + half_width));   // left down corner, center on circle, 1/5 tile

    //imshow("corner", crop3 * 5);


    black_level_calculate(crop1, b_level, b_offs1, g_offs1, r_offs1);
    black_level_calculate(crop2, b_level, b_offs2, g_offs2, r_offs2);
    black_level_calculate(crop3, b_level, b_offs3, g_offs3, r_offs3);
    black_level_calculate(crop4, b_level, b_offs4, g_offs4, r_offs4);

    row1 = round((double)row1 / resize_factor);
    row2 = round((double)row2 / resize_factor);
    row3 = round((double)row3 / resize_factor);
    row4 = round((double)row4 / resize_factor);
    col1 = round((double)col1 / resize_factor);
    col2 = round((double)col2 / resize_factor);
    col3 = round((double)col3 / resize_factor);
    col4 = round((double)col4 / resize_factor);

    // recalculate points for plane equation, if circular mask for background
    /*
    if (circular_mask_background_flag == 1) {
        row1 = round(image.rows / 2 - (circular_mask_background_size * image.rows * 0.35));
        col1 = round(image.cols / 2 - (circular_mask_background_size * image.rows * 0.35));   // points for plane equation, boundary of circular mask

        row2 = round(image.rows / 2 - (circular_mask_background_size * image.rows * 0.35));
        col2 = round(image.cols / 2 + (circular_mask_background_size * image.rows * 0.35));

        row3 = round(image.rows / 2 + (circular_mask_background_size * image.rows * 0.35));
        col3 = round(image.cols / 2 + (circular_mask_background_size * image.rows * 0.35));

        row4 = round(image.rows / 2 + (circular_mask_background_size * image.rows * 0.35));
        col4 = round(image.cols / 2 - (circular_mask_background_size * image.rows * 0.35));
    }
    /**/
    //cout << row1 << " " << col1 << endl;

    float a1_b, b1_b, c1_b, d1_b;
    float a1_g, b1_g, c1_g, d1_g;
    float a1_r, b1_r, c1_r, d1_r;

    float a2_b, b2_b, c2_b, d2_b;
    float a2_g, b2_g, c2_g, d2_g;
    float a2_r, b2_r, c2_r, d2_r;


    plane_equation(row1, col1, b_offs1,
        row3, col3, b_offs3,
        row4, col4, b_offs4,
        a1_b, b1_b, c1_b, d1_b);

    plane_equation(row1, col1, g_offs1,
        row3, col3, g_offs3,
        row4, col4, g_offs4,
        a1_g, b1_g, c1_g, d1_g);

    plane_equation(row1, col1, r_offs1,
        row3, col3, r_offs3,
        row4, col4, r_offs4,
        a1_r, b1_r, c1_r, d1_r);




    plane_equation(row2, col2, b_offs2,
        row3, col3, b_offs3,
        row4, col4, b_offs4,
        a2_b, b2_b, c2_b, d2_b);

    plane_equation(row2, col2, g_offs2,
        row3, col3, g_offs3,
        row4, col4, g_offs4,
        a2_g, b2_g, c2_g, d2_g);

    plane_equation(row2, col2, r_offs2,
        row3, col3, r_offs3,
        row4, col4, r_offs4,
        a2_r, b2_r, c2_r, d2_r);




    //printf("equation of plane is %.2f x + %.2f"
    //    " y + %.2f z + %.2f = 0.", a1, b1, c1, d1);

    //float a_b = a1_b, b_b = b1_b, c_b = c1_b, d_b = d1_b;
    //float a_g = a1_g, b_g = b1_g, c_g = c1_g, d_g = d1_g;
    //float a_r = a1_r, b_r = b1_r, c_r = c1_r, d_r = d1_r;

    float a_b = (a1_b + a2_b) / 2, b_b = (b1_b + b2_b) / 2, c_b = (c1_b + c2_b) / 2, d_b = (d1_b + d2_b) / 2;
    float a_g = (a1_g + a2_g) / 2, b_g = (b1_g + b2_g) / 2, c_g = (c1_g + c2_g) / 2, d_g = (d1_g + d2_g) / 2;
    float a_r = (a1_r + a2_r) / 2, b_r = (b1_r + b2_r) / 2, c_r = (c1_r + c2_r) / 2, d_r = (d1_r + d2_r) / 2;

    //Mat plane_32fc3(image.rows, image.cols, CV_32FC3);
    //vector<Mat> bgr_planes;
    //split(plane_32fc3, bgr_planes);


    //Mat b_plane(image.rows, image.cols, CV_32FC1);
    //Mat g_plane(image.rows, image.cols, CV_32FC1);
    //Mat r_plane(image.rows, image.cols, CV_32FC1);
    Mat planes(image.rows, image.cols, CV_32FC3);

    /*
    for (int i = 0; i < image.rows; i++)
        for (int j = 0; j < image.cols; j++) {
            //bgr_planes[0].at<float>(i, j) = (-a_b * i - b_b * j - d_b) / c_b;
            //bgr_planes[1].at<float>(i, j) = (-a_g * i - b_g * j - d_g) / c_g;
            //bgr_planes[2].at<float>(i, j) = (-a_r * i - b_r * j - d_r) / c_r;

            b_plane.at<float>(i, j) = (-a_b * i - b_b * j - d_b) / c_b;
            g_plane.at<float>(i, j) = (-a_g * i - b_g * j - d_g) / c_g;
            r_plane.at<float>(i, j) = (-a_r * i - b_r * j - d_r) / c_r;
        }
    /**/

    if (planes.isContinuous()) // check, if gaps in memory
        //if (false)
    {
        // using point arithmetics

        float* p = (float*)planes.data;

        for (int i = 0; i < planes.rows; i++)
            for (int j = 0; j < planes.cols; j++) {
                *p = (-a_b * i - b_b * j - d_b) / c_b;  //B
                p++;
                *p = (-a_g * i - b_g * j - d_g) / c_g;  //G
                p++;
                *p = (-a_r * i - b_r * j - d_r) / c_r;  //R
                p++;
            }
    }
    else {
        // using iterators - safe, if gaps in memory
        MatIterator_<Vec3f> it, end;
        int i = 0, j = 0;
        for (it = planes.begin<Vec3f>(), end = planes.end<Vec3f>(); it != end; ++it)
        {
            (*it)[2] = (-a_r * i - b_r * j - d_r) / c_r;  //R
            (*it)[1] = (-a_g * i - b_g * j - d_g) / c_g;  //G
            (*it)[0] = (-a_b * i - b_b * j - d_b) / c_b;  //B
            j++;
            if (j == planes.cols) {
                j = 0;
                i++;
            }

        }
    }


    //imshow("plane", bgr_planes[2]*5);

    /*
    vector<Mat> image_bgr_planes;
    split(image, image_bgr_planes);

    //image_bgr_planes[0] = image_bgr_planes[0] - bgr_planes[0];
    //image_bgr_planes[1] = image_bgr_planes[1] - bgr_planes[1];
    //image_bgr_planes[2] = image_bgr_planes[2] - bgr_planes[2];

    image_bgr_planes[0] = image_bgr_planes[0] - b_plane;
    image_bgr_planes[1] = image_bgr_planes[1] - g_plane;
    image_bgr_planes[2] = image_bgr_planes[2] - r_plane;

    merge(image_bgr_planes, image);
    /**/



    //vector<Mat> bgr_planes;

    //bgr_planes.push_back(b_plane);
    //bgr_planes.push_back(g_plane);
    //bgr_planes.push_back(r_plane);

    //Mat planes;
    //merge(bgr_planes, planes);

    image = image - planes;
}




void black_level_gradient_mono(Mat& image, float b_level) {

    if (debug_flag == 1) {
        cout << "Apply black level gradient mono correction" << endl;
        logfile << "Apply black level gradient mono correction" << endl;
    }

    // Black level correction with gradient, based on 4 corner pictures, histogram threshold from left - b_level

    float m_offs1; //offsets for left upper corner
    float m_offs2; //offsets for right upper corner
    float m_offs3; //offsets for right down corner
    float m_offs4; //offsets for left down corner

    Mat image2;
    //image.copyTo(image2);
    double resize_factor = 500.0 / image.rows;    //300.0 / image.rows;

    if (stack_from_file == 2)  // special dataset generation mode
        resize(sub_base_image, image2, Size(0, 0), resize_factor, resize_factor, INTER_LINEAR);
    else
        resize(image, image2, Size(0, 0), resize_factor, resize_factor, INTER_LINEAR);

    int sz = 3; //7;
    blur(image2, image2, Size(sz, sz));

    // apply circular mask for background compensation
    if (circular_mask_background_flag == 1) {
        //Prepare circular mask for background
        Mat circular_mask_b(image2.rows, image2.cols, CV_32FC1, 2.0);
        circle(circular_mask_b, Point(circular_mask_b.cols / 2, circular_mask_b.rows / 2), round(0.5 * circular_mask_b.rows * circular_mask_background_size), 0.0, FILLED, LINE_AA);

        image2 = max(image2, circular_mask_b);
    }
    //image2.copyTo(image);  // test

    int half_height = round(image2.rows / 10);  // dimensions of 1/5 tile
    int half_width = round(image2.cols / 10);

    int row1 = half_height;                     int col1 = half_width;
    int row2 = half_height;                     int col2 = image2.cols - 1 - half_width;
    int row3 = image2.rows - 1 - half_height;   int col3 = image2.cols - 1 - half_width;
    int row4 = image2.rows - 1 - half_height;   int col4 = half_width;

    if (circular_mask_background_flag == 1) {

        row1 = round(image2.rows / 2 - (circular_mask_background_size * image2.rows * 0.35));   // points for plane equation, boundary of circular mask
        col1 = round(image2.cols / 2 - (circular_mask_background_size * image2.rows * 0.35));

        row2 = round(image2.rows / 2 - (circular_mask_background_size * image2.rows * 0.35));
        col2 = round(image2.cols / 2 + (circular_mask_background_size * image2.rows * 0.35));

        row3 = round(image2.rows / 2 + (circular_mask_background_size * image2.rows * 0.35));
        col3 = round(image2.cols / 2 + (circular_mask_background_size * image2.rows * 0.35));

        row4 = round(image2.rows / 2 + (circular_mask_background_size * image2.rows * 0.35));
        col4 = round(image2.cols / 2 - (circular_mask_background_size * image2.rows * 0.35));

        if ((row1 - half_height) < 0) row1 = half_height;
        if ((col1 - half_width) < 0) col1 = half_width;
        if ((row2 - half_height) < 0) row2 = half_height;
        if ((col2 + half_width) > (image2.cols - 1)) col2 = image2.cols - 1 - half_width;
        if ((row3 + half_height) > (image2.rows - 1)) row3 = image2.rows - 1 - half_height;
        if ((col3 + half_width) > (image2.cols - 1)) col3 = image2.cols - 1 - half_width;
        if ((row4 + half_height) > (image2.rows - 1)) row4 = image2.rows - 1 - half_height;
        if ((col4 - half_width) < 0) col4 = half_width;
    }

    Mat crop1 = image2(Range(row1 - half_height, row1 + half_height), Range(col1 - half_width, col1 + half_width));   // left upper corner, center on circle, 1/5 tile
    Mat crop2 = image2(Range(row2 - half_height, row2 + half_height), Range(col2 - half_width, col2 + half_width));   // right upper corner, center on circle, 1/5 tile
    Mat crop3 = image2(Range(row3 - half_height, row3 + half_height), Range(col3 - half_width, col3 + half_width));   // right down corner, center on circle, 1/5 tile
    Mat crop4 = image2(Range(row4 - half_height, row4 + half_height), Range(col4 - half_width, col4 + half_width));   // left down corner, center on circle, 1/5 tile

    black_level_calculate_mono(crop1, b_level, m_offs1);
    black_level_calculate_mono(crop2, b_level, m_offs2);
    black_level_calculate_mono(crop3, b_level, m_offs3);
    black_level_calculate_mono(crop4, b_level, m_offs4);

    row1 = round((double)row1 / resize_factor);
    row2 = round((double)row2 / resize_factor);
    row3 = round((double)row3 / resize_factor);
    row4 = round((double)row4 / resize_factor);
    col1 = round((double)col1 / resize_factor);
    col2 = round((double)col2 / resize_factor);
    col3 = round((double)col3 / resize_factor);
    col4 = round((double)col4 / resize_factor);

    float a1_m, b1_m, c1_m, d1_m;

    float a2_m, b2_m, c2_m, d2_m;


    plane_equation(row1, col1, m_offs1,
        row3, col3, m_offs3,
        row4, col4, m_offs4,
        a1_m, b1_m, c1_m, d1_m);


    plane_equation(row2, col2, m_offs2,
        row3, col3, m_offs3,
        row4, col4, m_offs4,
        a2_m, b2_m, c2_m, d2_m);


    float a_m = (a1_m + a2_m) / 2, b_m = (b1_m + b2_m) / 2, c_m = (c1_m + c2_m) / 2, d_m = (d1_m + d2_m) / 2;


    Mat plane(image.rows, image.cols, CV_32FC1);


    if (plane.isContinuous()) // check, if gaps in memory
        //if (false)
    {
        // using point arithmetics

        float* p = (float*)plane.data;

        for (int i = 0; i < plane.rows; i++)
            for (int j = 0; j < plane.cols; j++) {
                *p = (-a_m * i - b_m * j - d_m) / c_m;  //mono
                p++;
            }
    }
    else {
        // using iterators - safe, if gaps in memory
        MatIterator_<float> it, end;
        int i = 0, j = 0;
        for (it = plane.begin<float>(), end = plane.end<float>(); it != end; ++it)
        {
            (*it) = (-a_m * i - b_m * j - d_m) / c_m;  //mono
            j++;
            if (j == plane.cols) {
                j = 0;
                i++;
            }

        }
    }


    image = image - plane;
}