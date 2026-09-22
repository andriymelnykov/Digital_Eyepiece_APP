void highlight_protection(Mat& image, Mat& dark_image, float factor, float floor)
{
    Mat mask_for_dark, mask_for_orig, mask_for_obj;
    if (image.channels() == 3)
        cvtColor(image, mask_for_dark, cv::COLOR_BGR2GRAY);
    else
        mask_for_dark = image.clone();

    // cut low tones from mask
    if (floor > 0.01) {
        float scale = 1.0f / (1.0f - floor);
        mask_for_dark = (mask_for_dark - Scalar::all(floor)) * scale;
        // Clamp negatives to 0 (anything that was <= floor)
        max(mask_for_dark, Scalar::all(0.0f), mask_for_dark);
        // (optional) cap tiny round-off to exactly 1
        min(mask_for_dark, Scalar::all(1.0f), mask_for_dark);
    }

    // blur mask
    int blur_size = std::min(image.cols, image.rows);
    blur_size = blur_size / 300;
    if (blur_size < 4) blur_size = 4;
    if ((blur_size % 2) == 0) blur_size -= 1;
    medianBlur(mask_for_dark, mask_for_dark, blur_size);
    GaussianBlur(mask_for_dark, mask_for_dark, Size(blur_size, blur_size), 0);
    //imshow("mask", mask_for_dark);
    //waitKey(1);

    // detect big bright objects (additional mask)
    mask_for_dark.copyTo(mask_for_obj);
    resize(mask_for_obj, mask_for_obj, cv::Size(), 1.0 / 5.0, 1.0 / 5.0, cv::INTER_AREA);
    blur_size = std::min(mask_for_obj.cols, mask_for_obj.rows);
    blur_size = blur_size / 5;
    if (blur_size < 4) blur_size = 4;
    if ((blur_size % 2) == 0) blur_size -= 1;
    GaussianBlur(mask_for_obj, mask_for_obj, Size(blur_size, blur_size), 0);
    //mask_for_obj = (mask_for_obj - Scalar::all(0.2)) * 2.5;
    mask_for_obj = (mask_for_obj - Scalar::all(0.4)) * 2.5;
    max(mask_for_obj, Scalar::all(0.0f), mask_for_obj);
    min(mask_for_obj, Scalar::all(1.0f), mask_for_obj);
    resize(mask_for_obj, mask_for_obj, image.size(), 0, 0, cv::INTER_LINEAR);
    //imshow("bright object", mask_for_obj);
    //waitKey(1);

    // choose factor for big bright objects
    float factor_obj;
    if (factor < 0.2) factor_obj = factor;
    else if (factor < 0.7) factor_obj = 0.2;
    else factor_obj = (1 - factor) * 0.67;
    //cout << factor << "  " << factor_obj << endl;

    mask_for_dark = mask_for_dark * factor + mask_for_obj * factor_obj;
    //mask_for_dark = mask_for_dark * factor;

    if (image.channels() == 3)
        cvtColor(mask_for_dark, mask_for_dark, cv::COLOR_GRAY2BGR);

    mask_for_orig = Scalar::all(1) - mask_for_dark;

    multiply(dark_image, mask_for_dark, dark_image);

    multiply(image, mask_for_orig, image);

    image = image + dark_image;

    //imshow("mask", mask_for_dark);
    //waitKey(1);
}


void star_protection(Mat& image, Mat& star_image, Mat& star_linear, float factor, float floor)
{
    Mat mask_for_star, mask_for_orig, mask_for_obj, mask_for_obj2;
    Mat mask_for_star_soft, mask_for_orig_soft;
    Mat mask_for_star3, mask_for_orig3;
    Mat mask_for_star_soft3, mask_for_orig_soft3;
    Mat temp1, temp2;

    if (star_linear.channels() == 3)
        cvtColor(star_linear, mask_for_star, cv::COLOR_BGR2GRAY);
    else
        mask_for_star = star_linear.clone();


    // detect bright object
    mask_for_star.copyTo(mask_for_obj);
    resize(mask_for_obj, mask_for_obj, cv::Size(), 1.0 / 5.0, 1.0 / 5.0, cv::INTER_AREA);
    int blur_size = std::min(mask_for_obj.cols, mask_for_obj.rows);
    blur_size = blur_size / 5;
    if (blur_size < 4) blur_size = 4;
    if ((blur_size % 2) == 0) blur_size -= 1;
    GaussianBlur(mask_for_obj, mask_for_obj, Size(blur_size, blur_size), 0);
    mask_for_obj = (mask_for_obj - Scalar::all(0.02)) * 40;   //0.02
    max(mask_for_obj, Scalar::all(0.0f), mask_for_obj);
    min(mask_for_obj, Scalar::all(1.0f), mask_for_obj);
    resize(mask_for_obj, mask_for_obj, image.size(), 0, 0, cv::INTER_LINEAR);
    //imshow("bright object", mask_for_obj);
    //waitKey(1);


    // detect big bright objects (from stretched)
    if (image.channels() == 3)
        cvtColor(image, mask_for_obj2, cv::COLOR_BGR2GRAY);
    else
        mask_for_obj2 = image.clone();
    resize(mask_for_obj2, mask_for_obj2, cv::Size(), 1.0 / 5.0, 1.0 / 5.0, cv::INTER_AREA);
    blur_size = std::min(mask_for_obj2.cols, mask_for_obj2.rows);
    blur_size = blur_size / 5;
    if (blur_size < 4) blur_size = 4;
    if ((blur_size % 2) == 0) blur_size -= 1;
    GaussianBlur(mask_for_obj2, mask_for_obj2, Size(blur_size, blur_size), 0);
    mask_for_obj2 = (mask_for_obj2 - Scalar::all(0.3)) * 4;
    max(mask_for_obj2, Scalar::all(0.0f), mask_for_obj2);
    min(mask_for_obj2, Scalar::all(1.0f), mask_for_obj2);
    resize(mask_for_obj2, mask_for_obj2, image.size(), 0, 0, cv::INTER_LINEAR);
    //imshow("bright object2", mask_for_obj2);
    //waitKey(1);


    // sum mask of big bright object
    cv::max(mask_for_obj, mask_for_obj2, mask_for_obj);
    //imshow("bright object3", mask_for_obj2);
    //waitKey(1);


    // star mask hard
    //imshow("stars linear gray", mask_for_star);
    //waitKey(1);
    //mask_for_star = (mask_for_star - Scalar::all(floor)) * 200;
    mask_for_star = (mask_for_star - Scalar::all(floor)) * 20;
    max(mask_for_star, Scalar::all(0.0f), mask_for_star);
    min(mask_for_star, Scalar::all(1.0f), mask_for_star);
    int radius = std::min(mask_for_star.cols, mask_for_star.rows);
    radius = radius / 200; //300;
    if (radius < 2) radius = 2;
    Mat kernel = cv::getStructuringElement(MORPH_ELLIPSE, Size(2 * radius + 1, 2 * radius + 1));
    dilate(mask_for_star, mask_for_star, kernel);
    mask_for_star = mask_for_star - mask_for_obj;
    //imshow("star mask hard", mask_for_star);
    //waitKey(1);



    // star mask soft
    blur_size = std::min(image.cols, image.rows);
    blur_size = blur_size / 100;  //50;
    if (blur_size < 4) blur_size = 4;
    if ((blur_size % 2) == 0) blur_size -= 1;
    //Mat kernel2 = cv::getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
    //dilate(mask_for_star, mask_for_star_soft, kernel);
    //GaussianBlur(mask_for_star_soft, mask_for_star_soft, Size(blur_size, blur_size), 0);
    GaussianBlur(mask_for_star, mask_for_star_soft, Size(blur_size, blur_size), 0);
    mask_for_star_soft = (mask_for_star_soft - Scalar::all(0.1)) * 1.111;
    //imshow("star mask soft", mask_for_star_soft);
    //waitKey(1);



    // 3-channel masks
    cvtColor(mask_for_star, mask_for_star3, cv::COLOR_GRAY2BGR);
    mask_for_orig3 = Scalar::all(1) - mask_for_star3;
    cvtColor(mask_for_star_soft, mask_for_star_soft3, cv::COLOR_GRAY2BGR);
    mask_for_orig_soft3 = Scalar::all(1) - mask_for_star_soft3;



    // test star mask
    //multiply(image, mask_for_orig3, image);
    //imshow("image", image);
    //waitKey(1);



    // create starless strong stretch image
    Mat starlessStrong;
    Mat mask8u;
    //threshold(mask_for_star, mask8u, 0.1, 255.0, THRESH_BINARY);
    threshold(mask_for_star, mask8u, 0.03, 255.0, THRESH_BINARY);
    mask8u.convertTo(mask8u, CV_8UC1);
    vector<Mat> bgr_planes;
    split(image, bgr_planes);
    inpaint(bgr_planes[2], mask8u, bgr_planes[2], 10, INPAINT_NS);
    inpaint(bgr_planes[1], mask8u, bgr_planes[1], 10, INPAINT_NS);
    inpaint(bgr_planes[0], mask8u, bgr_planes[0], 10, INPAINT_NS);
    merge(bgr_planes, starlessStrong);
    multiply(starlessStrong, mask_for_star_soft3, temp1);
    multiply(image, mask_for_orig_soft3, temp2);
    starlessStrong = temp1 + temp2;
    //imshow("starlessStrong", starlessStrong);
    //waitKey(1);




    // create starless mild stretch image
    Mat starlessMild;
    split(star_image, bgr_planes);
    inpaint(bgr_planes[2], mask8u, bgr_planes[2], 10, INPAINT_NS);
    inpaint(bgr_planes[1], mask8u, bgr_planes[1], 10, INPAINT_NS);
    inpaint(bgr_planes[0], mask8u, bgr_planes[0], 10, INPAINT_NS);
    merge(bgr_planes, starlessMild);
    multiply(starlessMild, mask_for_star_soft3, temp1);
    multiply(star_image, mask_for_orig_soft3, temp2);
    starlessMild = temp1 + temp2;
    //imshow("starlessMild", starlessMild);
    //waitKey(1);



    // create isolated mild stretch stars
    Mat starsMild;
    starsMild = star_image - starlessMild;
    //imshow("starsMild", starsMild);
    //waitKey(1);


    image = starlessStrong + starsMild;
}

void highlight_protection2(Mat& image, Mat& dark_image, float factor, float floor)
{

    Mat mask_for_dark, mask_for_orig;
    cvtColor(dark_image, mask_for_dark, cv::COLOR_BGR2GRAY);

    if (floor > 0.01) {
        //float scale = 1.0f / (1.0f - floor);
        mask_for_dark = (mask_for_dark - Scalar::all(floor)) * 200;  //* scale;   // fast, uses SIMD under the hood

        // Clamp negatives to 0 (anything that was <= floor)
        max(mask_for_dark, Scalar::all(0.0f), mask_for_dark);

        // (optional) cap tiny round-off to exactly 1
        min(mask_for_dark, Scalar::all(1.0f), mask_for_dark);
    }

    int radius = std::min(image.cols, image.rows);
    radius = radius / 500;
    if (radius < 2) radius = 2;
    Mat kernel = cv::getStructuringElement(MORPH_ELLIPSE, Size(2 * radius + 1, 2 * radius + 1));
    dilate(mask_for_dark, mask_for_dark, kernel);

    int blur_size = std::min(image.cols, image.rows);
    blur_size = blur_size / 100;
    if (blur_size < 4) blur_size = 4;
    if ((blur_size % 2) == 0) blur_size -= 1;

    //medianBlur(mask_for_dark, mask_for_dark, blur_size);
    GaussianBlur(mask_for_dark, mask_for_dark, Size(blur_size, blur_size), 0);

    mask_for_dark = mask_for_dark * factor;

    cvtColor(mask_for_dark, mask_for_dark, cv::COLOR_GRAY2BGR);



    //mask_for_dark.copyTo(test_image);



    mask_for_orig = Scalar::all(1) - mask_for_dark;

    multiply(dark_image, mask_for_dark, dark_image);

    multiply(image, mask_for_orig, image);

    image = image + dark_image;

    //imshow("mask higlights 2", mask_for_dark);
    //waitKey(1);
}