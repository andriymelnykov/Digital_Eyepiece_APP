void enhance_stars(Mat& stack_image, Mat& final_image, float star_blob_radius, float star_blob_strength)
{

    if (debug_flag == 1) {
        cout << "Enhance stars" << endl;
        logfile << "Enhance stars" << endl;
    }

    Mat stars_image;

    // OpenCL variant
    /**/

    UMat u_stars_image;
    stack_image.copyTo(u_stars_image);
    multiply(u_stars_image, u_stars_image, u_stars_image);
    GaussianBlur(u_stars_image, u_stars_image, Size(star_blob_radius * 2 + 1, star_blob_radius * 2 + 1), 0);
    //normalize(u_stars_image, u_stars_image, 0.0, star_blob_strength, NORM_MINMAX);
    u_stars_image.copyTo(stars_image);
    double minVal, maxVal;
    minMaxLoc(stars_image, &minVal, &maxVal);
    float scale = star_blob_strength / static_cast<float>(maxVal);
    stars_image *= scale;

    final_image = final_image + stars_image;
    final_image = final_image / (1 + star_blob_strength);

    //imshow("stars", stars_image * 10.0);

    for (int i = 0; i < final_image.rows; i++)
        for (int j = 0; j < final_image.cols; j++) {
            float thr = 0.8;
            if ((final_image.at<Vec3f>(i, j)[0] > thr) && (final_image.at<Vec3f>(i, j)[1] > thr) && (final_image.at<Vec3f>(i, j)[2] > thr)) {
                float max_ch = std::max({ stars_image.at<Vec3f>(i, j)[0], stars_image.at<Vec3f>(i, j)[1], stars_image.at<Vec3f>(i, j)[2] });

                final_image.at<Vec3f>(i, j)[0] = final_image.at<Vec3f>(i, j)[0] * stars_image.at<Vec3f>(i, j)[0] / max_ch;
                final_image.at<Vec3f>(i, j)[1] = final_image.at<Vec3f>(i, j)[1] * stars_image.at<Vec3f>(i, j)[1] / max_ch;
                final_image.at<Vec3f>(i, j)[2] = final_image.at<Vec3f>(i, j)[2] * stars_image.at<Vec3f>(i, j)[2] / max_ch;
                ;
            }

        }

    //imshow("stars", final_image);


    /**/

    // CPU variant
    /*
    stack_image.copyTo(stars_image);
    multiply(stars_image, stars_image, stars_image);

    GaussianBlur(stars_image, stars_image, Size(star_blob_radius * 2 + 1, star_blob_radius * 2 + 1), 0);

    normalize(stars_image, stars_image, 0.0, star_blob_strength, NORM_MINMAX);
    final_image = final_image + stars_image;
    /**/

    // draft difraction spikes variant
    /*
    int N = star_blob_radius * 2 + 1; // Kernel size
    Mat kernel(N, N, CV_32FC3, cv::Scalar(0, 0, 0)); // Initialize kernel as a 3-channel float matrix

    int centerX = N / 2;
    int centerY = N / 2;

    // Set values for the four perpendicular lines
    for (int i = 0; i < N; ++i) {
        // Horizontal line
        kernel.at<cv::Vec3f>(N*2/3 -i/3, i) = cv::Vec3f(1.0, 1.0, 1.0) * (1.0 - static_cast<float>(abs(i - centerX)) / (N / 2));

        // Vertical line
        kernel.at<cv::Vec3f>(i, N/3 + i/3) = cv::Vec3f(1.0, 1.0, 1.0) * (1.0 - static_cast<float>(abs(i - centerY)) / (N / 2));
    }

    imshow("kernel", kernel);

    // Apply the filter using cv::filter2D
    cv::filter2D(stars_image, stars_image, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    /**/


}