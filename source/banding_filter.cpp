// Function to apply moving average filter on a one-dimensional array
void movingAverage(const Mat& input, Mat& output, int windowSize) {
    Mat kernel = Mat::ones(windowSize, 1, CV_32F) / (float)windowSize;
    filter2D(input, output, -1, kernel, cv::Point(-1, -1), 0, BORDER_DEFAULT);
}

void movingMedian(const Mat& input, Mat& output, int windowSize) {
    int halfWindow = windowSize / 2;
    output = Mat::zeros(input.size(), input.type());

    for (int i = 0; i < input.rows; ++i) {
        std::vector<float> window;
        for (int j = -halfWindow; j <= halfWindow; ++j) {
            int idx = std::max(0, std::min(i + j, input.rows - 1));
            window.push_back(input.at<float>(idx, 0));
        }

        std::nth_element(window.begin(), window.begin() + window.size() / 2, window.end());
        output.at<float>(i, 0) = window[window.size() / 2];
    }
}

void banding_filter(Mat& image, int flag, int strength, float threshold) {

    float pixelValue;

    if (debug_flag == 1) {
        cout << "Apply banding filter" << endl;
        logfile << "Apply banding filter" << endl;
    }

    //vector<Mat> bgr_planes;
    //split(image, bgr_planes);

    Scalar m = mean(image);
    //cout << m[0] << " " << m[1] << " " << m[2];


    if (flag == 1) {
        // Step 1: Calculate sum of each row
        Mat rowSums = Mat::zeros(image.rows, 1, CV_32F);

        for (int i = 0; i < image.rows; ++i) {
            int N = 0;
            for (int j = 0; j < image.cols; ++j) {
                if (image.channels() == 3) {  // BGR path
                    pixelValue = image.at<Vec3f>(i, j)[1];          // only green here!

                    if (pixelValue < (m[1] * threshold)) {     // image mean value as threshold to exclude stars
                        rowSums.at<float>(i, 0) += pixelValue;
                        N++;
                    }
                }
                else {       // mono path
                    pixelValue = image.at<float>(i, j);

                    if (pixelValue < (m[0] * threshold)) {     // image mean value as threshold to exclude stars
                        rowSums.at<float>(i, 0) += pixelValue;
                        N++;
                    }
                }

                //if (pixelValue < (m[1]*threshold)) {     // image mean value as threshold to exclude stars
                //    rowSums.at<float>(i, 0) += pixelValue;
                //    N++;
                //}
            }
            rowSums.at<float>(i, 0) = rowSums.at<float>(i, 0) / N;
            //cout << N << endl;
            //cout << rowSums.at<float>(i, 0) << endl;
        }

        // Step 2: Apply moving average filter
        Mat filteredRowSums;
        int windowSize = strength; // Define your window size here
        movingAverage(rowSums, filteredRowSums, windowSize);
        //movingMedian(rowSums, filteredRowSums, windowSize);

        // Step 3: Scale original image rows
        for (int i = 0; i < image.rows; ++i) {
            float scale = filteredRowSums.at<float>(i, 0) / rowSums.at<float>(i, 0);
            image.row(i) *= scale;
        }
    }
    if (flag == 2) {
        // Step 1: Calculate sum of each column
        Mat colSums = Mat::zeros(image.cols, 1, CV_32F);

        for (int i = 0; i < image.cols; ++i) {
            int N = 0;
            for (int j = 0; j < image.rows; ++j) {
                if (image.channels() == 3) {  // BGR path
                    pixelValue = image.at<Vec3f>(j, i)[1];          // only green here!

                    if (pixelValue < (m[1] * threshold)) {     // image mean value as threshold to exclude stars
                        colSums.at<float>(i, 0) += pixelValue;
                        N++;
                    }
                }
                else {      // mono path
                    pixelValue = image.at<float>(j, i);

                    if (pixelValue < (m[0] * threshold)) {     // image mean value as threshold to exclude stars
                        colSums.at<float>(i, 0) += pixelValue;
                        N++;
                    }
                }

                //if (pixelValue < (m[1] * threshold)) {                          // threshold here!
                //    colSums.at<float>(i, 0) += pixelValue;
                //    N++;
                //}
            }
            colSums.at<float>(i, 0) = colSums.at<float>(i, 0) / N;
            //cout << N << endl;
            //cout << colSums.at<float>(i, 0) << endl;
        }

        // Step 2: Apply moving average filter
        Mat filteredColSums;
        int windowSize = strength; // Define your window size here
        movingAverage(colSums, filteredColSums, windowSize);

        // Step 3: Scale original image rows
        for (int i = 0; i < image.cols; ++i) {
            float scale = filteredColSums.at<float>(i, 0) / colSums.at<float>(i, 0);
            image.col(i) *= scale;
        }
    }

}