Mat NN_noise_reduction_tile(const fdeep::model& model, const Mat& image, const Mat& window2d) {

    // Split RGB parts of a tile
    //vector<Mat> image_bgr;
    //split(image, image_bgr);

    //Apply window
    //multiply(image_bgr[0], window2d, image_bgr[0]);
    //multiply(image_bgr[1], window2d, image_bgr[1]);
    //multiply(image_bgr[2], window2d, image_bgr[2]);

    //imshow("before", image_bgr[1]);

    /*
    for (int i = 0; i < 3; i++) {
        // convert cv::Mat to fdeep::tensor (image tile to tensor)--------------
        // Prepare the data vector
        std::vector<float> img_data;
        img_data.reserve(image.rows * image.cols);

        // Convert cv::Mat to a vector of floats
        img_data.assign(image_bgr[i].begin<float>(), image_bgr[i].end<float>());

        // Create the tensor
        const fdeep::tensor tensor = fdeep::tensor(
            fdeep::tensor_shape(image.rows, image.cols, 1),
            img_data
        );
        // -----------------------------------------

        // calculate NN response
        const auto result = model.predict({ tensor });

        // convert fdeep::tensor to cv::Mat (tensor to image tile)--------------
        // convert fdeep::tensor to float cv::Mat
        const cv::Mat image_out(cv::Size(result.front().shape().width_, result.front().shape().height_), CV_32F);
        const auto values = result.front().to_vector();
        std::memcpy(image_out.data, values.data(), values.size() * sizeof(float));
        // -----------------------------------------
        image_out.copyTo(image_bgr[i]);
    }
    //merge(image_bgr, image);
    /**/

    cvtColor(image, image, cv::COLOR_BGR2RGB);

    // convert cv::Mat to fdeep::tensor (image tile to tensor)--------------
        // Prepare the data vector
    std::vector<float> img_data;
    img_data.reserve(image.rows * image.cols * 3);
    img_data.resize(image.rows * image.cols * 3);

    // Copy cv::Mat to a vector of floats
    //img_data.assign(image.begin<Vec3f>()[0], image.end<Vec3f>()[2]);

    std::memcpy(img_data.data(), (float*)image.data, (image.rows * image.cols * 3) * sizeof(float));

    // Create the tensor
    const fdeep::tensor tensor = fdeep::tensor(
        fdeep::tensor_shape(image.rows, image.cols, 3),
        img_data
    );
    // -----------------------------------------

    // calculate NN response
    const auto result = model.predict({ tensor });

    // convert fdeep::tensor to cv::Mat (tensor to image tile)--------------
    // convert fdeep::tensor to float cv::Mat
    //const Mat image_out(Size(result.front().shape().width_, result.front().shape().height_), CV_32FC3);
    const auto values = result.front().to_vector();
    //std::memcpy(image_out.data, values.data(), values.size() * sizeof(float));
    std::memcpy(image.data, values.data(), values.size() * sizeof(float));
    // -----------------------------------------
    //image_out.copyTo(image);

    cvtColor(image, image, cv::COLOR_RGB2BGR);

    multiply(image, window2d, image);

    return image;
}

Mat NN_noise_reduction_tile_mono(const fdeep::model& model, const Mat& image, const Mat& window2d) {

    // convert cv::Mat to fdeep::tensor (image tile to tensor)--------------
        // Prepare the data vector
    std::vector<float> img_data;
    img_data.reserve(image.rows * image.cols);
    img_data.resize(image.rows * image.cols);

    // Copy cv::Mat to a vector of floats

    //cout << "test tile...1" << endl;

    std::memcpy(img_data.data(), (float*)image.data, (image.rows * image.cols) * sizeof(float));

    //cout << "test tile...2" << endl;

    // Create the tensor
    const fdeep::tensor tensor = fdeep::tensor(
        fdeep::tensor_shape(image.rows, image.cols, 1),
        img_data
    );

    //cout << "test tile...3" << endl;
    // -----------------------------------------

    // calculate NN response
    const auto result = model.predict({ tensor });

    //cout << "test tile...4" << endl;

    // convert fdeep::tensor to cv::Mat (tensor to image tile)--------------
    // convert fdeep::tensor to float cv::Mat
    const auto values = result.front().to_vector();
    std::memcpy(image.data, values.data(), values.size() * sizeof(float));
    // -----------------------------------------

    //cout << "test tile...4" << endl;

    multiply(image, window2d, image);

    return image;
}



void NN_noise_reduction(const fdeep::model& model, Mat& image, double mix) {

    // prepeare window
    /**/
    int N = 128;   //64; // Set the desired size for the window
    Mat window1d(N, 1, CV_32F); // Create a 1D window matrix
    // Populate the 1D window using the Hanning window function
    /*
    for (int i = 0; i < N; i++) {
        float val = 0.5 * (1 - cos(2 * CV_PI * i / (N - 1)));
        //cout << val << endl;
        window1d.at<float>(i, 0) = val;
    }
    /**/
    // Populate 1D window, overlap 3
    /*
    for (int i = 0; i < N; i++) {
        if (i == 0) window1d.at<float>(i, 0) = 0;
        else if (i == 1) window1d.at<float>(i, 0) = 0.5;
        else if (i == (N-2)) window1d.at<float>(i, 0) = 0.5;
        else if (i == (N-1)) window1d.at<float>(i, 0) = 0;
        else window1d.at<float>(i, 0) = 1.0;

    }
    /**/
    // Populate 1D window, overlap 5
    /*
    for (int i = 0; i < N; i++) {
        if (i == 0) window1d.at<float>(i, 0) = 0;
        else if (i == 1) window1d.at<float>(i, 0) = 0.1;
        else if (i == 2) window1d.at<float>(i, 0) = 0.5;
        else if (i == 3) window1d.at<float>(i, 0) = 0.9;
        else if (i == (N - 4)) window1d.at<float>(i, 0) = 0.9;
        else if (i == (N - 3)) window1d.at<float>(i, 0) = 0.5;
        else if (i == (N - 2)) window1d.at<float>(i, 0) = 0.1;
        else if (i == (N - 1)) window1d.at<float>(i, 0) = 0;
        else window1d.at<float>(i, 0) = 1.0;

    }
    /**/
    // Populate 1D window, overlap 7
    /**
    for (int i = 0; i < N; i++) {
        if      (i == 0) window1d.at<float>(i, 0) = 0;
        else if (i == 1) window1d.at<float>(i, 0) = 0;
        else if (i == 2) window1d.at<float>(i, 0) = 0.1;
        else if (i == 3) window1d.at<float>(i, 0) = 0.5;
        else if (i == 4) window1d.at<float>(i, 0) = 0.9;
        else if (i == 5) window1d.at<float>(i, 0) = 1.0;
        else if (i == (N - 6)) window1d.at<float>(i, 0) = 1.0;
        else if (i == (N - 5)) window1d.at<float>(i, 0) = 0.9;
        else if (i == (N - 4)) window1d.at<float>(i, 0) = 0.5;
        else if (i == (N - 3)) window1d.at<float>(i, 0) = 0.1;
        else if (i == (N - 2)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 1)) window1d.at<float>(i, 0) = 0;
        else window1d.at<float>(i, 0) = 1.0;

    } /**/

    // Populate 1D window, overlap 10
    /*
    for (int i = 0; i < N; i++) {
        if      (i == 0) window1d.at<float>(i, 0) = 0;
        else if (i == 1) window1d.at<float>(i, 0) = 0;
        else if (i == 2) window1d.at<float>(i, 0) = 0;
        else if (i == 3) window1d.at<float>(i, 0) = 0.1;
        else if (i == 4) window1d.at<float>(i, 0) = 0.2;
        else if (i == 5) window1d.at<float>(i, 0) = 0.8;
        else if (i == 6) window1d.at<float>(i, 0) = 0.9;
        else if (i == 7) window1d.at<float>(i, 0) = 1.0;
        else if (i == (N - 8)) window1d.at<float>(i, 0) = 1.0;
        else if (i == (N - 7)) window1d.at<float>(i, 0) = 0.9;
        else if (i == (N - 6)) window1d.at<float>(i, 0) = 0.8;
        else if (i == (N - 5)) window1d.at<float>(i, 0) = 0.2;
        else if (i == (N - 4)) window1d.at<float>(i, 0) = 0.1;
        else if (i == (N - 3)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 2)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 1)) window1d.at<float>(i, 0) = 0;
        else window1d.at<float>(i, 0) = 1.0;

    } /**/

    // Populate 1D window, overlap 16
    /**/
    for (int i = 0; i < N; i++) {
        if (i == 0) window1d.at<float>(i, 0) = 0;
        else if (i == 1) window1d.at<float>(i, 0) = 0;
        else if (i == 2) window1d.at<float>(i, 0) = 0;
        else if (i == 3) window1d.at<float>(i, 0) = 0;
        else if (i == 4) window1d.at<float>(i, 0) = 0;
        else if (i == 5) window1d.at<float>(i, 0) = 0;
        else if (i == 6) window1d.at<float>(i, 0) = 0;
        else if (i == 7) window1d.at<float>(i, 0) = 0.2;
        else if (i == 8) window1d.at<float>(i, 0) = 0.8;
        else if (i == 9) window1d.at<float>(i, 0) = 1.0;
        else if (i == (N - 10)) window1d.at<float>(i, 0) = 1.0;
        else if (i == (N - 9)) window1d.at<float>(i, 0) = 0.8;
        else if (i == (N - 8)) window1d.at<float>(i, 0) = 0.2;
        else if (i == (N - 7)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 6)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 5)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 4)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 3)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 2)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 1)) window1d.at<float>(i, 0) = 0;
        else window1d.at<float>(i, 0) = 1.0;

    } /**/

    //Hanning window
    /*
    const float two_pi_over_Nm1 = 2.0f * static_cast<float>(CV_PI) / (N - 1);
    for (int i = 0; i < N; ++i)
    {
        float hann = 0.5f * (1.0f - std::cos(two_pi_over_Nm1 * i));  // Hann
        window1d.at<float>(i, 0) = hann;
    }   /**/


    // Create a 2D window by taking the outer product of the 1D window with itself
    Mat window2d = window1d * window1d.t();
    /*
    cout << window2d.at<float>(0, 0) << endl;
    cout << window2d.at<float>(1, 0) << endl;
    cout << window2d.at<float>(2, 0) << endl;
    cout << window2d.at<float>(0, 1) << endl;
    cout << window2d.at<float>(1, 1) << endl;
    cout << window2d.at<float>(2, 1) << endl;
    cout << window2d.at<float>(0, 2) << endl;
    cout << window2d.at<float>(1, 2) << endl;
    cout << window2d.at<float>(2, 2) << endl;
    /**/
    //Mat window2d_gray = window2d;
    cvtColor(window2d, window2d, cv::COLOR_GRAY2BGR);
    /**/

    //Mat mat16uc3;
    //window2d.convertTo(mat16uc3, CV_16UC3, 65535);
    //imwrite("C:/Users/HOME/Desktop/stacks_test/window.tiff", mat16uc3);
    //imshow("window", window2d);

    /*
    Mat image1;
    image.copyTo(image1);
    Mat cropped_image;
    int y = 350;
    int x = 350;
    cropped_image = image1(Range(y, y+32), Range(x, x+32));

    //imshow("before", cropped_image);

    NN_noise_reduction_tile(model, cropped_image, window2d);

    //imshow("after", cropped_image);
    /**/


    /**/
    //cout << "start NN filter..." << endl;
    if (debug_flag == 1) {
        //cout << "start NN filter..." << endl;
        logfile << "start NN filter..." << endl;
    }

    std::vector<std::thread> threads;
    int maxThreads = std::thread::hardware_concurrency(); // Number of available hardware threads
    if (maxThreads == 0) maxThreads = 4;
    if (AI_num_threads > 0) maxThreads = AI_num_threads;

    //cout << "used number of threads: " << maxThreads << endl;

    //int tileSize = 64;
    int tileSize = N;  // see window definition
    //int overlap = 7;
    int overlap = 16;
    //int overlap = 32;
    int step = tileSize - overlap;


    Mat bigImage;
    image.copyTo(bigImage);
    //copyMakeBorder(image, bigImage, 0, tileSize, 0, tileSize, BORDER_CONSTANT, Scalar(0, 0, 0));
    copyMakeBorder(image, bigImage, tileSize - overlap, tileSize - overlap, tileSize - overlap, tileSize - overlap, BORDER_REPLICATE, Scalar(0, 0, 0));

    //define black image for output
    Mat bigImage_out(bigImage.rows, bigImage.cols, CV_32FC3, Scalar(0, 0, 0));
    //Mat bigImage_out; bigImage.copyTo(bigImage_out);
    //Mat weight_sum(bigImage.rows, bigImage.cols, CV_32FC3, Scalar::all(0));


    std::vector<std::future<cv::Mat>> futures;


    //for (int y = 350; y < 351; y += step) {  //process only one tile for test
    //    for (int x = 350; x < 351; x += step) {
    //for (int y = 500; y < 1000; y += step) {
    //    for (int x = 500; x < 1000; x += step) {
    for (int y = 0; y < (bigImage.rows - tileSize); y += step) {
        for (int x = 0; x < (bigImage.cols - tileSize); x += step) {
            //for (int y = 0; y < (bigImage.rows); y += step) {
            //    for (int x = 0; x < (bigImage.cols); x += step) {

                    //cout << y << " " << x << endl;

                    // Define tile region with overlap
            int width = tileSize;
            int height = tileSize;
            cv::Rect tileRegion(x, y, width, height);

            // Extract tile
            Mat tile = bigImage(tileRegion).clone();


            //--//imshow("before", tile2);

            // Filter tile

            //Mat tile2 = NN_noise_reduction_tile(model, tile, window2d);


            //--//imshow("after", tile2);

            //Apply window
            //--//multiply(tile2, window2d, tile2);

            // Blend or place filtered tile back into big image
            //add(tile2, bigImage_out(tileRegion), tile2);
            //tile2.copyTo(bigImage_out(tileRegion));

            futures.push_back(std::async(std::launch::async, NN_noise_reduction_tile, model, tile, window2d));
        }
        //cout << y << endl;
    }


    int index = 0;
    for (int y = 0; y < (bigImage.rows - tileSize); y += step) {
        for (int x = 0; x < (bigImage.cols - tileSize); x += step) {

            // Define tile region with overlap
            int width = tileSize;
            int height = tileSize;
            cv::Rect tileRegion(x, y, width, height);

            Mat tile2 = futures[index++].get(); // Get the result from future and place in the corresponding region
            //add(tile2, bigImage_out(tileRegion), tile2);
            //tile2.copyTo(bigImage_out(tileRegion));

            add(tile2, bigImage_out(tileRegion), bigImage_out(tileRegion));          // accumulate colour
            //add(window2d, weight_sum(tileRegion), weight_sum(tileRegion));     // accumulate weights
        }
    }

    //Mat mat8uc3;
    //weight_sum.convertTo(mat8uc3, CV_8UC3, 250);
    //imwrite("weights.tiff", mat8uc3);



    int width = image.cols;
    int height = image.rows;
    //Rect cropRegion(0, 0, width, height);
    Rect cropRegion(tileSize - overlap, tileSize - overlap, width, height);
    if (mix > 0.99)
        image = bigImage_out(cropRegion);
    else
        image = image * (1.0 - mix) + bigImage_out(cropRegion) * mix;

    //bigImage_out.copyTo(image);

    //cout << "end NN filter" << endl;
    if (debug_flag == 1) {
        //cout << "end NN filter..." << endl;
        logfile << "end NN filter..." << endl;
    }


    /**/
}

void NN_noise_reduction_mono(const fdeep::model& model, Mat& image, double mix) {

    // prepeare window
    int N = 128;  // Set the desired size for the window
    Mat window1d(N, 1, CV_32F); // Create a 1D window matrix

    // Populate 1D window, overlap 16
    for (int i = 0; i < N; i++) {
        if (i == 0) window1d.at<float>(i, 0) = 0;
        else if (i == 1) window1d.at<float>(i, 0) = 0;
        else if (i == 2) window1d.at<float>(i, 0) = 0;
        else if (i == 3) window1d.at<float>(i, 0) = 0;
        else if (i == 4) window1d.at<float>(i, 0) = 0;
        else if (i == 5) window1d.at<float>(i, 0) = 0;
        else if (i == 6) window1d.at<float>(i, 0) = 0;
        else if (i == 7) window1d.at<float>(i, 0) = 0.2;
        else if (i == 8) window1d.at<float>(i, 0) = 0.8;
        else if (i == 9) window1d.at<float>(i, 0) = 1.0;
        else if (i == (N - 10)) window1d.at<float>(i, 0) = 1.0;
        else if (i == (N - 9)) window1d.at<float>(i, 0) = 0.8;
        else if (i == (N - 8)) window1d.at<float>(i, 0) = 0.2;
        else if (i == (N - 7)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 6)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 5)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 4)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 3)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 2)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 1)) window1d.at<float>(i, 0) = 0;
        else window1d.at<float>(i, 0) = 1.0;

    }

    // Create a 2D window by taking the outer product of the 1D window with itself
    Mat window2d = window1d * window1d.t();

    //cout << "start NN filter mono..." << endl;
    if (debug_flag == 1) {
        //cout << "start NN filter mono..." << endl;
        logfile << "start NN filter mono..." << endl;
    }

    std::vector<std::thread> threads;
    int maxThreads = std::thread::hardware_concurrency(); // Number of available hardware threads
    if (maxThreads == 0) maxThreads = 4;
    if (AI_num_threads > 0) maxThreads = AI_num_threads;

    //cout << "used number of threads: " << maxThreads << endl;








    int tileSize = N;  // see window definition
    int overlap = 16;
    int step = tileSize - overlap;


    Mat bigImage;
    image.copyTo(bigImage);
    //copyMakeBorder(image, bigImage, 0, tileSize, 0, tileSize, BORDER_CONSTANT, Scalar(0, 0, 0));
    //copyMakeBorder(image, bigImage, tileSize - overlap, tileSize - overlap, tileSize - overlap, tileSize - overlap, BORDER_REPLICATE, Scalar::all(0));
    copyMakeBorder(image, bigImage, overlap, tileSize - overlap, overlap, tileSize - overlap, BORDER_REPLICATE, Scalar::all(0));

    //define black image for output
    Mat bigImage_out(bigImage.rows, bigImage.cols, CV_32FC1, Scalar::all(0));

    //cout << "test..." << endl;



    /**/
    std::vector<std::future<cv::Mat>> futures;

    for (int y = 0; y < (bigImage.rows - tileSize); y += step) {
        for (int x = 0; x < (bigImage.cols - tileSize); x += step) {

            // Define tile region with overlap
            int width = tileSize;
            int height = tileSize;
            cv::Rect tileRegion(x, y, width, height);

            // Extract tile
            Mat tile = bigImage(tileRegion).clone();


            //--//imshow("before", tile2);

            // Filter tile

            //Mat tile2 = NN_noise_reduction_tile(model, tile, window2d);


            //--//imshow("after", tile2);

            //Apply window
            //--//multiply(tile2, window2d, tile2);

            // Blend or place filtered tile back into big image
            //add(tile2, bigImage_out(tileRegion), tile2);
            //tile2.copyTo(bigImage_out(tileRegion));

            //cout << "test..." << endl;

          //futures.push_back(std::async(std::launch::async, NN_noise_reduction_tile_mono, model, tile, window2d));
            futures.push_back(std::async(std::launch::async, NN_noise_reduction_tile_mono, std::cref(model), tile, std::cref(window2d)));

        }
        //cout << y << endl;
    }

    //cout << "test..." << endl;

    int index = 0;
    for (int y = 0; y < (bigImage.rows - tileSize); y += step) {
        for (int x = 0; x < (bigImage.cols - tileSize); x += step) {

            // Define tile region with overlap
            int width = tileSize;
            int height = tileSize;
            cv::Rect tileRegion(x, y, width, height);

            //cout << "test..." << endl;

            Mat tile2 = futures[index++].get(); // Get the result from future and place in the corresponding region

            //cout << "test..." << endl;

            //imshow("tile", tile2);
            //key = waitKey(2000);


            add(tile2, bigImage_out(tileRegion), bigImage_out(tileRegion));          // accumulate colour
            //add(window2d, weight_sum(tileRegion), weight_sum(tileRegion));     // accumulate weights

            //cout << "test..." << endl;
        }
    }/**/



    /*
#include <deque>

    struct Inflight {
        cv::Rect roi;
        std::future<cv::Mat> fut;
    };

    std::deque<Inflight> inflight;

    // helper to wait one and accumulate
    auto finish_one = [&]() {
        auto& front = inflight.front();
        cv::Mat tile2 = front.fut.get();
        add(tile2, bigImage_out(front.roi), bigImage_out(front.roi));
        inflight.pop_front();
    };

    for (int y = 0; y < (bigImage.rows - tileSize); y += step) {
        for (int x = 0; x < (bigImage.cols - tileSize); x += step) {

            cv::Rect tileRegion(x, y, tileSize, tileSize);
            cv::Mat tile = bigImage(tileRegion).clone();

            // IMPORTANT: pass heavy args by reference to avoid copies
            auto fut = std::async(
                std::launch::async,
                NN_noise_reduction_tile_mono,
                std::cref(model),            // <-- no model copy
                std::move(tile),             // tile by value (moved)
                std::cref(window2d)          // <-- no window copy
            );

            inflight.push_back(Inflight{ tileRegion, std::move(fut) });

            // throttle: keep at most maxThreads tasks running
            if ((int)inflight.size() >= maxThreads) {
                finish_one();
            }
        }
    }

    // drain remaining
    while (!inflight.empty()) finish_one();
    /**/





    //cout << "test..." << endl;

    int width = image.cols;
    int height = image.rows;
    //Rect cropRegion(0, 0, width, height);
    //Rect cropRegion(tileSize - overlap, tileSize - overlap, width, height);
    Rect cropRegion(overlap, overlap, width, height);
    if (mix > 0.99)
        image = bigImage_out(cropRegion);
    else
        image = image * (1.0 - mix) + bigImage_out(cropRegion) * mix;

    //bigImage_out.copyTo(image);

    //cout << "end NN filter mono" << endl;
    if (debug_flag == 1) {
        //cout << "end NN filter..." << endl;
        logfile << "end NN filter mono..." << endl;
    }


    /**/
}



void NN_noise_reduction_mono_onnx(cv::dnn::Net net, Mat& image, double mix) {

    // prepeare window
    int N = 128;  // Set the desired size for the window
    Mat window1d(N, 1, CV_32F); // Create a 1D window matrix

    // Populate 1D window, overlap 16
    for (int i = 0; i < N; i++) {
        if (i == 0) window1d.at<float>(i, 0) = 0;
        else if (i == 1) window1d.at<float>(i, 0) = 0;
        else if (i == 2) window1d.at<float>(i, 0) = 0;
        else if (i == 3) window1d.at<float>(i, 0) = 0;
        else if (i == 4) window1d.at<float>(i, 0) = 0;
        else if (i == 5) window1d.at<float>(i, 0) = 0;
        else if (i == 6) window1d.at<float>(i, 0) = 0;
        else if (i == 7) window1d.at<float>(i, 0) = 0.2;
        else if (i == 8) window1d.at<float>(i, 0) = 0.8;
        else if (i == 9) window1d.at<float>(i, 0) = 1.0;
        else if (i == (N - 10)) window1d.at<float>(i, 0) = 1.0;
        else if (i == (N - 9)) window1d.at<float>(i, 0) = 0.8;
        else if (i == (N - 8)) window1d.at<float>(i, 0) = 0.2;
        else if (i == (N - 7)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 6)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 5)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 4)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 3)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 2)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 1)) window1d.at<float>(i, 0) = 0;
        else window1d.at<float>(i, 0) = 1.0;

    }

    // Create a 2D window by taking the outer product of the 1D window with itself
    Mat window2d = window1d * window1d.t();

    if (debug_flag == 1) {
        //cout << "start NN filter mono onnx" << endl;
        logfile << "start NN filter mono onnx" << endl;
    }

    int tileSize = N;  // see window definition
    int overlap = 16;
    int step = tileSize - overlap;


    Mat bigImage;
    image.copyTo(bigImage);
    copyMakeBorder(image, bigImage, overlap, tileSize - overlap, overlap, tileSize - overlap, BORDER_REPLICATE, Scalar::all(0));

    //define black image for output
    Mat bigImage_out(bigImage.rows, bigImage.cols, CV_32FC1, Scalar::all(0));



    // Batch buffers
    const int BATCH = 4; // tune 8–32
    std::vector<cv::Mat> batchImgs;
    batchImgs.reserve(BATCH);
    std::vector<cv::Rect> rois;
    rois.reserve(BATCH);



    auto flush_batch = [&]() {
        if (batchImgs.empty()) return;

        // OpenCV expects NCHW by default: blobFromImages will convert NHWC->NCHW when swapRB=false
        cv::Mat blob = cv::dnn::blobFromImages(
            batchImgs,    // vector<Mat> NHWC float32
            1.0,          // scale already applied in preprocess()
            cv::Size(N, N),
            cv::Scalar(), // mean already handled if needed
            false,        // swapRB (we are single-channel)
            false,        // crop
            CV_32F
        );
        net.setInput(blob);                  // shape: N x 1 x N x N
        cv::Mat outBlob = net.forward();    // shape: N x 1 x N x N
        //cv::Mat outBlob = blob;

        // Split batch outputs and blend
        int n = outBlob.size[0];
        for (int i = 0; i < n; ++i) {
            // get 1 x 1 x N x N plane as Mat
            cv::Mat one(N, N, CV_32F, outBlob.ptr<float>(i, 0));
            cv::Mat win; cv::multiply(one, window2d, win);
            //win.copyTo(bigImage_out(rois[i]));  // or add() into accumulator if you also keep a weight_sum
            add(win, bigImage_out(rois[i]), bigImage_out(rois[i]));
        }
        batchImgs.clear(); rois.clear();
        };

    // Prepare tiles (NHWC single-channel float)
    for (int y = 0; y <= bigImage.rows - N; y += step) {
        for (int x = 0; x <= bigImage.cols - N; x += step) {
            cv::Rect r(x, y, N, N);
            cv::Mat tile = bigImage(r).clone();   // clone because we’ll feed it
            // here we window the OUTPUT, not the input)
            batchImgs.emplace_back(tile);
            rois.emplace_back(r);
            if ((int)batchImgs.size() == BATCH) flush_batch();
        }
    }
    flush_batch();


    int width = image.cols;
    int height = image.rows;
    Rect cropRegion(overlap, overlap, width, height);
    if (mix > 0.99)
        image = bigImage_out(cropRegion);
    else
        image = image * (1.0 - mix) + bigImage_out(cropRegion) * mix;

    if (debug_flag == 1) {
        //cout << "end NN filter mono onnx" << endl;
        logfile << "end NN filter mono onnx" << endl;
    }
}


void save_sub_image(Mat& image)
{

    cout << "Saving sub image..." << endl;

    time_t t = time(0);   // get time now
    struct tm* now = localtime(&t);
    char filename[80];

    Mat mat16uc3;

    strftime(filename, 80, "subs/sub_%Y-%m-%d_%H-%M-%S.tiff", now);

    image.convertTo(mat16uc3, CV_16UC3, 65535);
    imwrite(filename, mat16uc3);

    cout << "Sub images saved" << endl;
}