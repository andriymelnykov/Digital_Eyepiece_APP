void read_fits_file(char* filename, unsigned char* image_buffer) {
    fitsfile* fptr;  // FITS file pointer
    int status = 0;  // CFITSIO status value MUST be initialized to zero!
    int bitpix, naxis;
    long naxes[2] = { 1,1 }, fpixel[2] = { 1,1 };
    char err_text[80];

    // Open the FITS file
    fits_open_file(&fptr, filename, READONLY, &status);
    if (status) {
        fits_get_errstatus(status, err_text); // get mnemonic of error message
        cout << "Fits lib error, reading " << filename << ": " << err_text << endl;
        logfile << "Fits lib error, reading " << filename << ": " << err_text << endl;
        abort_app();
    }

    // Read the image parameters
    fits_get_img_param(fptr, 2, &bitpix, &naxis, naxes, &status);
    if (status) {
        fits_get_errstatus(status, err_text); // get mnemonic of error message
        cout << "Fits lib error, reading " << filename << ": " << err_text << endl;
        logfile << "Fits lib error, reading " << filename << ": " << err_text << endl;
        abort_app();
    }

    // Check if it's a 2D image
    if (naxis != 2) {
        cout << "Error, reading " << filename << ": not a 2D image" << endl;
        logfile << "Error, reading " << filename << ": not a 2D image" << endl;
        abort_app();
    }

    // Check image data type
    if (bitpix != 16) {
        cout << "Error, reading " << filename << ": wrong data type" << endl;
        logfile << "Error, reading " << filename << ": wrong data type" << endl;
        abort_app();
    }

    // Check image dimensions
    if ((naxes[0] != camera_image_width) || (naxes[1] != camera_image_height)) {
        cout << "Error, reading " << filename << ": wrong image dimensions" << endl;
        cout << filename << " dimensions: " << naxes[0] << "x" << naxes[1] << endl;
        cout << "Camera image dimensions: " << camera_image_width << "x" << camera_image_height << endl;
        logfile << "Error, reading " << filename << ": wrong image dimensions" << endl;
        logfile << filename << " dimensions: " << naxes[0] << "x" << naxes[1] << endl;
        logfile << "Camera image dimensions: " << camera_image_width << "x" << camera_image_height << endl;
        abort_app();
    }

    // Read the image into the buffer
    long imageSize = naxes[0] * naxes[1];
    fits_read_pix(fptr, TUSHORT, fpixel, imageSize, NULL, image_buffer, NULL, &status);
    if (status) {
        fits_get_errstatus(status, err_text); // get mnemonic of error message
        cout << "Fits lib error, reading " << filename << ": " << err_text << endl;
        logfile << "Fits lib error, reading " << filename << ": " << err_text << endl;
        abort_app();
    }

    fits_close_file(fptr, &status);
    if (status) {
        fits_get_errstatus(status, err_text); // get mnemonic of error message
        cout << "Fits lib error, reading " << filename << ": " << err_text << endl;
        logfile << "Fits lib error, reading " << filename << ": " << err_text << endl;
        abort_app();
    }
}


void read_darks(Mat& dark_v_32sc1, double& dark_v_mean, Mat& dark_f_32sc1, double& dark_f_mean)
{
    double num_pixel;
    //ifstream myfile;

    if ((dark_v_hotpixel_flag == 1) || (dark_v_subtract_flag == 1)) {

        //char filename[] = "dark_v.fits";
        dark_v_image = (unsigned char*)malloc(sizeof(unsigned char) * image_size);

        //cout << "Reading dark_v.fits..." << endl;
        //logfile << "Reading dark_v.fits..." << endl;
        cout << "Reading " << dark_v_filename << "..." << endl;
        logfile << "Reading " << dark_v_filename << "..." << endl;
        //read_fits_file(filename, dark_v_image); dark_v_filename
        read_fits_file(dark_v_filename, dark_v_image);


        // ----------- simple read fits file
        /*
        myfile.open("dark_v.fits", ios::in | ios::binary);

        if (myfile.is_open()) {
            printf("Reading dark_v.fits...\n");
            myfile.seekg(2880, ios::beg);
            myfile.read((char*)dark_v_image, image_size);
            myfile.close();

            int16_t* p = (int16_t*)dark_v_image;
            uint16_t* p2 = (uint16_t*)dark_v_image;

            for (long i = 0; i < (image_size / 2); i++) {
                unsigned char t = dark_v_image[i * 2];
                dark_v_image[i * 2] = dark_v_image[i * 2 + 1];
                dark_v_image[i * 2 + 1] = t;
                p2[i] = (uint16_t)((int32_t)p[i] + 32768);   // see how unsigned 16 bit is stored as signed + offset in FITS file format
            }
        }
        else {
            printf("Couldn't find file dark_v.fits\n");
            cout << "Press Enter to close...";
            cin.get();
            exit(1); // return 1;
        }
        /**/
        // ----------- 




        // Copy the data into an OpenCV Mat structure
        //Mat dark_v_16uc1(asi_camera_info[cam]->MaxWidth / bin / monobin_k, asi_camera_info[cam]->MaxHeight / bin / monobin_k, CV_16UC1, dark_v_image);
        Mat dark_v_16uc1(camera_image_height, camera_image_width, CV_16UC1, dark_v_image);

        // Convert to int32
        dark_v_16uc1.convertTo(dark_v_32sc1, CV_32SC1);

        //printf("size: %d, %d\n", dark_v_32sc1.rows, dark_v_32sc1.cols);
        //imshow("dark_v", dark_v_32sc1);
        //printf("pixel: %d, %d\n", dark_v_16uc1.at<uint16_t>(500, 500), dark_v_32sc1.at<int32_t>(500, 500));

        SigmaClippedStats s_v = sigmaClippedMeanStddev(dark_v_32sc1, 5.0, 10);

        //std::cout << "dark_v sigma clipped mean   = " << s_v.mean << "\n";
        //std::cout << "dark_v sigma clipped stddev = " << s_v.stddev << "\n";

        //MedianMadStats s_mv = medianMadStats(dark_v_32sc1);

        //std::cout << "dark_v median   = " << s_mv.median << "\n";
        //std::cout << "dark_v sigmaMad = " << s_mv.sigmaMad << "\n";

        //-----------Calculate dark mean and standard deviation value
        //Scalar mean, stddev;
        //meanStdDev(dark_v_32sc1, mean, stddev);
        dark_v_mean = s_v.mean;    //dark_v_mean = mean[0];
        dark_v_stdev = s_v.stddev; //dark_v_stdev = stddev[0];
        hotpixel_threshold = dark_v_mean + dark_v_stdev * hot_pixel_sigma; //7;
        //coldpixel_threshold = dark_v_mean - dark_v_stdev * hot_pixel_sigma;

        if (debug_flag == 1) {
            cout << "Dark v mean value: " << dark_v_mean << endl;
            cout << "Dark v stdev value: " << dark_v_stdev << endl;
            cout << "Dark v hotpixel threshold value: " << hotpixel_threshold << endl;
            logfile << "Dark v mean value: " << dark_v_mean << endl;
            logfile << "Dark v stdev value: " << dark_v_stdev << endl;
            logfile << "Dark v hotpixel threshold value: " << hotpixel_threshold << endl;
        }

        //-----------Search and count hot pixels
        num_hotpixel_v = 0;

        for (int i = 0; i < dark_v_32sc1.rows; i++)
            for (int j = 0; j < dark_v_32sc1.cols; j++) {
                //if ((dark_v_32sc1.at<int32_t>(i, j) > hotpixel_threshold) || (dark_v_32sc1.at<int32_t>(i, j) < coldpixel_threshold)) {
                if (dark_v_32sc1.at<int32_t>(i, j) > hotpixel_threshold) {
                    //printf("Hot pixel at: %d, %d\n", i, j);
                    //printf("pixel: %d, %d\n", dark_v_16uc1.at<uint16_t>(i, j), dark_v_32sc1.at<int32_t>(i, j));
                    num_hotpixel_v++;
                }

            }

        if (debug_flag == 1) {
            cout << "Hot pixels found in video dark frame: " << num_hotpixel_v << endl;
            logfile << "Hot pixels found in video dark frame: " << num_hotpixel_v << endl;
        }

        //-----------Search and list hot pixels
        hotpixel_list_v = (int*)malloc(sizeof(int) * num_hotpixel_v * 10);
        num_hotpixel_v = 0;

        for (int i = 0; i < dark_v_32sc1.rows; i++)
            for (int j = 0; j < dark_v_32sc1.cols; j++) {
                if (dark_v_32sc1.at<int32_t>(i, j) > hotpixel_threshold) {
                    hotpixel_list_v[num_hotpixel_v * 10 + 0] = i; //save coordinates of hotpixel
                    hotpixel_list_v[num_hotpixel_v * 10 + 1] = j;

                    //printf("Hot pixels num, at: %d, %d, %d\n", num_hotpixel_v, hotpixel_list_v[num_hotpixel_v *10 + 0], hotpixel_list_v[num_hotpixel_v *10 + 1]);

                    /*
                    if ((i > 1) && (i < (dark_v_32sc1.rows - 2)) &&
                        (j > 1) && (j < (dark_v_32sc1.cols - 2)) &&
                        (dark_v_32sc1.at<int32_t>( (i-2), (j-2) ) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 2] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 2] = 0;
                    //-----
                    /**/
                    //-----
                    if ((i > 1) &&
                        (j > 1) &&
                        (dark_v_32sc1.at<int32_t>((i - 2), (j - 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 2] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 2] = 0;
                    //-----
                    if ((i > 1) &&
                        (dark_v_32sc1.at<int32_t>((i - 2), (j)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 3] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 3] = 0;
                    //-----
                    if ((i > 1) &&
                        (j < (dark_v_32sc1.cols - 2)) &&
                        (dark_v_32sc1.at<int32_t>((i - 2), (j + 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 4] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 4] = 0;
                    //-----
                    if ((j < (dark_v_32sc1.cols - 2)) &&
                        (dark_v_32sc1.at<int32_t>((i), (j + 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 5] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 5] = 0;
                    //-----
                    if ((i < (dark_v_32sc1.rows - 2)) &&
                        (j < (dark_v_32sc1.cols - 2)) &&
                        (dark_v_32sc1.at<int32_t>((i + 2), (j + 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 6] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 6] = 0;
                    //-----
                    if ((i < (dark_v_32sc1.rows - 2)) &&
                        (dark_v_32sc1.at<int32_t>((i + 2), (j)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 7] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 7] = 0;
                    //-----
                    if ((i < (dark_v_32sc1.rows - 2)) &&
                        (j > 1) &&
                        (dark_v_32sc1.at<int32_t>((i + 2), (j - 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 8] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 8] = 0;
                    //-----
                    if ((j > 1) &&
                        (dark_v_32sc1.at<int32_t>((i), (j - 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 9] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 9] = 0;
                    //-----
                    /**/
                    num_hotpixel_v++;
                }

            }

        /*
        //int d = 458;
        for (int d = 0; d < 200; d++) //num_hotpixel_v; d++)
            printf("Hot pixels coord: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", hotpixel_list_v[d*10 + 0], hotpixel_list_v[d*10 + 1],
                hotpixel_list_v[d*10 + 2], hotpixel_list_v[d*10 + 3], hotpixel_list_v[d*10 + 4], hotpixel_list_v[d*10 + 5],
                hotpixel_list_v[d*10 + 6], hotpixel_list_v[d*10 + 7], hotpixel_list_v[d*10 + 8], hotpixel_list_v[d*10 + 9]);
        /**/

        if (debug_flag == 1) {
            cout << "Hot pixels in video dark listed" << endl;
            logfile << "Hot pixels in video dark listed" << endl;
        }

        /*
        //-----------Calculate dark mean value
        num_pixel = 0;
        dark_v_mean = 0;

        for (int i = 0; i < dark_v_32sc1.rows; i++)
            for (int j = 0; j < dark_v_32sc1.cols; j++) {
                if (dark_v_32sc1.at<int32_t>(i, j) < hotpixel_threshold) {
                    dark_v_mean += dark_v_32sc1.at<int32_t>(i, j);
                    num_pixel += 1;
                }

            }
        dark_v_mean = dark_v_mean / num_pixel;
        printf("Dark v mean value: %f\n", dark_v_mean);
        /**/
    }






    if ((dark_f_hotpixel_flag == 1) || (dark_f_subtract_flag == 1)) {

        //char filename[] = "dark_f.fits";
        dark_f_image = (unsigned char*)malloc(sizeof(unsigned char) * image_size);

        //cout << "Reading dark_f.fits..." << endl;
        //logfile << "Reading dark_f.fits..." << endl;
        cout << "Reading " << dark_f_filename << "..." << endl;
        logfile << "Reading " << dark_f_filename << "..." << endl;
        //read_fits_file(filename, dark_f_image);
        read_fits_file(dark_f_filename, dark_f_image);



        /*
        myfile.open("dark_f.fits", ios::in | ios::binary);

        if (myfile.is_open()) {
            printf("Reading dark_f.fits...\n");
            myfile.seekg(2880, ios::beg);
            myfile.read((char*)dark_f_image, image_size);
            myfile.close();

            int16_t* p = (int16_t*)dark_f_image;
            uint16_t* p2 = (uint16_t*)dark_f_image;

            for (long i = 0; i < (image_size / 2); i++) {
                unsigned char t = dark_f_image[i * 2];
                dark_f_image[i * 2] = dark_f_image[i * 2 + 1];
                dark_f_image[i * 2 + 1] = t;
                p2[i] = (uint16_t)((int32_t)p[i] + 32768);   // see how unsigned 16 bit is stored as signed + offset in FITS file format
            }
        }
        else {
            printf("Couldn't find file dark_f.fits\n");
            cout << "Press Enter to close...";
            cin.get();
            exit(1); // return 1;
        }
        /**/


        // Copy the data into an OpenCV Mat structure
        Mat dark_f_16uc1(camera_image_height, camera_image_width, CV_16UC1, dark_f_image);

        // Convert to int32
        dark_f_16uc1.convertTo(dark_f_32sc1, CV_32SC1);

        SigmaClippedStats s_f = sigmaClippedMeanStddev(dark_f_32sc1, 5.0, 10);

        //std::cout << "dark_f sigma clipped mean   = " << s_f.mean << "\n";
        //std::cout << "dark_f sigma clipped stddev = " << s_f.stddev << "\n";

        //MedianMadStats s_mf = medianMadStats(dark_f_32sc1);

        //std::cout << "dark_f median   = " << s_mf.median << "\n";
        //std::cout << "dark_f sigmaMad = " << s_mf.sigmaMad << "\n";

        //-----------Calculate dark mean and standard deviation value
        //Scalar mean, stddev;
        //meanStdDev(dark_f_32sc1, mean, stddev);
        dark_f_mean = s_f.mean;    // dark_f_mean = mean[0];
        dark_f_stdev = s_f.stddev; // dark_f_stdev = stddev[0];
        hotpixel_threshold = dark_f_mean + dark_f_stdev * hot_pixel_sigma; // 7;
        //coldpixel_threshold = dark_f_mean - dark_f_stdev * hot_pixel_sigma;

        if (debug_flag == 1) {
            cout << "Dark f mean value: " << dark_f_mean << endl;
            cout << "Dark f stdev value: " << dark_f_stdev << endl;
            cout << "Dark f hotpixel threshold value: " << hotpixel_threshold << endl;
            logfile << "Dark f mean value: " << dark_f_mean << endl;
            logfile << "Dark f stdev value: " << dark_f_stdev << endl;
            logfile << "Dark f hotpixel threshold value: " << hotpixel_threshold << endl;
        }


        //-----------Search and count hot pixels
        num_hotpixel_f = 0;

        for (int i = 0; i < dark_f_32sc1.rows; i++)
            for (int j = 0; j < dark_f_32sc1.cols; j++) {
                //if ((dark_f_32sc1.at<int32_t>(i, j) > hotpixel_threshold) || (dark_f_32sc1.at<int32_t>(i, j) < coldpixel_threshold)) {
                if (dark_f_32sc1.at<int32_t>(i, j) > hotpixel_threshold) {
                    num_hotpixel_f++;
                }

            }

        if (debug_flag == 1) {
            cout << "Hot pixels found in foto dark frame: " << num_hotpixel_f << endl;
            logfile << "Hot pixels found in foto dark frame: " << num_hotpixel_f << endl;
        }


        //-----------Search and list hot pixels
        hotpixel_list_f = (int*)malloc(sizeof(int) * num_hotpixel_f * 10);
        num_hotpixel_f = 0;

        for (int i = 0; i < dark_f_32sc1.rows; i++)
            for (int j = 0; j < dark_f_32sc1.cols; j++) {
                if (dark_f_32sc1.at<int32_t>(i, j) > hotpixel_threshold) {
                    hotpixel_list_f[num_hotpixel_f * 10 + 0] = i; //save coordinates of hotpixel
                    hotpixel_list_f[num_hotpixel_f * 10 + 1] = j;


                    //-----
                    if ((i > 1) &&
                        (j > 1) &&
                        (dark_f_32sc1.at<int32_t>((i - 2), (j - 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 2] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 2] = 0;
                    //-----
                    if ((i > 1) &&
                        (dark_f_32sc1.at<int32_t>((i - 2), (j)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 3] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 3] = 0;
                    //-----
                    if ((i > 1) &&
                        (j < (dark_f_32sc1.cols - 2)) &&
                        (dark_f_32sc1.at<int32_t>((i - 2), (j + 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 4] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 4] = 0;
                    //-----
                    if ((j < (dark_f_32sc1.cols - 2)) &&
                        (dark_f_32sc1.at<int32_t>((i), (j + 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 5] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 5] = 0;
                    //-----
                    if ((i < (dark_f_32sc1.rows - 2)) &&
                        (j < (dark_f_32sc1.cols - 2)) &&
                        (dark_f_32sc1.at<int32_t>((i + 2), (j + 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 6] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 6] = 0;
                    //-----
                    if ((i < (dark_f_32sc1.rows - 2)) &&
                        (dark_f_32sc1.at<int32_t>((i + 2), (j)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 7] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 7] = 0;
                    //-----
                    if ((i < (dark_f_32sc1.rows - 2)) &&
                        (j > 1) &&
                        (dark_f_32sc1.at<int32_t>((i + 2), (j - 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 8] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 8] = 0;
                    //-----
                    if ((j > 1) &&
                        (dark_f_32sc1.at<int32_t>((i), (j - 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 9] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 9] = 0;
                    //-----
                    /**/
                    num_hotpixel_f++;
                }

            }

        if (debug_flag == 1) {
            cout << "Hot pixels in foto dark listed" << endl;
            logfile << "Hot pixels in foto dark listed" << endl;
        }


        /*
        //-----------Calculate dark mean value
        num_pixel = 0;
        dark_f_mean = 0;

        for (int i = 0; i < dark_f_32sc1.rows; i++)
            for (int j = 0; j < dark_f_32sc1.cols; j++) {
                if (dark_f_32sc1.at<int32_t>(i, j) < hotpixel_threshold) {
                    dark_f_mean += dark_f_32sc1.at<int32_t>(i, j);
                    num_pixel += 1;
                }

            }
        dark_f_mean = dark_f_mean / num_pixel;
        printf("Dark f mean value: %f\n", dark_f_mean);
        /**/
    }



}


void add_frame_hotpixels(Mat& frame, double dark_stdev)
{


    //if ((dark_f_hotpixel_flag == 1) || (dark_f_subtract_flag == 1)) {
    if (true) {

        //int32_t hotpixel_threshold = dark_stdev * 10;
        int32_t hotpixel_threshold = 65000 / bin / bin / 2;

        if (debug_flag == 1) {
            cout << "Additional hotpixel threshold value: " << hotpixel_threshold << endl;
            logfile << "Additional hotpixel threshold value: " << hotpixel_threshold << endl;
        }


        //-----------Search and count hot pixels
        num_hotpixel_add = 0;

        for (int i = 0; i < frame.rows; i++)
            for (int j = 0; j < frame.cols; j++) {

                int32_t sum = 0;
                int n = 0;

                if ((i > 1) && (j > 1) && (i < (frame.rows - 2)) && (j < (frame.cols - 2))) {
                    sum = frame.at<int32_t>((i - 2), (j - 2)) +
                        frame.at<int32_t>((i + 2), (j + 2)) +
                        frame.at<int32_t>((i - 2), (j + 2)) +
                        frame.at<int32_t>((i + 2), (j - 2)) +
                        frame.at<int32_t>((i - 2), (j)) +
                        frame.at<int32_t>((i + 2), (j)) +
                        frame.at<int32_t>((i), (j - 2)) +
                        frame.at<int32_t>((i), (j + 2));
                    n = 8;
                }

                if ((n > 0) && (frame.at<int32_t>(i, j) > (sum / 8 + hotpixel_threshold)) && ((sum / 8) < 13000)) {
                    num_hotpixel_add++;
                }

                // for test
                //if (i == 61 && j == 282)
                //    cout << sum << "  " << frame.at<int32_t>(i, j) << endl;

            }

        if (debug_flag == 1) {
            cout << "Additional hot pixels found in frame: " << num_hotpixel_add << endl;
            logfile << "Additional hot pixels found in frame: " << num_hotpixel_add << endl;
        }


        //-----------Search and list hot pixels
        hotpixel_list_add = (int*)malloc(sizeof(int) * num_hotpixel_add * 10);

        num_hotpixel_add = 0;

        for (int i = 0; i < frame.rows; i++)
            for (int j = 0; j < frame.cols; j++) {

                int64_t sum = 0;
                int n = 0;

                if ((i > 1) && (j > 1) && (i < (frame.rows - 2)) && (j < (frame.cols - 2))) {
                    sum = frame.at<int32_t>((i - 2), (j - 2)) +
                        frame.at<int32_t>((i + 2), (j + 2)) +
                        frame.at<int32_t>((i - 2), (j + 2)) +
                        frame.at<int32_t>((i + 2), (j - 2)) +
                        frame.at<int32_t>((i - 2), (j)) +
                        frame.at<int32_t>((i + 2), (j)) +
                        frame.at<int32_t>((i), (j - 2)) +
                        frame.at<int32_t>((i), (j + 2));
                    n = 8;
                }

                if ((n > 0) && (frame.at<int32_t>(i, j) > (sum / 8 + hotpixel_threshold)) && ((sum / 8) < 13000)) {

                    hotpixel_list_add[num_hotpixel_add * 10 + 0] = i; //save coordinates of hotpixel
                    hotpixel_list_add[num_hotpixel_add * 10 + 1] = j;


                    //-----
                    if ((i > 1) &&
                        (j > 1) &&
                        (frame.at<int32_t>((i - 2), (j - 2)) < (sum / 8 + hotpixel_threshold))) {   // valid neighbor pixel for interpolation
                        hotpixel_list_add[num_hotpixel_add * 10 + 2] = 1;
                    }
                    else
                        hotpixel_list_add[num_hotpixel_add * 10 + 2] = 0;
                    //-----
                    if ((i > 1) &&
                        (frame.at<int32_t>((i - 2), (j)) < (sum / 8 + hotpixel_threshold))) {   // valid neighbor pixel for interpolation
                        hotpixel_list_add[num_hotpixel_add * 10 + 3] = 1;
                    }
                    else
                        hotpixel_list_add[num_hotpixel_add * 10 + 3] = 0;
                    //-----
                    if ((i > 1) &&
                        (j < (frame.cols - 2)) &&
                        (frame.at<int32_t>((i - 2), (j + 2)) < (sum / 8 + hotpixel_threshold))) {   // valid neighbor pixel for interpolation
                        hotpixel_list_add[num_hotpixel_add * 10 + 4] = 1;
                    }
                    else
                        hotpixel_list_add[num_hotpixel_add * 10 + 4] = 0;
                    //-----
                    if ((j < (frame.cols - 2)) &&
                        (frame.at<int32_t>((i), (j + 2)) < (sum / 8 + hotpixel_threshold))) {   // valid neighbor pixel for interpolation
                        hotpixel_list_add[num_hotpixel_add * 10 + 5] = 1;
                    }
                    else
                        hotpixel_list_add[num_hotpixel_add * 10 + 5] = 0;
                    //-----
                    if ((i < (frame.rows - 2)) &&
                        (j < (frame.cols - 2)) &&
                        (frame.at<int32_t>((i + 2), (j + 2)) < (sum / 8 + hotpixel_threshold))) {   // valid neighbor pixel for interpolation
                        hotpixel_list_add[num_hotpixel_add * 10 + 6] = 1;
                    }
                    else
                        hotpixel_list_add[num_hotpixel_add * 10 + 6] = 0;
                    //-----
                    if ((i < (frame.rows - 2)) &&
                        (frame.at<int32_t>((i + 2), (j)) < (sum / 8 + hotpixel_threshold))) {   // valid neighbor pixel for interpolation
                        hotpixel_list_add[num_hotpixel_add * 10 + 7] = 1;
                    }
                    else
                        hotpixel_list_add[num_hotpixel_add * 10 + 7] = 0;
                    //-----
                    if ((i < (frame.rows - 2)) &&
                        (j > 1) &&
                        (frame.at<int32_t>((i + 2), (j - 2)) < (sum / 8 + hotpixel_threshold))) {   // valid neighbor pixel for interpolation
                        hotpixel_list_add[num_hotpixel_add * 10 + 8] = 1;
                    }
                    else
                        hotpixel_list_add[num_hotpixel_add * 10 + 8] = 0;
                    //-----
                    if ((j > 1) &&
                        (frame.at<int32_t>((i), (j - 2)) < (sum / 8 + hotpixel_threshold))) {   // valid neighbor pixel for interpolation
                        hotpixel_list_add[num_hotpixel_add * 10 + 9] = 1;
                    }
                    else
                        hotpixel_list_add[num_hotpixel_add * 10 + 9] = 0;
                    //-----

                    num_hotpixel_add++;
                }

            }

        if (debug_flag == 1) {
            cout << "Additional hot pixels in frame listed" << endl;
            logfile << "Additional hot pixels in frame listed" << endl;
        }


    }



}


void correct_hotpixel(Mat& image, int* hotpixel_list, int num_hotpixel)
{

    if (debug_flag == 1) {
        cout << "Hotpixels correction" << endl;
        logfile << "Hotpixels correction" << endl;
    }

    //printf("list test: %d, %d, %d\n", hotpixel_list[0], hotpixel_list[1], hotpixel_list[2]);

    for (int i = 0; i < num_hotpixel; i++) {

        int n = 0;
        int32_t sum = 0;

        if (hotpixel_list[i * 10 + 2] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0] - 2, hotpixel_list[i * 10 + 1] - 2);
            n++;
        }
        if (hotpixel_list[i * 10 + 3] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0] - 2, hotpixel_list[i * 10 + 1]);
            n++;
        }
        if (hotpixel_list[i * 10 + 4] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0] - 2, hotpixel_list[i * 10 + 1] + 2);
            n++;
        }
        if (hotpixel_list[i * 10 + 5] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0], hotpixel_list[i * 10 + 1] + 2);
            n++;
        }
        if (hotpixel_list[i * 10 + 6] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0] + 2, hotpixel_list[i * 10 + 1] + 2);
            n++;
        }
        if (hotpixel_list[i * 10 + 7] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0] + 2, hotpixel_list[i * 10 + 1]);
            n++;
        }
        if (hotpixel_list[i * 10 + 8] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0] + 2, hotpixel_list[i * 10 + 1] - 2);
            n++;
        }
        if (hotpixel_list[i * 10 + 9] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0], hotpixel_list[i * 10 + 1] - 2);
            n++;
        }


        if (n > 0)
            image.at<int32_t>(hotpixel_list[i * 10 + 0], hotpixel_list[i * 10 + 1]) = sum / n;  //correct hotpixel with interpolated value
        else
            image.at<int32_t>(hotpixel_list[i * 10 + 0], hotpixel_list[i * 10 + 1]) = 0;   // no neighbor pixel to interpolate from, set to black

    }

}


void apply_dark(Mat& image, Mat dark_image, double dark_mean)
{
    if (debug_flag == 1) {
        cout << "Apply dark" << endl;
        logfile << "Apply dark" << endl;
    }

    //printf("pixel before: %d\n", image.at<int32_t>(500, 500));
    //printf("pixel dark: %d\n", dark_image.at<int32_t>(500, 500));

    image = image - dark_image;
    //printf("pixel after dark: %d\n", image.at<int32_t>(500, 500));

    //cout << dark_mean << endl;
    //cout << (int32_t)dark_mean << endl;




    image = image + (int32_t)dark_mean;




    //printf("pixel after dark mean: %d\n", image.at<int32_t>(500, 500));

    //printf("dark_mean in int32: %d\n", (int32_t)dark_mean);
    //printf("pixel: %d\n", image.at<int32_t>(500, 500));


}


// Creates a float32 BGR mask (monochrome: B=G=R), values in [0,1].
// param: fraction of min dimension for the diameter of the "all ones" circle.
//        Example: W=1500,H=1000,param=0.8 => inner diameter = 0.8*1000 = 800 px.
Mat makeCosineVignetteMaskBGRf(int width, int height, float param)
{
    param = std::clamp(param, 0.0f, 1.0f);

    cv::Mat mask(height, width, CV_32FC3, cv::Scalar(0, 0, 0));

    const float minDim = static_cast<float>(std::min(width, height));
    const float R0 = 0.5f * param * minDim;   // inner radius where value=1
    const float R1 = 0.5f * minDim;           // radius at edge of min dimension

    // Handle degenerate transition (param==1 => R0==R1: all ones inside R1, else 0)
    const float denom = (R1 - R0);

    const float cx = (width - 1) * 0.5f;
    const float cy = (height - 1) * 0.5f;

    const float halfPi = static_cast<float>(CV_PI * 0.5);

    for (int y = 0; y < height; ++y)
    {
        cv::Vec3f* row = mask.ptr<cv::Vec3f>(y);
        const float dy = y - cy;

        for (int x = 0; x < width; ++x)
        {
            const float dx = x - cx;
            const float r = std::sqrt(dx * dx + dy * dy);

            float v = 0.0f;
            if (r <= R0)
            {
                v = 1.0f;
            }
            else if (r >= R1)
            {
                v = 0.0f;
            }
            else
            {
                if (denom <= 1e-6f)
                {
                    // param ~ 1: no transition region
                    v = 1.0f;
                }
                else
                {
                    const float t = (r - R0) / denom;      // 0..1
                    v = std::cos(t * halfPi);              // 1..0
                }
            }

            row[x] = cv::Vec3f(v, v, v); // B,G,R all equal
        }
    }

    return mask;
}


void read_flat(Mat& flat_32fc3, Mat& flat_inv_32fc3)
{


    //char filename[] = "flat.fits";
    flat_image = (unsigned char*)malloc(sizeof(unsigned char) * image_size);

    //cout << "Reading flat.fits..." << endl;
    //logfile << "Reading flat.fits..." << endl;
    cout << "Reading " << flat_filename << "..." << endl;
    logfile << "Reading " << flat_filename << "..." << endl;
    //read_fits_file(filename, flat_image);
    read_fits_file(flat_filename, flat_image);


    /*
    ifstream myfile;

    myfile.open("flat.fits", ios::in | ios::binary);

    if (myfile.is_open()) {
        printf("Reading flat.fits...\n");
        myfile.seekg(2880, ios::beg);
        myfile.read((char*)flat_image, image_size);
        myfile.close();
        //printf("flat.fits closed\n");

        int16_t* p = (int16_t*)flat_image;
        uint16_t* p2 = (uint16_t*)flat_image;

        for (long i = 0; i < (image_size / 2); i++) {
            unsigned char t = flat_image[i * 2];
            flat_image[i * 2] = flat_image[i * 2 + 1];
            flat_image[i * 2 + 1] = t;
            p2[i] = (uint16_t)((int32_t)p[i] + 32768);   // see how unsigned 16 bit is stored as signed + offset in FITS file format
        }

        //printf("Reading flat.fits done\n");
    }
    else {
        printf("Couldn't find file flat.fits\n");
        cout << "Press Enter to close...";
        cin.get();
        exit(1); // return 1;
    }
    /**/


    //--------------------Copy flat frame to Mat variables

    // Copy the data into an OpenCV Mat structure
    Mat flat_16uc1(camera_image_height, camera_image_width, CV_16UC1, flat_image);

    // Decode Bayer data to RGB
    //Mat mat16uc3_rgb(camera_image_height, camera_image_width, CV_16UC3);
    //cvtColor(flat_16uc1, mat16uc3_rgb, cv::COLOR_BayerRGGB2BGR);

    // Decode Bayer data to RGB or mix monochrome to RGB
    Mat mat16uc3_rgb(camera_image_height, camera_image_width, CV_16UC3);
    if (is_color_cam)
        cvtColor(flat_16uc1, mat16uc3_rgb, cv::COLOR_BayerRGGB2BGR);
    else
        cvtColor(flat_16uc1, mat16uc3_rgb, cv::COLOR_GRAY2BGR);

    // Convert to float32
    Mat flat_32fc3_temp(camera_image_height, camera_image_width, CV_32FC3);
    mat16uc3_rgb.convertTo(flat_32fc3_temp, CV_32FC3, 1 / 65536.0);

    //---------- normalise rgb planes to 1
    vector<Mat> bgr_planes;
    split(flat_32fc3_temp, bgr_planes);
    double min, max;
    minMaxLoc(bgr_planes[0], &min, &max);
    //printf("flat blue max: %f\n", max);
    bgr_planes[0] = bgr_planes[0] / max;

    minMaxLoc(bgr_planes[1], &min, &max);
    //printf("flat green max: %f\n", max);
    bgr_planes[1] = bgr_planes[1] / max;

    minMaxLoc(bgr_planes[2], &min, &max);
    //printf("flat red max: %f\n", max);
    bgr_planes[2] = bgr_planes[2] / max;

    merge(bgr_planes, flat_32fc3_temp);

    // Prepare map for re-applying vignetting AFTER stretch/background compensation.
    {
        float f = flat_inv_factor;
        if (f < 0.0f) f = 0.0f;
        //if (f > 1.0f) f = 1.0f;

        {
            // flat_revign_32fc3 = (1 - f) + f * flat_32fc3
            flat_revign_32fc3 = flat_32fc3_temp * f;
            cv::add(flat_revign_32fc3,
                Scalar(1.0f - f, 1.0f - f, 1.0f - f),
                flat_revign_32fc3);

            // For mono pipeline: precompute grayscale variant
            cvtColor(flat_revign_32fc3, flat_revign_32fc1, cv::COLOR_BGR2GRAY);
        }
    }

    // Set near zero values to 1.0
    for (int i = 0; i < flat_32fc3_temp.rows; ++i)
        for (int j = 0; j < flat_32fc3_temp.cols; ++j)
        {
            if (flat_32fc3_temp.at<Vec3f>(i, j)[0] < 0.1)
                flat_32fc3_temp.at<Vec3f>(i, j)[0] = 1.0;
            if (flat_32fc3_temp.at<Vec3f>(i, j)[1] < 0.1)
                flat_32fc3_temp.at<Vec3f>(i, j)[1] = 1.0;
            if (flat_32fc3_temp.at<Vec3f>(i, j)[2] < 0.1)
                flat_32fc3_temp.at<Vec3f>(i, j)[2] = 1.0;
        }


    //------------ inverse flat frame

    flat_32fc3_temp.copyTo(flat_inv_32fc3);

    divide(1, flat_inv_32fc3, flat_inv_32fc3);


    //non-inverse flat must be scaled to display image size
    //resize(flat_32fc3_temp, flat_32fc3, Size(0, 0), display_scale, display_scale, INTER_AREA);

    //non-inverse flat must not be scaled to display image size
    flat_32fc3_temp.copyTo(flat_32fc3);


    //----- generate mask for artificial circular vignetting
    {
        flat_circvign_32fc3 = makeCosineVignetteMaskBGRf(flat_32fc3_temp.cols, flat_32fc3_temp.rows, circ_vign_radius);

        float f = circ_vign_factor;
        if (f < 0.0f) f = 0.0f;

        flat_circvign_32fc3 = flat_circvign_32fc3 * f;
        cv::add(flat_circvign_32fc3,
            Scalar(1.0f - f, 1.0f - f, 1.0f - f),
            flat_circvign_32fc3);

        // Match the stack image geometry after post-flat crop/resize.
        cdk_square_resize_after_flat(flat_circvign_32fc3);

        // For mono pipeline: precompute grayscale variant
        cvtColor(flat_circvign_32fc3, flat_circvign_32fc1, cv::COLOR_BGR2GRAY);
    }

}
